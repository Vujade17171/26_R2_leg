/**
  ******************************************************************************
  * @file    arm_control.c
  * @brief   Robot-arm end-position control (3 motors, shared FDCAN bus)
  ******************************************************************************
  * Pipeline: arm_target --> arm_inverse --> arm_traj --> 3 motor controls.
  *   - Shoulder : AK80-9  (ID=1, MIT)
  *   - Elbow    : AK45-10 (ID=2, MIT)
  *   - Wrist    : Lingzu-05(ID=3, RobStride)
  * Design: keep the bas/ drivers untouched; only call their public APIs.
  *
  * Safety: on power-up the arm is HOLD at its actual current position.
  * It only starts moving after the first arm_cmd_new command.
  ******************************************************************************
  */
#include "arm_control.h"
#include "fdcan_drv.h"
#include "mit_motor.h"
#include "robstride.h"
#include "arm_kinematics.h"
#include "arm_traj.h"
#include "arm_gravity.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

extern FDCAN_HandleTypeDef hfdcan1;

/* ---- motor IDs ---- */
#define ARM_M1_ID   1   /* AK80-9   shoulder */
#define ARM_M2_ID   2   /* AK45-10  elbow    */
#define ARM_M3_ID   3   /* Lingzu05 wrist    */

/* ---- controller gains (reference project) ---- */
#define ARM_KP_SHOULDER  120.0f
#define ARM_KD_SHOULDER  2.5f
#define ARM_KP_ELBOW     150.0f
#define ARM_KD_ELBOW     2.5f
#define ARM_KP_WRIST     100.0f
#define ARM_KD_WRIST     1.0f

/* ---- trajectory / hold ---- */
#define ARM_TRAJ_TIME    2.0f    /* s, default point-to-point duration */
#define ARM_POS_TOL      0.02f   /* rad, arrival tolerance             */

/* ---- startup / runtime safety ---- */
#define ARM_STARTUP_TIMEOUT_MS   3000U
#define ARM_EL05_ENABLE_RETRY_MS 200U
#define ARM_FEEDBACK_TIMEOUT_MS  200U
#define ARM_TEMP_LIMIT_C         80.0f

/* L3 absolute-level calibration from two measured level poses:
 *   pose A: q0=-0.891321, q1=2.131716, q2=1.697161
 *   pose B: q0=-0.078448, q1=3.052980, q2=-0.071664
 * Enforcing the known 1:1 wrist relation q0 = c - (q1 + q2)
 * gives c = 2.920211 rad.
 */
#define ARM_L3_LEVEL_C           2.9202114f
#define ARM_L3_MAX_SPEED         4.0f
#define ARM_L3_MAX_STEP_RAD      0.04f

/* ---- debug globals (watch in Keil) ---- */
arm_dbg_t arm_dbg = {0};
arm_xy_error_dbg_t arm_xy_err = {0};
volatile float    arm_target[3] = {0.35f, 0.25f, 0.0f};  /* x,z,yaw */
volatile uint8_t  arm_cmd_new  = 0;

volatile uint8_t  arm_gravity_test = 0U;

volatile uint8_t  arm_gravity_hold_enable = 0;

static FDCAN_HandleTypeDef *s_hfdcan = NULL;
static uint32_t s_last_tick = 0;
static float    s_dt        = 0.01f;
static uint8_t  s_inited    = 0;
static uint8_t  s_target_set = 0;   /* 0 until first target command */
static uint8_t  s_fault     = 0;    /* 1 = safety stop latched */
static uint32_t s_last_run_ms = 0;
static uint32_t s_next_el05_enable_ms = 0;
static uint8_t  s_gravity_hold_valid = 0U;
static float    s_gravity_hold_q[3] = {0.0f, 0.0f, 0.0f};
static float    s_q0_cmd = 0.0f;
static uint8_t  s_l3_level_active = 0U;

/* live joint state (feedback converted to joint angles) */
static float s_cur_joint[3] = {0, 0, 0};   /* q0,wrist q1,shoulder q2,elbow */
static float s_tgt_joint[3] = {0, 0, 0};   /* current commanded target       */

/* -------- register the 3 motor configs (do not enable here) -------- */
static uint8_t arm_setup_motors(FDCAN_HandleTypeDef *hfdcan)
{
    mit_motor_cfg_t mcfg;
    robstride_cfg_t rcfg;

    /* 1) bus init (filters + start) */
    if (fdcan_drv_init(hfdcan) != 0U) { return 1U; }

    /* 2) init drivers (register rx callback) */
    if (mit_motor_init(hfdcan) != 0U) { return 1U; }
    if (robstride_init(hfdcan) != 0U) { return 1U; }

    /* 3) AK80-9  shoulder */
    mcfg.id = ARM_M1_ID;
    mcfg.p_min = -12.5f; mcfg.p_max = 12.5f;
    mcfg.v_min = -50.0f; mcfg.v_max = 50.0f;
    mcfg.t_min = -18.0f; mcfg.t_max = 18.0f;
    mcfg.kp_min = 0.0f;  mcfg.kp_max = 500.0f;
    mcfg.kd_min = 0.0f;  mcfg.kd_max = 5.0f;
    mcfg.sign   = +1;    /* direction handled by arm_joint_to_motor_1 */
    if (mit_motor_add(&mcfg) < 0) { return 1U; }

    /* 4) AK45-10 elbow (direction handled by arm offset) */
    mcfg.id = ARM_M2_ID;
    mcfg.p_min = -12.6f; mcfg.p_max = 12.6f;
    mcfg.v_min = -8.0f;  mcfg.v_max = 8.0f;
    mcfg.t_min = -7.0f;  mcfg.t_max = 7.0f;
    mcfg.kp_min = 0.0f;  mcfg.kp_max = 500.0f;
    mcfg.kd_min = 0.0f;  mcfg.kd_max = 5.0f;
    mcfg.sign   = +1;
    if (mit_motor_add(&mcfg) < 0) { return 1U; }

    /* 5) Lingzu-05 wrist (RobStride extended frame) */
    rcfg.id        = ARM_M3_ID;
    rcfg.p_min     = -12.57f; rcfg.p_max = 12.57f;
    rcfg.v_min     = -50.0f;  rcfg.v_max = 50.0f;
    rcfg.t_min     = -5.5f;  rcfg.t_max = 5.5f;
    rcfg.kp_min    = 0.0f;   rcfg.kp_max = 500.0f;
    rcfg.kd_min    = 0.0f;   rcfg.kd_max = 5.0f;
    rcfg.sign      = -1;
    rcfg.master_id = 0xFFU;   /* host CAN id (reference project) */
    if (robstride_add(&rcfg) < 0) { return 1U; }

    return 0U;
}

/* -------- all feedback channels must be fresh and healthy -------- */
static uint8_t arm_feedback_ready(uint32_t now)
{
    mit_motor_state_t *st1 = mit_motor_get_state(ARM_M1_ID);
    mit_motor_state_t *st2 = mit_motor_get_state(ARM_M2_ID);
    robstride_state_t *st3 = robstride_get_state(ARM_M3_ID);

    if ((st1 == NULL) || (st2 == NULL) || (st3 == NULL)) { return 0U; }
    if ((st1->online == 0U) || (st2->online == 0U) || (st3->online == 0U)) { return 0U; }

    if ((uint32_t)(now - st1->last_rx_ms) > ARM_FEEDBACK_TIMEOUT_MS) { return 0U; }
    if ((uint32_t)(now - st2->last_rx_ms) > ARM_FEEDBACK_TIMEOUT_MS) { return 0U; }
    if ((uint32_t)(now - st3->last_rx_ms) > ARM_FEEDBACK_TIMEOUT_MS) { return 0U; }

    if ((st1->error != 0U) || (st2->error != 0U) || (st3->error != 0U)) { return 0U; }
    if ((float)st1->temp > ARM_TEMP_LIMIT_C) { return 0U; }
    if ((float)st2->temp > ARM_TEMP_LIMIT_C) { return 0U; }
    if (st3->temp > ARM_TEMP_LIMIT_C) { return 0U; }

    return 1U;
}

/* -------- stop all outputs and latch the fault -------- */
static void arm_fail_safe(void)
{
    arm_traj_stop();

    if (s_hfdcan != NULL)
    {
        mit_motor_disable(s_hfdcan, ARM_M1_ID);
        mit_motor_disable(s_hfdcan, ARM_M2_ID);
        robstride_disable(s_hfdcan, ARM_M3_ID, 0U);
    }

    s_fault  = 1U;
    s_inited = 0U;
    arm_dbg.last_err = -2;
}

/* -------- update Cartesian position error -------- */
static void arm_update_xy_error(void)
{
    arm_xy_err.x = arm_dbg.x - arm_dbg.x_actual;
    arm_xy_err.z = arm_dbg.z - arm_dbg.z_actual;
}

/* -------- read feedback and convert motor angle -> joint angle -------- */
static void arm_refresh_feedback(void)
{
    mit_motor_state_t *st1 = mit_motor_get_state(ARM_M1_ID);
    mit_motor_state_t *st2 = mit_motor_get_state(ARM_M2_ID);
    robstride_state_t *st3 = robstride_get_state(ARM_M3_ID);

    /* shoulder (motor1 -> joint1) */
    if (st1) {
        s_cur_joint[1] = arm_motor_to_joint_1(st1->pos);
        arm_dbg.joint[0].angle  = s_cur_joint[1];
        arm_dbg.joint[0].vel    = st1->vel;
        arm_dbg.joint[0].torque = st1->torque;
        arm_dbg.joint[0].online  = st1->online;
        arm_dbg.joint[0].error   = st1->error;
        arm_dbg.joint[0].pattern = 0U;
        arm_dbg.joint[0].sign    = +1;
    }
    /* elbow (motor2 -> joint2) */
    if (st2) {
        s_cur_joint[2] = arm_motor_to_joint_2(st2->pos);
        arm_dbg.joint[1].angle  = s_cur_joint[2];
        arm_dbg.joint[1].vel    = st2->vel;
        arm_dbg.joint[1].torque = st2->torque;
        arm_dbg.joint[1].online  = st2->online;
        arm_dbg.joint[1].error   = st2->error;
        arm_dbg.joint[1].pattern = 0U;
        arm_dbg.joint[1].sign    = +1;
    }
    /* wrist (robstride -> q0 directly) */
    if (st3) {
        s_cur_joint[0]       = st3->angle;
        arm_dbg.joint[2].angle  = st3->angle;
        arm_dbg.joint[2].vel    = st3->speed;
        arm_dbg.joint[2].torque = st3->torque;
        arm_dbg.joint[2].online  = st3->online;
        arm_dbg.joint[2].error   = st3->error;
        arm_dbg.joint[2].pattern = st3->pattern;
        arm_dbg.joint[2].sign    = -1;
    }

    /* Forward kinematics: current wrist centre, L1/L2 only. */
    arm_forward(s_cur_joint[1], s_cur_joint[2],
                &arm_dbg.x_actual, &arm_dbg.z_actual);
    arm_update_xy_error();
}

/* Keep the L3 axis level while q1/q2 move. */
static float arm_l3_level_target(float q1, float q2, float q0_ref)
{
    float q0 = ARM_L3_LEVEL_C - (q1 + q2);

    while ((q0 - q0_ref) > M_PI)  { q0 -= 2.0f * M_PI; }
    while ((q0 - q0_ref) < -M_PI) { q0 += 2.0f * M_PI; }
    arm_clamp_joints(&q0, NULL, NULL);
    return q0;
}

static float arm_limit_value(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static float arm_slew_limit(float target, float current, float max_step)
{
    float d = target - current;

    if (d >  max_step) { d =  max_step; }
    if (d < -max_step) { d = -max_step; }
    return current + d;
}

/* -------- send control to the 3 motors -------- */
static void arm_send_motors(float q0, float q1, float q2,
                            float v0, float v1, float v2)
{
    float m1 = arm_joint_to_motor_1(q1);
    float m2 = arm_joint_to_motor_2(q2);
    float tau_sh = 0.0f;
    float tau_el = 0.0f;

    arm_gravity_get(s_cur_joint[1], s_cur_joint[2], &tau_sh, &tau_el);

    /* Send EL05 first so the two AK frames cannot fill the Tx FIFO and starve it. */
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        robstride_set_control(s_hfdcan, ARM_M3_ID, 0.0f, q0, v0,
                              ARM_KP_WRIST, ARM_KD_WRIST);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M1_ID, m1, v1,
                              ARM_KP_SHOULDER, ARM_KD_SHOULDER, tau_sh);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M2_ID, m2, v2,
                              ARM_KP_ELBOW, ARM_KD_ELBOW, tau_el);
    }
}

/* Mode 3: gravity feed-forward + optional position hold.
 * arm_gravity_hold_enable = 1: latch current pose and hold with Kp/Kd.
 * arm_gravity_hold_enable = 0: pure feed-forward test (Kp=0, Kd=0).
 */
static void arm_send_gravity_test(void)
{
    float tau_sh = 0.0f;
    float tau_el = 0.0f;
    float q0_ref;
    float q1_ref;
    float q2_ref;
    float kp_sh;
    float kd_sh;
    float kp_el;
    float kd_el;

    arm_gravity_get(s_cur_joint[1], s_cur_joint[2], &tau_sh, &tau_el);

    if (arm_gravity_hold_enable != 0U)
    {
        if (s_gravity_hold_valid == 0U)
        {
            s_gravity_hold_q[0] = s_cur_joint[0];
            s_gravity_hold_q[1] = s_cur_joint[1];
            s_gravity_hold_q[2] = s_cur_joint[2];
            s_gravity_hold_valid = 1U;
        }

        q0_ref = s_gravity_hold_q[0];
        q1_ref = s_gravity_hold_q[1];
        q2_ref = s_gravity_hold_q[2];
        kp_sh = ARM_KP_SHOULDER;
        kd_sh = ARM_KD_SHOULDER;
        kp_el = ARM_KP_ELBOW;
        kd_el = ARM_KD_ELBOW;
    }
    else
    {
        s_gravity_hold_valid = 0U;
        q0_ref = s_cur_joint[0];
        q1_ref = s_cur_joint[1];
        q2_ref = s_cur_joint[2];
        kp_sh = 0.0f;
        kd_sh = 0.0f;
        kp_el = 0.0f;
        kd_el = 0.0f;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        robstride_set_control(s_hfdcan, ARM_M3_ID, 0.0f,
                              q0_ref, 0.0f,
                              ARM_KP_WRIST, ARM_KD_WRIST);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M1_ID,
                              arm_joint_to_motor_1(q1_ref), 0.0f,
                              kp_sh, kd_sh, tau_sh);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M2_ID,
                              arm_joint_to_motor_2(q2_ref), 0.0f,
                              kp_el, kd_el, tau_el);
    }

    arm_dbg.joint[0].target = q1_ref;
    arm_dbg.joint[1].target = q2_ref;
    arm_dbg.joint[2].target = q0_ref;
    arm_dbg.gravity_test = 3U;
    arm_dbg.reached = 0U;
}
/* -------- zero torque/gain frame: solicit feedback without motion -------- */
static void arm_send_safe_idle(void)
{
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        robstride_set_control(s_hfdcan, ARM_M3_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M1_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M2_ID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}

/* -------- public: init (wait for valid feedback, then HOLD) -------- */
void arm_init(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t deadline;

    s_hfdcan  = hfdcan;
    s_inited  = 0U;
    s_fault   = 0U;
    s_target_set = 0U;
    arm_cmd_new = 0U;
    s_gravity_hold_valid = 0U;
    s_l3_level_active = 0U;
    s_last_tick = HAL_GetTick();
    arm_traj_stop();

    if (arm_setup_motors(hfdcan) != 0U)
    {
        arm_fail_safe();
        return;
    }

    /* Enable, but keep zero gain/torque until all three positions are known. */
    HAL_Delay(800U);
    mit_motor_enable(hfdcan, ARM_M1_ID);
    mit_motor_enable(hfdcan, ARM_M2_ID);
    robstride_enable(hfdcan, ARM_M3_ID);
    s_next_el05_enable_ms = HAL_GetTick() + ARM_EL05_ENABLE_RETRY_MS;

    deadline = HAL_GetTick() + ARM_STARTUP_TIMEOUT_MS;
    for (;;)
    {
        uint32_t now;

        if (hfdcan != NULL) { fdcan_drv_service(hfdcan); }
        arm_send_safe_idle();
        if ((int32_t)(HAL_GetTick() - s_next_el05_enable_ms) >= 0)
        {
            if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) != 0U)
            {
                robstride_enable(hfdcan, ARM_M3_ID);
            }
            s_next_el05_enable_ms = HAL_GetTick() + ARM_EL05_ENABLE_RETRY_MS;
        }
        arm_refresh_feedback();
        now = HAL_GetTick();

        if (arm_feedback_ready(now) != 0U) { break; }
        if ((int32_t)(now - deadline) >= 0)
        {
            arm_fail_safe();
            return;
        }
        HAL_Delay(1U);
    }

    /* First valid positions become the no-motion hold targets. */
    arm_refresh_feedback();
    s_tgt_joint[0] = s_cur_joint[0];
    s_tgt_joint[1] = s_cur_joint[1];
    s_tgt_joint[2] = s_cur_joint[2];
    s_q0_cmd = s_cur_joint[0];
    arm_dbg.x = arm_dbg.x_actual;
    arm_dbg.z = arm_dbg.z_actual;
    arm_update_xy_error();
    s_target_set   = 0U;

    arm_dbg.mode         = 1U;
    arm_dbg.gravity_test = 0U;
    arm_dbg.reached      = 0U;
    arm_dbg.last_err = 0;
    s_last_run_ms = HAL_GetTick();
    s_fault  = 0U;
    s_inited = 1U;
}

/* -------- public: goto cartesian target -------- */
int arm_goto(float x, float z, float yaw)
{
    float q0, q1, q2;

    if ((s_inited == 0U) || (s_fault != 0U))
    {
        arm_dbg.last_err = -2;
        return -1;
    }

    if (arm_inverse_nearest(x, z, yaw, s_cur_joint[1], s_cur_joint[2], &q0, &q1, &q2) != 0)
    {
        arm_dbg.last_err = -1;   /* unreachable */
        return -1;
    }
    q0 = arm_l3_level_target(q1, q2, s_cur_joint[0]);
    arm_clamp_joints(&q0, &q1, &q2);

    arm_traj_start(s_cur_joint[0], s_cur_joint[1], s_cur_joint[2],
                   q0, q1, q2, ARM_TRAJ_TIME);
    s_l3_level_active = 1U;
    s_q0_cmd = s_cur_joint[0];
    s_tgt_joint[0] = q0;
    s_tgt_joint[1] = q1;
    s_tgt_joint[2] = q2;
    s_target_set   = 1;
    arm_dbg.last_err = 0;
    arm_dbg.reached  = 0;
    arm_dbg.x = x; arm_dbg.z = z; arm_dbg.yaw = yaw;
    arm_dbg.mode = 1;
    return 0;
}

/* -------- public: periodic run (~10 ms) -------- */
void arm_run(void)
{
    uint32_t now;
	
//本次轨迹计算出来的三个关节角，三个关节速度
    float q0, q1, q2, v0, v1, v2;
//done=0还在运行，=1结束
    int done = 0;
//三个关节之间的误差
    float err0, err1, err2;
//如果初始化失败就返回
    if (s_hfdcan != NULL) { fdcan_drv_service(s_hfdcan); }
    if (!s_inited) { return; }

    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_run_ms) < 10U) { return; }
    s_last_run_ms = now;

    if ((int32_t)(now - s_next_el05_enable_ms) >= 0)
    {
        if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
        {
            robstride_enable(s_hfdcan, ARM_M3_ID);
        }
        s_next_el05_enable_ms = now + ARM_EL05_ENABLE_RETRY_MS;
    }
//上一次run执行的时间
    if (s_last_tick) { s_dt = (float)(now - s_last_tick) * 0.001f; }
    s_last_tick = now;
    if (s_dt <= 0.0f)  { s_dt = 0.01f; }
    if (s_dt >  0.05f) { s_dt = 0.01f; }

    /* update feedback and stop immediately if a channel is unhealthy */
    arm_refresh_feedback();
    if (arm_feedback_ready(now) == 0U)
    {
        arm_fail_safe();
        return;
    }

    /* Gravity test mode 3: bypass trajectory control. */
    if (arm_gravity_test != 3U)
    {
        arm_gravity_test = 0U;
        s_gravity_hold_valid = 0U;
    }
    if (arm_gravity_test == 3U)
    {
        arm_traj_stop();
        s_target_set = 0U;
        arm_cmd_new = 0U;
        arm_send_gravity_test();
        return;
    }
    arm_dbg.gravity_test = 0U;
    /* handle new command from debug */
    if (arm_cmd_new)
    {
        arm_cmd_new = 0;
        (void)arm_goto(arm_target[0], arm_target[1], arm_target[2]);
    }

    /* run trajectory / hold */
//五次多项式规划成功
    if (arm_traj_is_active())
    {
        done = arm_traj_update(s_dt, &q0, &q1, &q2, &v0, &v1, &v2);

        if (s_l3_level_active != 0U)
        {
            q0 = arm_l3_level_target(q1, q2, s_q0_cmd);
            q0 = arm_slew_limit(q0, s_q0_cmd, ARM_L3_MAX_STEP_RAD);
            s_q0_cmd = q0;
            v0 = arm_limit_value(-(v1 + v2),
                                 -ARM_L3_MAX_SPEED, ARM_L3_MAX_SPEED);
        }

        arm_clamp_joints(&q0, &q1, &q2);
        if (s_l3_level_active != 0U) { s_q0_cmd = q0; }
        arm_send_motors(q0, q1, q2, v0, v1, v2);

        arm_dbg.joint[0].target = q1;
        arm_dbg.joint[1].target = q2;
        arm_dbg.joint[2].target = q0;

        if (done)
        {
            arm_dbg.reached = 1;   /* trajectory finished */
        }
    }
    else
    {
        /* hold:
         *   如果没有五次多项式规划时，直接保持在原地，不做运动 */
        if (!s_target_set)
        {
            s_tgt_joint[0] = s_cur_joint[0];
            s_tgt_joint[1] = s_cur_joint[1];
            s_tgt_joint[2] = s_cur_joint[2];
            s_q0_cmd = s_cur_joint[0];
        }
        else if (s_l3_level_active != 0U)
        {
            float q0_hold = arm_l3_level_target(s_tgt_joint[1],
                                                s_tgt_joint[2], s_q0_cmd);
            q0_hold = arm_slew_limit(q0_hold, s_q0_cmd, ARM_L3_MAX_STEP_RAD);
            arm_clamp_joints(&q0_hold, NULL, NULL);
            s_q0_cmd = q0_hold;
            s_tgt_joint[0] = q0_hold;
        }
        arm_send_motors(s_tgt_joint[0], s_tgt_joint[1], s_tgt_joint[2],
                        0.0f, 0.0f, 0.0f);
        arm_dbg.joint[0].target = s_tgt_joint[1];
        arm_dbg.joint[1].target = s_tgt_joint[2];
        arm_dbg.joint[2].target = s_tgt_joint[0];

        /* arrival check against the true current joints */
        err0 = fabsf(s_cur_joint[0] - s_tgt_joint[0]);
        err1 = fabsf(s_cur_joint[1] - s_tgt_joint[1]);
        err2 = fabsf(s_cur_joint[2] - s_tgt_joint[2]);
        if (s_target_set &&
            err0 < ARM_POS_TOL && err1 < ARM_POS_TOL && err2 < ARM_POS_TOL)
        {
            arm_dbg.reached = 1;
        }
    }
}
