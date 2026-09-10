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

/* ---- debug globals (watch in Keil) ---- */
arm_dbg_t arm_dbg = {0};
volatile uint8_t  arm_cmd_mode = 1;                       /* 控制模式 */
volatile float    arm_target[3] = {0.20f, 0.30f, 0.0f};  /* x,z,yaw */
volatile uint8_t  arm_cmd_new  = 0;

static FDCAN_HandleTypeDef *s_hfdcan = NULL;
static uint32_t s_last_tick = 0;
static float    s_dt        = 0.01f;
static uint8_t  s_inited    = 0;
static uint8_t  s_target_set = 0;   /* 0 until first target command */

/* live joint state (feedback converted to joint angles) */
static float s_cur_joint[3] = {0, 0, 0};   /* q0,wrist q1,shoulder q2,elbow */
static float s_tgt_joint[3] = {0, 0, 0};   /* current commanded target       */

/* -------- register the 3 motor configs and enable them -------- */
static void arm_setup_motors(FDCAN_HandleTypeDef *hfdcan)
{
    mit_motor_cfg_t mcfg;
    robstride_cfg_t rcfg;

    /* 1) bus init (filters + start) */
    fdcan_drv_init(hfdcan);

    /* 2) init drivers (register rx callback) */
    mit_motor_init(hfdcan);
    robstride_init(hfdcan);

    /* 3) AK80-9  shoulder */
    mcfg.id = ARM_M1_ID;
    mcfg.p_min = -12.5f; mcfg.p_max = 12.5f;
    mcfg.v_min = -50.0f; mcfg.v_max = 50.0f;
    mcfg.t_min = -18.0f; mcfg.t_max = 18.0f;
    mcfg.kp_min = 0.0f;  mcfg.kp_max = 500.0f;
    mcfg.kd_min = 0.0f;  mcfg.kd_max = 5.0f;
    mcfg.sign   = +1;    /* direction handled by arm_joint_to_motor_1 */
    mit_motor_add(&mcfg);

    /* 4) AK45-10 elbow (direction handled by arm offset) */
    mcfg.id = ARM_M2_ID;
    mcfg.p_min = -12.6f; mcfg.p_max = 12.6f;
    mcfg.v_min = -8.0f;  mcfg.v_max = 8.0f;
    mcfg.t_min = -7.0f;  mcfg.t_max = 7.0f;
    mcfg.kp_min = 0.0f;  mcfg.kp_max = 500.0f;
    mcfg.kd_min = 0.0f;  mcfg.kd_max = 5.0f;
    mcfg.sign   = +1;
    mit_motor_add(&mcfg);

    /* 5) Lingzu-05 wrist (RobStride extended frame) */
    rcfg.id        = ARM_M3_ID;
    rcfg.p_min     = -3.14f; rcfg.p_max = 3.14f;
    rcfg.v_min     = -10.0f; rcfg.v_max = 10.0f;
    rcfg.t_min     = -5.5f;  rcfg.t_max = 5.5f;
    rcfg.kp_min    = 0.0f;   rcfg.kp_max = 500.0f;
    rcfg.kd_min    = 0.0f;   rcfg.kd_max = 5.0f;
    rcfg.sign      = +1;
    rcfg.master_id = 0x11;   /* host CAN id (reference project) */
    robstride_add(&rcfg);

    /* 6) enable all three */
    mit_motor_enable(hfdcan, ARM_M1_ID);
    mit_motor_enable(hfdcan, ARM_M2_ID);
    robstride_enable(hfdcan, ARM_M3_ID);
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
        arm_dbg.joint[0].online = st1->online;
        arm_dbg.joint[0].sign   = +1;
    }
    /* elbow (motor2 -> joint2) */
    if (st2) {
        s_cur_joint[2] = arm_motor_to_joint_2(st2->pos);
        arm_dbg.joint[1].angle  = s_cur_joint[2];
        arm_dbg.joint[1].vel    = st2->vel;
        arm_dbg.joint[1].torque = st2->torque;
        arm_dbg.joint[1].online = st2->online;
        arm_dbg.joint[1].sign   = +1;
    }
    /* wrist (robstride -> q0 directly) */
    if (st3) {
        s_cur_joint[0]       = st3->angle;
        arm_dbg.joint[2].angle  = st3->angle;
        arm_dbg.joint[2].vel    = st3->speed;
        arm_dbg.joint[2].torque = st3->torque;
        arm_dbg.joint[2].online = st3->online;
        arm_dbg.joint[2].sign   = +1;
    }
}

/* -------- send control to the 3 motors -------- */
static void arm_send_motors(float q0, float q1, float q2,
                            float v0, float v1, float v2)
{
    float m1 = arm_joint_to_motor_1(q1);
    float m2 = arm_joint_to_motor_2(q2);

    /* motor velocity: dm1/dt = -dq1/dt (shoulder), dm2/dt = +dq2/dt (elbow) */
    mit_motor_set_control(s_hfdcan, ARM_M1_ID, m1, -v1,
                          ARM_KP_SHOULDER, ARM_KD_SHOULDER, 0.0f);
    mit_motor_set_control(s_hfdcan, ARM_M2_ID, m2,  v2,
                          ARM_KP_ELBOW,    ARM_KD_ELBOW,    0.0f);
    robstride_set_control(s_hfdcan, ARM_M3_ID, 0.0f, q0, v0,
                          ARM_KP_WRIST, ARM_KD_WRIST);
}

/* -------- public: init (HOLD at current position, no command yet) -------- */
void arm_init(FDCAN_HandleTypeDef *hfdcan)
{
    s_hfdcan = hfdcan;
    arm_setup_motors(hfdcan);

    /* do NOT force a command on power-up; instead lock at actual position */
    arm_refresh_feedback();
    s_tgt_joint[0] = s_cur_joint[0];
    s_tgt_joint[1] = s_cur_joint[1];
    s_tgt_joint[2] = s_cur_joint[2];
    s_target_set   = 0;

    arm_dbg.mode   = arm_cmd_mode;
    arm_dbg.reached = 0;
    arm_dbg.last_err = 0;
    s_inited = 1;
}

/* -------- public: goto cartesian target -------- */
int arm_goto(float x, float z, float yaw)
{
    float q0, q1, q2;
    if (arm_inverse(x, z, yaw, &q0, &q1, &q2) != 0)
    {
        arm_dbg.last_err = -1;   /* unreachable */
        return -1;
    }
    arm_traj_start(s_cur_joint[0], s_cur_joint[1], s_cur_joint[2],
                   q0, q1, q2, ARM_TRAJ_TIME);
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

/* -------- public: goto joint target -------- */
void arm_set_joint(float q0, float q1, float q2)
{
    arm_clamp_joints(&q0, &q1, &q2);
    arm_traj_start(s_cur_joint[0], s_cur_joint[1], s_cur_joint[2],
                   q0, q1, q2, ARM_TRAJ_TIME);
    s_tgt_joint[0] = q0;
    s_tgt_joint[1] = q1;
    s_tgt_joint[2] = q2;
    s_target_set   = 1;
    arm_dbg.mode   = 0;
    arm_dbg.reached = 0;
    arm_dbg.last_err = 0;
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

    if (!s_inited) { return; }

    now = HAL_GetTick();
		
    if (s_last_tick) { s_dt = (float)(now - s_last_tick) * 0.001f; }
    s_last_tick = now;
    if (s_dt <= 0.0f)  { s_dt = 0.01f; }
    if (s_dt >  0.05f) { s_dt = 0.01f; }

    /* update feedback */
    arm_refresh_feedback();

    /* handle new command from debug */
    if (arm_cmd_new)
    {
        arm_cmd_new = 0;
        if (arm_cmd_mode == 1)
        {
            arm_goto(arm_target[0], arm_target[1], arm_target[2]);
        }
        else
        {
            arm_set_joint(arm_target[0], arm_target[1], arm_target[2]);
        }
    }

    /* run trajectory / hold */
    if (arm_traj_is_active())
    {
        done = arm_traj_update(s_dt, &q0, &q1, &q2, &v0, &v1, &v2);
        arm_clamp_joints(&q0, &q1, &q2);
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
         *   before first target -> lock at actual position (no jump).
         *   after a target       -> keep that target with zero velocity. */
        if (!s_target_set)
        {
            s_tgt_joint[0] = s_cur_joint[0];
            s_tgt_joint[1] = s_cur_joint[1];
            s_tgt_joint[2] = s_cur_joint[2];
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
