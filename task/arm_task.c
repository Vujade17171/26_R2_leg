/**
  ******************************************************************************
  * @file    arm_task.c
  * @brief   机械臂 FreeRTOS 控制任务（3 个电机，共用 FDCAN 总线）
  ******************************************************************************
  * 控制流程：
  *   arm_target -> arm_inverse_nearest -> arm_traj -> 3 路电机控制。
  *
  * 电机分配：
  *   - 肩关节：AK80-9（ID=1，MIT）
  *   - 肘关节：AK45-10（ID=2，MIT）
  *   - 腕关节：灵足-05（ID=3，RobStride）
  *
  * 安全策略：
  *   - 上电后机械臂保持在实际当前位置；
  *   - 只有收到第一次 arm_cmd_new 命令后才开始运动；
  *   - 任一反馈通道超时或故障时立即停机并锁存故障。
  ******************************************************************************
  */
#include "arm_task.h"
#include "fdcan_drv.h"
#include "mit_motor.h"
#include "robstride.h"
#include "arm_kinematics.h"
#include "arm_traj.h"
#include "arm_gravity.h"
#include "tim.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include <math.h>

/* ---- 电机 ID ---- */
#define ARM_M1_ID   1   /* AK80-9 肩关节 */
#define ARM_M2_ID   2   /* AK45-10 肘关节 */
#define ARM_M3_ID   3   /* 灵足-05 腕关节 */

/* ---- 控制器增益 ---- */
#define ARM_KP_SHOULDER  130.0f
#define ARM_KD_SHOULDER  1.40f
#define ARM_KP_ELBOW     150.0f
#define ARM_KD_ELBOW     1.5f
#define ARM_KP_WRIST     100.0f
#define ARM_KD_WRIST     1.0f

/* ---- 轨迹与保持 ---- */
#define ARM_TRAJ_TIME    1.0f    /* s，默认点到点运动时长 */
#define ARM_POS_TOL      0.02f   /* rad，到位容差 */

/* ---- 控制周期 ---- */
#define ARM_CONTROL_PERIOD_MS   2U
#define ARM_CONTROL_DT          ((float)ARM_CONTROL_PERIOD_MS * 0.001f)

/* ---- 启动与运行安全 ---- */
#define ARM_STARTUP_TIMEOUT_MS   3000U
#define ARM_EL05_ENABLE_RETRY_MS 200U
#define ARM_FEEDBACK_TIMEOUT_MS  200U
#define ARM_TEMP_LIMIT_C         80.0f

/* arm_dbg.joint[] 的显示顺序与内部关节编号不同：肩、肘、腕 */
enum
{
    ARM_DBG_SHOULDER = 0,
    ARM_DBG_ELBOW = 1,
    ARM_DBG_WRIST = 2
};

/* ---- 调试全局变量（在 Keil Watch 中查看） ---- */
arm_dbg_t arm_dbg = {0};

volatile float   arm_target[3] = {0.35f, 0.25f, 0.0f};  /* x、z、偏航角 */
volatile uint8_t arm_cmd_new = 0U;

/* ---- 控制器内部状态 ---- */
static FDCAN_HandleTypeDef *s_hfdcan = NULL;
static uint32_t s_last_tick = 0U;
static float    s_dt = ARM_CONTROL_DT;
static uint8_t  s_inited = 0U;
static uint8_t  s_target_set = 0U;  /* 收到第一次目标命令前为 0 */
static uint8_t  s_fault = 0U;       /* 1 表示安全停机已锁存 */
static uint32_t s_next_el05_enable_ms = 0U;

/* 实时关节状态：q0=腕，q1=肩，q2=肘，单位均为 rad */
static float s_cur_joint[3] = {0.0f, 0.0f, 0.0f};
static float s_tgt_joint[3] = {0.0f, 0.0f, 0.0f};

/* ============================== 电机配置 =============================== */

static uint8_t arm_setup_motors(FDCAN_HandleTypeDef *hfdcan)
{
    mit_motor_cfg_t mcfg;
    robstride_cfg_t rcfg;

    /* 1) FDCAN 驱动初始化（滤波器 + 启动） */
    if (fdcan_drv_init(hfdcan) != 0U)
    {
        return 1U;
    }

    /* 2) 初始化电机驱动并注册接收回调 */
    if (mit_motor_init(hfdcan) != 0U)
    {
        return 1U;
    }
    if (robstride_init(hfdcan) != 0U)
    {
        return 1U;
    }

    /* 3) AK80-9 肩关节 */
    mcfg.id = ARM_M1_ID;
    mcfg.p_min = -12.5f;
    mcfg.p_max = 12.5f;
    mcfg.v_min = -50.0f;
    mcfg.v_max = 50.0f;
    mcfg.t_min = -18.0f;
    mcfg.t_max = 18.0f;
    mcfg.kp_min = 0.0f;
    mcfg.kp_max = 500.0f;
    mcfg.kd_min = 0.0f;
    mcfg.kd_max = 5.0f;
    mcfg.sign = +1;    /* 方向由 arm_joint_to_motor_1 处理 */
    if (mit_motor_add(&mcfg) < 0)
    {
        return 1U;
    }

    /* 4) AK45-10 肘关节 */
    mcfg.id = ARM_M2_ID;
    mcfg.p_min = -12.6f;
    mcfg.p_max = 12.6f;
    mcfg.v_min = -8.0f;
    mcfg.v_max = 8.0f;
    mcfg.t_min = -7.0f;
    mcfg.t_max = 7.0f;
    mcfg.kp_min = 0.0f;
    mcfg.kp_max = 500.0f;
    mcfg.kd_min = 0.0f;
    mcfg.kd_max = 5.0f;
    mcfg.sign = +1;    /* 方向由 arm_joint_to_motor_2 处理 */
    if (mit_motor_add(&mcfg) < 0)
    {
        return 1U;
    }

    /* 5) 灵足-05 腕关节（RobStride 扩展帧） */
    rcfg.id = ARM_M3_ID;
    rcfg.p_min = -12.57f;
    rcfg.p_max = 12.57f;
    rcfg.v_min = -50.0f;
    rcfg.v_max = 50.0f;
    rcfg.t_min = -5.5f;
    rcfg.t_max = 5.5f;
    rcfg.kp_min = 0.0f;
    rcfg.kp_max = 500.0f;
    rcfg.kd_min = 0.0f;
    rcfg.kd_max = 5.0f;
    rcfg.sign = -1;
    rcfg.master_id = 0xFFU;
    if (robstride_add(&rcfg) < 0)
    {
        return 1U;
    }

    return 0U;
}

/* ============================== 反馈处理 =============================== */

static void arm_refresh_feedback(void)
{
    mit_motor_state_t *st1 = mit_motor_get_state(ARM_M1_ID);
    mit_motor_state_t *st2 = mit_motor_get_state(ARM_M2_ID);
    robstride_state_t *st3 = robstride_get_state(ARM_M3_ID);

    if (st1 != NULL)
    {
        s_cur_joint[1] = arm_motor_to_joint_1(st1->pos);
        arm_dbg.joint[ARM_DBG_SHOULDER].angle = s_cur_joint[1];
    }

    if (st2 != NULL)
    {
        s_cur_joint[2] = arm_motor_to_joint_2(st2->pos);
        arm_dbg.joint[ARM_DBG_ELBOW].angle = s_cur_joint[2];
    }

    if (st3 != NULL)
    {
        s_cur_joint[0] = st3->angle;
        arm_dbg.joint[ARM_DBG_WRIST].angle = s_cur_joint[0];
    }

    /* 当前腕关节中心位置，仅用于调试显示。 */
    arm_forward(s_cur_joint[1], s_cur_joint[2],
                &arm_dbg.x_actual, &arm_dbg.z_actual);
}

static uint8_t arm_feedback_ready(uint32_t now)
{
    mit_motor_state_t *st1 = mit_motor_get_state(ARM_M1_ID);
    mit_motor_state_t *st2 = mit_motor_get_state(ARM_M2_ID);
    robstride_state_t *st3 = robstride_get_state(ARM_M3_ID);

    if ((st1 == NULL) || (st2 == NULL) || (st3 == NULL))
    {
        return 0U;
    }
    if ((st1->online == 0U) || (st2->online == 0U) || (st3->online == 0U))
    {
        return 0U;
    }
    if ((uint32_t)(now - st1->last_rx_ms) > ARM_FEEDBACK_TIMEOUT_MS)
    {
        return 0U;
    }
    if ((uint32_t)(now - st2->last_rx_ms) > ARM_FEEDBACK_TIMEOUT_MS)
    {
        return 0U;
    }
    if ((uint32_t)(now - st3->last_rx_ms) > ARM_FEEDBACK_TIMEOUT_MS)
    {
        return 0U;
    }
    if ((st1->error != 0U) || (st2->error != 0U) || (st3->error != 0U))
    {
        return 0U;
    }
    if ((float)st1->temp > ARM_TEMP_LIMIT_C)
    {
        return 0U;
    }
    if ((float)st2->temp > ARM_TEMP_LIMIT_C)
    {
        return 0U;
    }
    if (st3->temp > ARM_TEMP_LIMIT_C)
    {
        return 0U;
    }

    return 1U;
}

/* ============================== 电机发送 =============================== */

static void arm_send_motors(float q0, float q1, float q2,
                            float v0, float v1, float v2)
{
    float motor_shoulder = arm_joint_to_motor_1(q1);
    float motor_elbow = arm_joint_to_motor_2(q2);
    float torque_shoulder = 0.0f;
    float torque_elbow = 0.0f;
    float torque_wrist_ff = 0.0f;

    arm_gravity_get(s_cur_joint[1], s_cur_joint[2], s_dt,
                    &torque_shoulder, &torque_elbow);
    torque_wrist_ff = arm_wrist_gravity_get(s_cur_joint[0],
                                            s_cur_joint[1],
                                            s_cur_joint[2],
                                            ARM_L3_LEVEL_C,
                                            s_dt);

    /* 先发送 EL05，避免两个 AK 帧占满 Tx FIFO。 */
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        robstride_set_control(s_hfdcan, ARM_M3_ID, torque_wrist_ff, q0, v0,
                              ARM_KP_WRIST, ARM_KD_WRIST);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M1_ID, motor_shoulder, v1,
                              ARM_KP_SHOULDER, ARM_KD_SHOULDER,
                              torque_shoulder);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M2_ID, motor_elbow, v2,
                              ARM_KP_ELBOW, ARM_KD_ELBOW,
                              torque_elbow);
    }
}

static void arm_send_safe_idle(void)
{
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        robstride_set_control(s_hfdcan, ARM_M3_ID,
                              0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M1_ID,
                              0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
    {
        mit_motor_set_control(s_hfdcan, ARM_M2_ID,
                              0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
}

static void arm_update_debug_targets(float q0, float q1, float q2)
{
    arm_dbg.joint[ARM_DBG_SHOULDER].target = q1;
    arm_dbg.joint[ARM_DBG_ELBOW].target = q2;
    arm_dbg.joint[ARM_DBG_WRIST].target = q0;
}

/* ============================== 安全停机 =============================== */

static void arm_fail_safe(void)
{
    arm_traj_stop();

    if (s_hfdcan != NULL)
    {
        mit_motor_disable(s_hfdcan, ARM_M1_ID);
        mit_motor_disable(s_hfdcan, ARM_M2_ID);
        robstride_disable(s_hfdcan, ARM_M3_ID, 0U);
    }

    s_fault = 1U;
    s_inited = 0U;
    arm_dbg.last_err = -2;
}

/* ============================== 公开接口 =============================== */

void arm_task_hardware_init(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t deadline;

    s_hfdcan = hfdcan;
    s_inited = 0U;
    s_fault = 0U;
    s_target_set = 0U;
    arm_cmd_new = 0U;
    arm_l3_level_stop();
    arm_traj_stop();
    s_last_tick = HAL_GetTick();

    if (arm_setup_motors(hfdcan) != 0U)
    {
        arm_fail_safe();
        return;
    }

    /* 先使能，但在三个位置反馈都有效前保持零增益、零力矩。 */
    HAL_Delay(800U);
    mit_motor_enable(hfdcan, ARM_M1_ID);
    mit_motor_enable(hfdcan, ARM_M2_ID);
    robstride_enable(hfdcan, ARM_M3_ID);
    s_next_el05_enable_ms = HAL_GetTick() + ARM_EL05_ENABLE_RETRY_MS;

    deadline = HAL_GetTick() + ARM_STARTUP_TIMEOUT_MS;
    for (;;)
    {
        uint32_t now;

        if (hfdcan != NULL)
        {
            fdcan_drv_service(hfdcan);
        }
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

        if (arm_feedback_ready(now) != 0U)
        {
            break;
        }
        if ((int32_t)(now - deadline) >= 0)
        {
            arm_fail_safe();
            return;
        }
        HAL_Delay(1U);
    }

    /* 首次有效反馈作为无运动保持目标。 */
    arm_refresh_feedback();
    s_tgt_joint[0] = s_cur_joint[0];
    s_tgt_joint[1] = s_cur_joint[1];
    s_tgt_joint[2] = s_cur_joint[2];
    arm_l3_level_init(s_cur_joint[0]);

    arm_dbg.x = arm_dbg.x_actual;
    arm_dbg.z = arm_dbg.z_actual;
    arm_dbg.reached = 0U;
    arm_dbg.last_err = 0;
    s_target_set = 0U;
    s_last_tick = HAL_GetTick();
    s_fault = 0U;
    s_inited = 1U;
}

int arm_goto(float x, float z, float yaw)
{
    float q0;
    float q1;
    float q2;

    if ((s_inited == 0U) || (s_fault != 0U))
    {
        arm_dbg.last_err = -2;
        return -1;
    }

    if (arm_inverse_nearest(x, z, yaw,
                            s_cur_joint[1], s_cur_joint[2],
                            &q0, &q1, &q2) != 0)
    {
        arm_dbg.last_err = -1;  /* 目标不可达或超出关节限位 */
        return -1;
    }

    /* 逆解先返回传入 yaw；随后由 L3 水平约束重新计算腕关节目标。 */
    q0 = arm_l3_level_target(q1, q2, s_cur_joint[0]);
    arm_clamp_joints(&q0, &q1, &q2);

    arm_traj_start(s_cur_joint[0], s_cur_joint[1], s_cur_joint[2],
                   q0, q1, q2, ARM_TRAJ_TIME);
    arm_l3_level_start(s_cur_joint[0]);

    s_tgt_joint[0] = q0;
    s_tgt_joint[1] = q1;
    s_tgt_joint[2] = q2;
    s_target_set = 1U;
    arm_dbg.last_err = 0;
    arm_dbg.reached = 0U;
    arm_dbg.x = x;
    arm_dbg.z = z;
    return 0;
}

static void arm_control_step(void)
{
    uint32_t now;
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float v0 = 0.0f;
    float v1 = 0.0f;
    float v2 = 0.0f;
    float l3_max_step;
    int trajectory_done = 0;

    if (s_hfdcan != NULL)
    {
        fdcan_drv_service(s_hfdcan);
    }
    if (s_inited == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    /* 周期性重发腕关节使能帧，避免 EL05 掉线后无法恢复。 */
    if ((int32_t)(now - s_next_el05_enable_ms) >= 0)
    {
        if (HAL_FDCAN_GetTxFifoFreeLevel(s_hfdcan) != 0U)
        {
            robstride_enable(s_hfdcan, ARM_M3_ID);
        }
        s_next_el05_enable_ms = now + ARM_EL05_ENABLE_RETRY_MS;
    }

    /* 控制算法使用固定 2 ms 周期，避免 HAL_GetTick 的 1 ms 量化造成 dt 抖动。 */
    s_dt = ARM_CONTROL_DT;
    s_last_tick = now;

    l3_max_step = ARM_L3_MAX_SPEED * s_dt;

    arm_refresh_feedback();
    if (arm_feedback_ready(now) == 0U)
    {
        arm_fail_safe();
        return;
    }

    /* 处理来自 Keil Watch 的笛卡尔目标命令。 */
    if (arm_cmd_new != 0U)
    {
        arm_cmd_new = 0U;
        (void)arm_goto(arm_target[0], arm_target[1], arm_target[2]);
    }

    if (arm_traj_is_active() != 0)
    {
        trajectory_done = arm_traj_update(s_dt,
                                          &q0, &q1, &q2,
                                          &v0, &v1, &v2);

        if (arm_l3_level_is_active() != 0U)
        {
            q0 = arm_l3_level_update(q1, q2, v1, v2,
                                     l3_max_step, &v0);
        }

        arm_clamp_joints(&q0, &q1, &q2);
        arm_send_motors(q0, q1, q2, v0, v1, v2);
        arm_update_debug_targets(q0, q1, q2);

        if (trajectory_done != 0)
        {
            arm_dbg.reached = 1U;
        }
        return;
    }

    /*
     * 没有五次多项式轨迹时保持在目标位置：
     *   - 尚未收到目标命令时，以当前反馈位置为保持目标；
     *   - 收到目标后保持 arm_goto() 锁存的目标位置；
     *   - 保持时速度前馈固定为零。
     */
    if (s_target_set == 0U)
    {
        s_tgt_joint[0] = s_cur_joint[0];
        s_tgt_joint[1] = s_cur_joint[1];
        s_tgt_joint[2] = s_cur_joint[2];
        arm_l3_level_set_q0_cmd(s_cur_joint[0]);
    }
    else if (arm_l3_level_is_active() != 0U)
    {
        float q0_hold = arm_l3_level_update(s_tgt_joint[1],
                                            s_tgt_joint[2],
                                            0.0f, 0.0f,
                                            l3_max_step,
                                            NULL);
        arm_clamp_joints(&q0_hold, NULL, NULL);
        s_tgt_joint[0] = q0_hold;
    }

    arm_send_motors(s_tgt_joint[0], s_tgt_joint[1], s_tgt_joint[2],
                    0.0f, 0.0f, 0.0f);
    arm_update_debug_targets(s_tgt_joint[0], s_tgt_joint[1], s_tgt_joint[2]);

    if (s_target_set != 0U)
    {
        float err0 = fabsf(s_cur_joint[0] - s_tgt_joint[0]);
        float err1 = fabsf(s_cur_joint[1] - s_tgt_joint[1]);
        float err2 = fabsf(s_cur_joint[2] - s_tgt_joint[2]);

        if ((err0 < ARM_POS_TOL) && (err1 < ARM_POS_TOL) && (err2 < ARM_POS_TOL))
        {
            arm_dbg.reached = 1U;
        }
    }
}


/* ============================== RTOS 周期任务 ============================== */

#define ARM_TASK_STACK_SIZE  (1024U * 4U)

static osThreadId_t s_arm_task_handle = NULL;
static volatile uint32_t s_control_tick = 0U;
static uint32_t s_control_tick_seen = 0U;

static const osThreadAttr_t s_arm_task_attributes =
{
    .name = "ArmControlTask",
    .stack_size = ARM_TASK_STACK_SIZE,
    .priority = (osPriority_t)osPriorityHigh,
};

static void arm_control_task(void *argument);

void arm_task_start(void)
{
    s_arm_task_handle = osThreadNew(arm_control_task, NULL, &s_arm_task_attributes);

    if (s_arm_task_handle == NULL)
    {
        Error_Handler();
    }
}

void arm_task_tick(void)
{
    /* TIM6 按固定控制周期调用，只记录出现了一个新的控制节拍。 */
    s_control_tick++;
}

static void arm_control_task(void *argument)
{
    (void)argument;

    /* 进入任务后再启动 TIM6，避免调度器启动前产生控制节拍。 */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    for (;;)
    {
        /* 每 1 ms 检查一次，有新节拍时执行一次控制更新。 */
        (void)osDelay(1);

        if (s_control_tick != s_control_tick_seen)
        {
            s_control_tick_seen = s_control_tick;
            arm_control_step();
        }
    }
}
