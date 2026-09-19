/**
  ******************************************************************************
  * @file    arm_control.c
  * @brief   机械臂控制器（3 个电机，共用 FDCAN 总线）
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
#include "arm_control.h"
#include "fdcan_drv.h"
#include "mit_motor.h"
#include "robstride.h"
#include "arm_kinematics.h"
#include "arm_traj.h"
#include "arm_gravity.h"
#include <math.h>

/* ---- 电机 ID ---- */
#define ARM_M1_ID   1   /* AK80-9 肩关节 */
#define ARM_M2_ID   2   /* AK45-10 肘关节 */
#define ARM_M3_ID   3   /* 灵足-05 腕关节 */

/* ---- 轨迹与保持 ---- */
#define ARM_TRAJ_TIME    1.0f    /* s，默认点到点运动时长 */

/* ---- 启动与运行安全 ---- */
#define ARM_STARTUP_TIMEOUT_MS       3000U
#define ARM_EL05_ENABLE_ACK_MS        100U
#define ARM_EL05_ENABLE_MAX_TRIES       3U
#define ARM_FEEDBACK_TIMEOUT_MS       200U

/* 连续发送失败达到该时间后进入安全停机。 */
#define ARM_TX_FAULT_TIME_MS          100U
#define ARM_TX_FAULT_CYCLES           (ARM_TX_FAULT_TIME_MS / ARM_CONTROL_PERIOD_MS)

/*
 * 反馈力矩过载保护：
 * 连续 100 ms 超过对应关节限值才判定为故障，避免瞬时噪声造成误停机。
 */
#define ARM_TORQUE_LIMIT_SHOULDER  12.0f
#define ARM_TORQUE_LIMIT_ELBOW      6.0f
#define ARM_TORQUE_LIMIT_WRIST      4.5f
#define ARM_TORQUE_FAULT_TIME_MS   100U
#define ARM_TORQUE_FAULT_CYCLES    (ARM_TORQUE_FAULT_TIME_MS / ARM_CONTROL_PERIOD_MS)

/* 温度字段仅用于调试观察，不参与故障判断。 */

/* arm_dbg.joint[] 的显示顺序与内部关节编号不同：肩、肘、腕 */
enum
{
    ARM_DBG_SHOULDER = 0,
    ARM_DBG_ELBOW = 1,
    ARM_DBG_WRIST = 2
};

/* ---- 调试全局变量（在 Keil Watch 中查看） ---- */
arm_dbg_t arm_dbg = {0};

/* 可在 Keil Watch 中在线修改的电机增益，默认值保持不变。 */
volatile float arm_kp_shoulder = 150.0f;
volatile float arm_kd_shoulder = 1.40f;
volatile float arm_kp_elbow    = 150.0f;
volatile float arm_kd_elbow    = 1.50f;
volatile float arm_kp_wrist    = 100.0f;
volatile float arm_kd_wrist    = 1.00f;


volatile float   arm_target[3] = {0.35f, 0.25f, 0.0f};  /* x、z、偏航角 */
volatile uint8_t arm_cmd_new = 0U;

/* ---- 控制器内部状态 ---- */
static FDCAN_HandleTypeDef *s_hfdcan = NULL;
static uint8_t  s_inited = 0U;
static uint8_t  s_target_set = 0U;  /* 收到第一次目标命令前为 0 */
/* 三个电机的反馈力矩过载连续计数，单位：控制周期数 */
static uint16_t s_torque_over_count[3] = {0U, 0U, 0U};
/* 任意控制帧连续发送失败的周期数。 */
static uint16_t s_tx_fail_count = 0U;

/* 实时关节状态：q0=腕，q1=肩，q2=肘，单位均为 rad */
static float s_cur_joint[3] = {0.0f, 0.0f, 0.0f};
static float s_tgt_joint[3] = {0.0f, 0.0f, 0.0f};

/* ============================== 电机配置 =============================== */

/* 配置 FDCAN、注册三个电机并设置各关节的控制参数。 */
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
    /* 与示例工程一致：AK80-9 的速度范围是 ±65 rad/s。 */
    mcfg.v_min = -65.0f;
    mcfg.v_max = 65.0f;
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
    rcfg.master_id = 0x11U;
    if (robstride_add(&rcfg) < 0)
    {
        return 1U;
    }

    return 0U;
}

/* ============================== 反馈处理 =============================== */

/* 读取三个电机的实时位置，并更新内部关节角和调试显示。 */
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

/* 检查单个 AK 电机的在线、反馈和错误状态。 */
static uint8_t arm_mit_feedback_ready(const mit_motor_state_t *st, uint32_t now)
{
    if (st == NULL)
    {
        return 0U;
    }
    if ((st->online == 0U) ||
        ((uint32_t)(now - st->last_rx_ms) > ARM_FEEDBACK_TIMEOUT_MS) ||
        (st->error != 0U))
    {
        return 0U;
    }
    return 1U;
}

/* 检查单个 RobStride 电机的在线、反馈和错误状态。 */
static uint8_t arm_robstride_feedback_ready(const robstride_state_t *st,
                                            uint32_t now)
{
    if (st == NULL)
    {
        return 0U;
    }
    if ((st->online == 0U) ||
        ((uint32_t)(now - st->last_rx_ms) > ARM_FEEDBACK_TIMEOUT_MS) ||
        (st->error != 0U) ||
        (st->pattern != 2U))
    {
        return 0U;
    }
    return 1U;
}

/* 检查三个关节电机的反馈是否全部正常；任一异常时返回 0，禁止继续控制。 */
static uint8_t arm_feedback_ready(uint32_t now)
{
    mit_motor_state_t *st1 = mit_motor_get_state(ARM_M1_ID);
    mit_motor_state_t *st2 = mit_motor_get_state(ARM_M2_ID);
    robstride_state_t *st3 = robstride_get_state(ARM_M3_ID);

    if ((arm_mit_feedback_ready(st1, now) == 0U) ||
        (arm_mit_feedback_ready(st2, now) == 0U) ||
        (arm_robstride_feedback_ready(st3, now) == 0U))
    {
        return 0U;
    }

    return 1U;
}

/* 发送 EL05 使能并等待 pattern==2 的反馈确认；最多尝试 3 次。 */
static uint8_t arm_enable_el05_checked(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t attempt;
    uint32_t deadline;
    uint32_t now;
    robstride_state_t *st3;

    if (hfdcan == NULL)
    {
        return 0U;
    }

    for (attempt = 0U; attempt < ARM_EL05_ENABLE_MAX_TRIES; attempt++)
    {
        if (robstride_enable(hfdcan, ARM_M3_ID) != 0U)
        {
            HAL_Delay(1U);
            continue;
        }

        deadline = HAL_GetTick() + ARM_EL05_ENABLE_ACK_MS;
        for (;;)
        {
            arm_refresh_feedback();
            now = HAL_GetTick();
            st3 = robstride_get_state(ARM_M3_ID);

            if (arm_robstride_feedback_ready(st3, now) != 0U)
            {
                return 1U;
            }
            if ((int32_t)(now - deadline) >= 0)
            {
                break;
            }
            HAL_Delay(1U);
        }
    }

    return 0U;
}

/* 根据反馈力矩判断是否连续过载；返回 1 表示需要安全停机。 */
static uint8_t arm_torque_overload_check(void)
{
    mit_motor_state_t *st1 = mit_motor_get_state(ARM_M1_ID);
    mit_motor_state_t *st2 = mit_motor_get_state(ARM_M2_ID);
    robstride_state_t *st3 = robstride_get_state(ARM_M3_ID);
    float torque[3];
    float limit[3];
    uint8_t i;

    if ((st1 == NULL) || (st2 == NULL) || (st3 == NULL))
    {
        return 0U;
    }

    torque[0] = st1->torque;
    torque[1] = st2->torque;
    torque[2] = st3->torque;

    limit[0] = ARM_TORQUE_LIMIT_SHOULDER;
    limit[1] = ARM_TORQUE_LIMIT_ELBOW;
    limit[2] = ARM_TORQUE_LIMIT_WRIST;

    for (i = 0U; i < 3U; i++)
    {
        if (fabsf(torque[i]) > limit[i])
        {
            if (s_torque_over_count[i] < ARM_TORQUE_FAULT_CYCLES)
            {
                s_torque_over_count[i]++;
            }
            if (s_torque_over_count[i] >= ARM_TORQUE_FAULT_CYCLES)
            {
                return 1U;
            }
        }
        else
        {
            s_torque_over_count[i] = 0U;
        }
    }

    return 0U;
}
/* ============================== 电机发送 =============================== */

/* 连续发送失败计数；任意一帧失败都会累计，成功后清零。 */
static uint8_t arm_tx_failure_update(uint8_t send_failed)
{
    if (send_failed == 0U)
    {
        s_tx_fail_count = 0U;
        return 0U;
    }

    if (s_tx_fail_count < ARM_TX_FAULT_CYCLES)
    {
        s_tx_fail_count++;
    }

    return (s_tx_fail_count >= ARM_TX_FAULT_CYCLES) ? 1U : 0U;
}

/*
 * 计算前馈力矩并向三个电机发送位置、速度和增益控制帧。
 * 返回发送失败位图：bit0=腕，bit1=肩，bit2=肘。
 */
static uint8_t arm_send_motors(float q0, float q1, float q2,
                               float v0, float v1, float v2)
{
    float motor_shoulder = arm_joint_to_motor_1(q1);
    float motor_elbow = arm_joint_to_motor_2(q2);
    float torque_shoulder = 0.0f;
    float torque_elbow = 0.0f;
    float torque_wrist_ff = 0.0f;
    uint8_t send_failed = 0U;

    arm_gravity_get(s_cur_joint[1], s_cur_joint[2], ARM_CONTROL_DT,
                    &torque_shoulder, &torque_elbow);
    torque_wrist_ff = arm_wrist_gravity_get(s_cur_joint[0],
                                            s_cur_joint[1],
                                            s_cur_joint[2],
                                            ARM_L3_LEVEL_C,
                                            ARM_CONTROL_DT);

    /* 先发送 EL05，避免两个 AK 帧占满 Tx FIFO。 */
    if (robstride_set_control(s_hfdcan, ARM_M3_ID, torque_wrist_ff,
                              q0, v0, arm_kp_wrist, arm_kd_wrist) != FDCAN_DRV_OK)
    {
        send_failed |= 0x01U;
    }
    if (mit_motor_set_control(s_hfdcan, ARM_M1_ID, motor_shoulder, v1,
                              arm_kp_shoulder, arm_kd_shoulder,
                              torque_shoulder) != FDCAN_DRV_OK)
    {
        send_failed |= 0x02U;
    }
    if (mit_motor_set_control(s_hfdcan, ARM_M2_ID, motor_elbow, v2,
                              arm_kp_elbow, arm_kd_elbow,
                              torque_elbow) != FDCAN_DRV_OK)
    {
        send_failed |= 0x04U;
    }

    return send_failed;
}

/* 向三个电机发送零增益、零力矩安全帧。 */
static void arm_send_safe_idle(void)
{
    /* fdcan_drv_send() 内部会检查 Tx FIFO，失败会在下一次循环重试。 */
    (void)robstride_set_control(s_hfdcan, ARM_M3_ID,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    (void)mit_motor_set_control(s_hfdcan, ARM_M1_ID,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    (void)mit_motor_set_control(s_hfdcan, ARM_M2_ID,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/* ============================== 安全停机 =============================== */

/* 停止轨迹规划、关闭三个电机并锁存故障状态。 */
void arm_control_fail_safe(int16_t error_code)
{
    arm_traj_stop();
    s_tx_fail_count = 0U;

    if (s_hfdcan != NULL)
    {
        mit_motor_disable(s_hfdcan, ARM_M1_ID);
        mit_motor_disable(s_hfdcan, ARM_M2_ID);
        robstride_disable(s_hfdcan, ARM_M3_ID, 0U);
    }

    s_inited = 0U;
    arm_dbg.last_err = error_code;
}

/* ============================== 公开接口 =============================== */

/* 初始化 FDCAN 和三个电机，等待有效反馈并建立初始保持位置。 */
void arm_control_init(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t deadline;

    s_hfdcan = hfdcan;
    s_inited = 0U;
    s_target_set = 0U;
    arm_cmd_new = 0U;
    arm_l3_level_stop();
    arm_traj_stop();
    s_torque_over_count[0] = 0U;
    s_torque_over_count[1] = 0U;
    s_torque_over_count[2] = 0U;
    s_tx_fail_count = 0U;

    if (arm_setup_motors(hfdcan) != 0U)
    {
        arm_control_fail_safe(-2);
        return;
    }

    /* AK 电机只使能一次；EL05 由带反馈确认的有限重试流程使能。 */
    HAL_Delay(800U);
    mit_motor_enable(hfdcan, ARM_M1_ID);
    mit_motor_enable(hfdcan, ARM_M2_ID);

    if (arm_enable_el05_checked(hfdcan) == 0U)
    {
        arm_control_fail_safe(-2);
        return;
    }

    /* EL05 确认成功后，再发送安全空控制帧并等待三个电机反馈齐全。 */
    deadline = HAL_GetTick() + ARM_STARTUP_TIMEOUT_MS;
    for (;;)
    {
        uint32_t now;

        arm_send_safe_idle();
        arm_refresh_feedback();
        now = HAL_GetTick();

        if (arm_feedback_ready(now) != 0U)
        {
            break;
        }
        if ((int32_t)(now - deadline) >= 0)
        {
            arm_control_fail_safe(-2);
            return;
        }
        HAL_Delay(1U);
    }

    /* 本轮反馈已经刷新，直接作为初始保持目标。 */
    s_tgt_joint[0] = s_cur_joint[0];
    s_tgt_joint[1] = s_cur_joint[1];
    s_tgt_joint[2] = s_cur_joint[2];
    arm_l3_level_init(s_cur_joint[0]);

    arm_dbg.x = arm_dbg.x_actual;
    arm_dbg.z = arm_dbg.z_actual;
    arm_dbg.reached = 0U;
    arm_dbg.last_err = 0;
    s_target_set = 0U;
    s_inited = 1U;
}

/* 求解逆运动学并启动轨迹；成功返回 0，失败返回 -1。 */
int arm_goto(float x, float z, float yaw)
{
    float q0;
    float q1;
    float q2;

    if (s_inited == 0U)
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

/* 执行一次 2 ms 控制更新，包括反馈、安全检查、轨迹和电机输出。 */
void arm_control_step(float dt)
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

    if (s_inited == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    /* 控制算法固定使用 2 ms 周期，避免 HAL_GetTick 的 1 ms 量化造成 dt 抖动。 */
    l3_max_step = ARM_L3_MAX_SPEED * dt;

    arm_refresh_feedback();
		//电机状态检查
    if (arm_feedback_ready(now) == 0U)
    {
        arm_control_fail_safe(-2);
        return;
    }
		//电机力矩过载检查
    if (arm_torque_overload_check() != 0U)
    {
        arm_control_fail_safe(-3);
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
        trajectory_done = arm_traj_update(dt,
                                          &q0, &q1, &q2,
                                          &v0, &v1, &v2);

        if (arm_l3_level_is_active() != 0U)
        {
            q0 = arm_l3_level_update(q1, q2, v1, v2,
                                     l3_max_step, &v0);
        }

        arm_clamp_joints(&q0, &q1, &q2);
        if (arm_tx_failure_update(arm_send_motors(q0, q1, q2,
                                                  v0, v1, v2)) != 0U)
        {
            arm_control_fail_safe(-4);
            return;
        }

        if (trajectory_done != 0)
        {
            /* 五次多项式执行完成。 */
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

    if (arm_tx_failure_update(arm_send_motors(s_tgt_joint[0],
                                              s_tgt_joint[1],
                                              s_tgt_joint[2],
                                              0.0f, 0.0f, 0.0f)) != 0U)
    {
        arm_control_fail_safe(-4);
        return;
    }
}
