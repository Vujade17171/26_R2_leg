/**
  ******************************************************************************
  * @file    leg_task.c
  * @brief   Leg_Task FreeRTOS 任务实现（空任务骨架）
  *
  * 说明：
  *  - 本文件提供 leg_task() 的强定义，覆盖 CubeMX 在 freertos.c 中生成的
  *    __weak void leg_task(void *argument)；
  *  - 任务配置（CubeMX .ioc）：名称 Leg_Task，优先级 osPriorityLow(8)，
  *    堆栈 256*4 字节，CMSIS-RTOS v2；
  *  - 在下方两个 USER CODE 区域中填写实际控制逻辑即可。
  ******************************************************************************
  */
#include "leg_task.h"
#include "ak_motor.h"
#include "fdcan.h"    
#include "kinematics.h"  /* 腿部运动学接口：FK/IK、关节角<->电机角换算、重力补偿等 */
#include "joint_traj.h"  /* 关节空间五次多项式轨迹：Plan/Update/Abort */
#include "bsp_dwt.h"     /* DWT_GetDeltaT：实测调用周期 */
#include "Mycan.h"       /* g_rx_frames：反馈帧计数，用于等待驱动板反馈 */
#include "EL05_motor.h"  /* EL05 电机驱动（扩展帧私有协议，Mycan 分流用） */

/* 故障时"保持位姿"用的刚度/阻尼（比正常跟踪低，避免故障下继续大力输出） */
#define ARM_FAULT_HOLD_KP   40.0f
#define ARM_FAULT_HOLD_KD    1.5f

/* 控制周期 (s)：arm_control_tick 的**真实**调用周期是 2ms ——
 * 主循环每一轮里执行了两次 "tick += 1; osDelayUntil(tick);"（AK 帧 → +1ms → EL05 帧
 * → +1ms → 回到 AK 帧），所以两拍 AK 之间隔的是 2ms，不是 1ms。
 * 实际周期由 DWT 实测（见主循环 dt）；下面这个值仅在 DWT 异常时兜底。 */
#define ARM_CTRL_DT_S       0.002f

/* dt 来源开关：1 = DWT 实测周期（推荐）；0 = 固定使用 ARM_CTRL_DT_S */
#define ARM_DT_USE_DWT      1

/* 轨迹默认时长 (s)：目标点变化时按这个时长规划（曲线 0→1 走完全程） */
#define ARM_TRAJ_DURATION_S 0.8f

/* 目标点变化判定阈值 (m)：小于它认为"目标没变"，不重新规划 */
#define ARM_TARGET_EPS      0.001f

AK_Motor motors[2];  /* 电机句柄数组：motors[0]=大臂(肩)，motors[1]=小臂(肘) */

LegLinkParam leg_link_param = { D_L1, D_L2, D_L3 };  /* 连杆参数：大臂/小臂/腕部长度，单位 m */

LegJointAngles motor_angles = { 0.0f, 0.0f, 0.0f }; //当前关节角(由电机反馈换算)

FootPosition target_pos = { 0.1f, 0.3f }; //目标末端位置 x,z

FootPosition FK_pos = { 0.0f, 0.0f }; //正解得到的当前末端位置 x,z

volatile float b,c,d;   /* 调试用：b=肩前馈, c=肘前馈, d=腕(EL05)前馈 */

volatile LegJointAngles q_cmd_dbg = { 0.0f, 0.0f, 0.0f };  /* 调试用：限速后的关节指令角（对比 motor_angles） */

/* ================== 关节空间轨迹（五次多项式）状态 ================== */
/* 轨迹实例：显式初始化为 IDLE（全零初值 state=0 不是合法状态码） */
static JointTraj    g_traj = { JTRAJ_IDLE, 0.0f, 0.0f, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };
static FootPosition g_target_planned = { 0.0f, 0.0f };    /* 已规划的目标点（用于目标去重） */
static FootPosition g_target_pending = { 0.0f, 0.0f };    /* 运行中收到的新目标：锁存到本段跑完 */
static uint8_t      g_target_pending_valid = 0u;
static uint32_t     s_dwt_cnt = 0u;                       /* 轨迹/限速共用的实测周期计数 */
static float        g_move_duration = ARM_TRAJ_DURATION_S;/* 本次运动的规划时长 */

/* 调试观测（Watch 用，风格对齐 q_cmd_dbg） */
volatile JointTrajVec v_cmd_dbg = { 0.0f, 0.0f, 0.0f };   /* 本拍前馈速度指令 rad/s */
volatile JointTrajVec a_cmd_dbg = { 0.0f, 0.0f, 0.0f };   /* 本拍前馈加速度指令 rad/s² */
volatile float        dbg_tau = 0.0f;                     /* 轨迹进度 0~1 */
volatile uint8_t      dbg_traj_state = JTRAJ_IDLE;        /* 轨迹状态（JTRAJ_*） */
volatile float        dbg_dt = 0.0f;                      /* 实测控制周期 s */

/* 电机数量：供 Mycan 接收回调按 sizeof 自动计算，新增电机无需改这里 */
const uint8_t g_ak_motor_num = (uint8_t)(sizeof(motors) / sizeof(motors[0]));

/* EL05 电机句柄（扩展帧协议）；ID/主机ID 在 Init 时按实际设置 */
EL05_Motor el05_motors[1];
const uint8_t g_el05_motor_num = (uint8_t)(sizeof(el05_motors) / sizeof(el05_motors[0]));

/* ================== 机械臂控制接口 ==================
 * 分层：arm_control(x,y) 只登记笛卡尔目标 → arm_control_tick() 内部做
 *       "目标去重/规划 → 轨迹推进 → 关节层下发"。 */

/* 设置笛卡尔目标点 x,z（只登记，实际规划在 arm_control_tick 中按目标变化触发） */
void arm_control(float x, float y);

/* 带时长的目标入口：duration_s <= 0 时用默认 ARM_TRAJ_DURATION_S */
void leg_move_to(float x, float z, float duration_s);

static void    arm_control_tick(float dt);
static uint8_t arm_plan_to(float x, float z, float duration_s);
static void    arm_control_joint(const LegJointAngles *q_des,
                                 const JointTrajVec *v_des,
                                 const JointTrajVec *a_des,
                                 float dt);
static void    arm_fault_hold(void);

//========================================任务函数=========================================
void leg_task(void *argument)
{


  AK_Motor_Init(&motors[0], &hfdcan1, 1, &AK_MODEL_AK80_9);   /* 大臂 */
  AK_Motor_Init(&motors[1], &hfdcan1, 2, &AK_MODEL_AK45_10);  /* 小臂 */
  EL05_Init(&el05_motors[0], &hfdcan1, 3, 0xFF, &EL05_MODEL);   /* EL05: 电机CAN_ID=3 */
  
  AK_Motor_Enable(&motors[0]); 
  AK_Motor_Enable(&motors[1]); 
	osDelay(1);
  EL05_Enable(&el05_motors[0]);

  Kinematics_Init(&leg_link_param);
  DWT_Init(480);          /* CPU 480MHz：供 DWT_GetDeltaT 算实际周期 */
  osDelay(10);

  /* 等驱动板回传反馈（最多 500ms）：有真实反馈后再取初始姿态，
   * 否则 pos_rad 仍为 0，会算出虚假姿态当目标 → 上电大角度跳动 */
  {
    uint32_t wait_ms = 0u;
    while ((g_rx_frames == 0u) && (wait_ms < 500u)) { osDelay(1); wait_ms++; }
  }

  /* 上电初值：以"当前实际末端位置"为目标，先保持姿态(避免上电跳变) */
  motor_angles.q1 = motor_to_joint_1(motors[0].pos_rad);
  motor_angles.q2 = motor_to_joint_2(motors[1].pos_rad);
  motor_angles.q3 = 0.0f;
  Kinematics_Forward(&motor_angles, &target_pos);

  /* 关节限速器：以当前实际关节角为指令起点（上电首拍不跳变） */
  {
    LegJointAngles q_now_all = motor_angles;
    q_now_all.q3 = -el05_motors[0].pos_rad;   /* 腕：EL05 电机角取反即关节角 */
    joint_rate_limit_reset(&q_now_all);
  }

  (void)DWT_GetDeltaT(&s_dwt_cnt);   /* 预置 DWT 计数，避免首拍算出巨大的 dt */

  {
    uint32_t tick = osKernelGetTickCount();
    float dt;

    for (;;)
    {
      /* 实测控制周期：轨迹推进 / 限速 / 前馈共用同一个 dt
       * （重力补偿内部自己也调 DWT，计数器独立，互不影响） */
#if (ARM_DT_USE_DWT != 0)
      dt = DWT_GetDeltaT(&s_dwt_cnt);
#else
      dt = ARM_CTRL_DT_S;
#endif
      if (dt <= 0.0f) { dt = ARM_CTRL_DT_S; }   /* 异常周期：退回名义值，防止限速被绕过 */
      if (dt > 0.05f) { dt = 0.05f; }           /* 卡顿/断点：上限 50ms */

      arm_control_tick(dt);

      tick += 1u;
      osDelayUntil(tick);

			EL05_MotionControl(&el05_motors[0], joint_to_motor_3(3.141592f-(b+c)), 0.0f, 40.0f, 1.0f, d);

      tick += 1u;
      osDelayUntil(tick);
    }
  }
}

// ============================================================================

/* ============================================================================
 * 目标入口层
 * ============================================================================ */

/* 设置笛卡尔目标点 x,z（签名与原来一致）：只登记目标，不规划、不下发。
 * 实际规划由 arm_control_tick 按"目标变化"触发，故本函数可被上位机/调试随时调用。
 * 注意：轨迹正在运行时重复调用只会锁存最新目标，等本段跑完再执行。 */
void arm_control(float x, float y)
{
    target_pos.x = x;
    target_pos.z = y;
}

/* 带时长的目标入口：duration_s <= 0 时用默认 ARM_TRAJ_DURATION_S */
void leg_move_to(float x, float z, float duration_s)
{
    g_move_duration = (duration_s > 0.0f) ? duration_s : ARM_TRAJ_DURATION_S;
    arm_control(x, z);
}

/* ============================================================================
 * 关节层：期望关节量 → 限速 → 重力补偿 → MIT 下发
 * ============================================================================ */
static void arm_control_joint(const LegJointAngles *q_des,
                              const JointTrajVec *v_des,
                              const JointTrajVec *a_des,
                              float dt)
{
    LegJointAngles q_tmp, q_cmd;                          /* 限速后的指令关节角 */
    float tau_sh = 0.0f, tau_el = 0.0f, tau_wr = 0.0f;    /* 重力补偿力矩 N·m */

    (void)a_des;   /* 本轮不使用加速度前馈；将来接前馈时按 τ = gain·I·a/gear 折算后并入 tau_sh/tau_el */

    /* 1. 关节空间限速：限制每拍关节角增量（= 摆动速率上限，参数在 kinematics.h 顶部）
     *    轨迹正常时不该被削——被削说明 duration 给太短，或限速值设太小 */
    q_tmp    = *q_des;
    q_tmp.q3 = -el05_motors[0].pos_rad;   /* 腕：指令保持当前角（与原来一致） */
    joint_rate_limit(&q_tmp, dt, &q_cmd);
    q_cmd_dbg = q_cmd;                    /* 调试：Watch 里对比 q_cmd_dbg 与 motor_angles */

    /* 2. 重力补偿（杠杆原理）：τ = Σ 质量 × g × 到质心的水平力臂，
     *    再除以 gear（当前工程数值）折算到电机侧，并做限幅 + 变化率限制 */
    get_gravity_comp_torque(motor_angles.q1, motor_angles.q2, q_tmp.q3,
                            &tau_sh, &tau_el, &tau_wr);
    b = motor_angles.q1;
    c = motor_angles.q2;
    d = tau_wr;

    /* 3. MIT 下发：位置/力矩口径不变；第 2 个参数由 0.0f 换成轨迹前馈速度 v_des。
     *    - v 是前馈速度（配 kd 起阻尼/超前），不是限速指令；
     *    - 关节↔电机换算是"加偏移、增益 1、方向 +1"（kinematics.c），故速度数值 1:1 直接传；
     *      将来若给 joint_to_motor_x / motor_to_joint_x 加负号，这里必须同步取反；
     *    - 前馈力矩为【电机侧口径】(已除 gear) + 限幅/变化率限制；
     *      变化率周期取自 DWT，故必须在任务启动前调用过 DWT_Init()。 */
    AK_Motor_MIT(&motors[0], joint_to_motor_1(q_cmd.q1), v_des->q1, 30.0f, 3.0f, tau_sh);
    AK_Motor_MIT(&motors[1], joint_to_motor_2(q_cmd.q2), v_des->q2, 30.0f, 3.0f, tau_el);

    /* 若实测补偿过强/过弱，改 kinematics.c 里的 gear、dir 或质量参数。
     * EL05：位置 = 限速后的指令角（关节->电机：m = −q3，与"保持"指令等价）；
     *       当前 kp=kd=0，只出力矩，腕限速待接位置指令后自然生效 */
//力控测试
//     AK_Motor_MIT(&motors[0], 0.0f, 0.0f, 0.0f, 0.0f, tau_sh);
//     AK_Motor_MIT(&motors[1], 0.0f, 0.0f, 0.0f, 0.0f, tau_el);
//零力矩测试
//     AK_Motor_MIT(&motors[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//     AK_Motor_MIT(&motors[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/* ============================================================================
 * 故障保持：任一电机报错 → 放弃轨迹 + 保持当前位姿
 * ============================================================================ */
static void arm_fault_hold(void)
{
    float q1_hold = motor_to_joint_1(motors[0].pos_rad);
    float q2_hold = motor_to_joint_2(motors[1].pos_rad);

    /* 必须 Abort：否则故障期间 elapsed 仍在推进，恢复后会瞬间跳到轨迹后面的位置 */
    JointTraj_Abort(&g_traj);
    g_target_pending_valid = 0u;

    AK_Motor_MIT(&motors[0], joint_to_motor_1(q1_hold), 0.0f,
                 ARM_FAULT_HOLD_KP, ARM_FAULT_HOLD_KD, 0.0f);
    AK_Motor_MIT(&motors[1], joint_to_motor_2(q2_hold), 0.0f,
                 ARM_FAULT_HOLD_KP, ARM_FAULT_HOLD_KD, 0.0f);
}

/* ============================================================================
 * 笛卡尔层：目标点 → IK → 规划轨迹（只规划，不下发）
 * ============================================================================ */
static uint8_t arm_plan_to(float x, float z, float duration_s)
{
    FootPosition target;
    LegJointAngles q_now, q_goal;

    target.x = x;
    target.z = z;

    /* 反馈 → 当前关节角（含腕：EL05 电机角取反 = 关节角） */
    q_now.q1 = motor_to_joint_1(motors[0].pos_rad);
    q_now.q2 = motor_to_joint_2(motors[1].pos_rad);
    q_now.q3 = -el05_motors[0].pos_rad;

    /* 逆解：失败（不可达 / 无满足限位的解）→ 放弃本次规划，调用方保持当前位姿。
     * 与原实现"IK 失败则期望角 = 当前实际角"的语义一致。 */
    q_goal = q_now;
    if (Kinematics_Inverse(&target, &q_goal, -1) != 0) {
        return JTRAJ_ERR_LIMIT;
    }

    /* 腕不进轨迹：终点取当前腕角 → Δ3 = 0，腕通道恒等于规划时刻的角度 */
    q_goal.q3 = q_now.q3;

    if (JointTraj_Plan(&g_traj, &q_now, &q_goal, duration_s) != JTRAJ_OK) {
        return JTRAJ_ERR_LIMIT;   /* 参数非法 / 终点超机械限位 */
    }

    /* 限速器与轨迹起点对齐：否则限速器内部 g_q_cmd 还停在旧值，会有一段追赶 */
    joint_rate_limit_reset(&q_now);

    g_target_planned = target;
    return JTRAJ_OK;
}

/* ============================================================================
 * 主循环唯一入口：读反馈 → 目标管理(规划/锁存) → 轨迹推进 → 关节层下发
 * 无可用轨迹时（未规划 / IK 失败 / 超限 / 故障已 Abort）保持当前位姿。
 * ============================================================================ */
static void arm_control_tick(float dt)
{
    LegJointAngles q_now, q_des;
    JointTrajVec v_des, a_des;
    uint8_t st;

    /* 0. 安全检查：任一电机报错 → 保持当前位姿并放弃轨迹，等故障消失自动恢复 */
    if ((motors[0].err_code != 0u) || (motors[1].err_code != 0u)) {
        arm_fault_hold();
        dbg_traj_state = g_traj.state;
        dbg_dt         = dt;
        return;
    }

    /* 1. 反馈 → 当前关节角（电机角->关节角，含零位/方向换算） */
    motor_angles.q1 = motor_to_joint_1(motors[0].pos_rad);   /* 大臂 */
    motor_angles.q2 = motor_to_joint_2(motors[1].pos_rad);   /* 小臂 */
    motor_angles.q3 = 0.0f;

    q_now.q1 = motor_angles.q1;
    q_now.q2 = motor_angles.q2;
    q_now.q3 = -el05_motors[0].pos_rad;                      /* 腕：EL05 电机角取反 */

    /* 2. 正解：当前关节角 → 当前末端位置（保留：调试/观测用） */
    Kinematics_Forward(&motor_angles, &FK_pos);

    /* 3. 目标管理：目标变了才规划；正在跑就锁存，等本段 DONE 再规划
     *    （避免中途重启时"零初速五次多项式"从运动状态接手造成的速度跳变） */
    if ((fabsf(target_pos.x - g_target_planned.x) > ARM_TARGET_EPS) ||
        (fabsf(target_pos.z - g_target_planned.z) > ARM_TARGET_EPS))
    {
        if (g_traj.state == JTRAJ_RUNNING) {
            g_target_pending       = target_pos;   /* 锁存最新目标（后到覆盖先到） */
            g_target_pending_valid = 1u;
        } else {
            (void)arm_plan_to(target_pos.x, target_pos.z, g_move_duration);
            g_target_pending_valid = 0u;
        }
    }
    else if ((g_target_pending_valid != 0u) && (g_traj.state != JTRAJ_RUNNING))
    {
        (void)arm_plan_to(g_target_pending.x, g_target_pending.z, g_move_duration);
        g_target_pending_valid = 0u;
    }

    /* 4. 轨迹推进（IDLE 时出参不被写入 → 本拍退化为"保持当前位姿"） */
    st = JointTraj_Update(&g_traj, dt, &q_des, &v_des, &a_des);
    if (st == JTRAJ_IDLE) {
        q_des.q1 = q_now.q1;
        q_des.q2 = q_now.q2;
        q_des.q3 = q_now.q3;
        v_des.q1 = 0.0f; v_des.q2 = 0.0f; v_des.q3 = 0.0f;
        a_des.q1 = 0.0f; a_des.q2 = 0.0f; a_des.q3 = 0.0f;
    }

    /* 5. 调试观测（Watch） */
    dbg_traj_state = g_traj.state;
    dbg_tau        = (g_traj.duration > 0.0f) ? (g_traj.elapsed / g_traj.duration) : 0.0f;
    dbg_dt         = dt;
    v_cmd_dbg      = v_des;
    a_cmd_dbg      = a_des;

    /* 6. 关节层：限速 → 重力补偿 → MIT 下发 */
    arm_control_joint(&q_des, &v_des, &a_des, dt);
}
