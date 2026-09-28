/*****************************************************************************
 * leg_motion.c —— 腿部运动服务实现
 *
 * 控制流（每拍）：
 *   LegMotion_Step(dt)
 *     0. 任一 AK 电机报错 -> 放弃轨迹 + 保持当前位姿（撤回，不刷新 fb/tau_wr）
 *     1. 反馈 -> 关节角快照 fb（电机角 -> 关节角，含零位/方向换算）
 *     2. FK(fb) -> foot_fb（调试/观测）
 *     3. 目标管理：目标变了才规划；运行中则锁存，等本段 DONE 再规划
 *     4. 轨迹推进 -> q_des/v_des/a_des（IDLE 时退化为"保持当前位姿"）
 *     5. 观测量快照
 *     6. 关节层：限速 -> 重力补偿 -> MIT 下发
 *   LegMotion_WristStep()
 *     腕部位置保持"末端水平"（π − (q1+q2)）并叠加腕重力补偿力矩
 *
 * 本文件无任何可变全局量；所有状态都在 LegMotion 句柄里。
 *****************************************************************************/
#include "leg_motion.h"

#include <math.h>    /* fabsf */

#define LEG_PI  3.141592f

/* 默认配置（数值与旧 leg_task.c 顶部宏一一对应） */
const LegMotionCfg LEG_MOTION_CFG_DEFAULT = {
    .traj_duration_s = 0.8f,     /* 旧 ARM_TRAJ_DURATION_S   */
    .target_eps      = 0.005f,   /* 旧 ARM_TARGET_EPS        */
    .dt_fallback_s   = 0.002f,   /* 旧 ARM_CTRL_DT_S         */
    .dt_max_s        = 0.05f,    /* 旧 dt > 0.05f 钳位        */
    .kp              = 30.0f,    /* 旧 MIT 下发 kp            */
    .kd              = 3.0f,     /* 旧 MIT 下发 kd            */
    .fault_hold_kp   = 40.0f,    /* 旧 ARM_FAULT_HOLD_KP     */
    .fault_hold_kd   = 1.5f,     /* 旧 ARM_FAULT_HOLD_KD     */
    .wrist_kp        = 40.0f,    /* 旧 EL05_MotionControl kp */
    .wrist_kd        = 1.0f      /* 旧 EL05_MotionControl kd */
};

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */
int32_t LegMotion_Init(LegMotion *lm, AK_Motor *ak, uint8_t ak_num,
                       EL05_Motor *el05, uint8_t el05_num,
                       const LegLinkParam *link,
                       const LegMotionCfg *cfg, const GravCfg *grav_cfg)
{
    if ((lm == 0) || (ak == 0) || (el05 == 0) || (link == 0)) { return SER_ERR_PARAM; }
    if ((ak_num < 2u) || (el05_num < 1u)) { return SER_ERR_PARAM; }

    /* 清零：state 全 0 恰好等于 JTRAJ_IDLE / 未初始化限速器 / 无锁存目标 */
    {
        uint8_t *p = (uint8_t *)lm;
        uint32_t i;
        for (i = 0u; i < (uint32_t)sizeof(LegMotion); i++) { p[i] = 0u; }
    }

    lm->ak       = ak;
    lm->ak_num   = ak_num;
    lm->el05     = el05;
    lm->el05_num = el05_num;

    lm->cfg  = (cfg != 0) ? *cfg : LEG_MOTION_CFG_DEFAULT;
    lm->link = *link;

    lm->move_duration_s = lm->cfg.traj_duration_s;

    (void)JointTraj_Abort(&lm->traj);
    GravityComp_Init(&lm->grav, grav_cfg);

    /* 运动学连杆参数：本服务负责初始化，app 不必再调一次 */
    return Kinematics_Init(link);
}

/* ---------------------------------------------------------------------------
 * 反馈快照：电机角 -> 关节角（含零位/方向换算）
 * 与旧 arm_control_tick 第 1 步完全一致。
 * ------------------------------------------------------------------------- */
static void refresh_feedback(LegMotion *lm)
{
    lm->fb.q1 = motor_to_joint_1(lm->ak[0].pos_rad);
    lm->fb.q2 = motor_to_joint_2(lm->ak[1].pos_rad);
    lm->fb.q3 = -lm->el05[0].pos_rad;    /* 腕：EL05 电机角取反 = 关节角 */
}

/* 正解：旧实现传入的 motor_angles.q3 恒为 0，这里保持一致 */
static void refresh_foot(LegMotion *lm)
{
    LegJointAngles q = lm->fb;

    q.q3 = 0.0f;
    Kinematics_Forward(&q, &lm->foot_fb);
}

/* ---------------------------------------------------------------------------
 * 上电初始化姿态
 * 与旧实现一致：FK 用 q3=0 算末端位置当作目标；限速器以"含腕的实际关节角"复位。
 * 注意：故意【不】把 target_planned 设成当前点 —— 保持旧行为（首个 Step 会规划
 * 一条 Δ≈0 的轨迹，输出与"保持"完全等价）。
 * ------------------------------------------------------------------------- */
int32_t LegMotion_InitPose(LegMotion *lm)
{
    if (lm == 0) { return SER_ERR_PARAM; }

    refresh_feedback(lm);
    refresh_foot(lm);

    lm->target = lm->foot_fb;

    JointRateLimit_Init(&lm->ratelim, &lm->fb);
    return SER_OK;
}

/* ---------------------------------------------------------------------------
 * 目标登记
 * ------------------------------------------------------------------------- */
void LegMotion_SetTarget(LegMotion *lm, float x, float z, float duration_s)
{
    if (lm == 0) { return; }

    if (duration_s > 0.0f) { lm->move_duration_s = duration_s; }

    lm->target.x = x;
    lm->target.z = z;
}

void LegMotion_Abort(LegMotion *lm)
{
    if (lm == 0) { return; }
    JointTraj_Abort(&lm->traj);
    lm->target_pending_valid = 0u;
}

/* ---------------------------------------------------------------------------
 * 笛卡尔层：目标点 -> IK -> 规划轨迹（只规划，不下发）
 * ------------------------------------------------------------------------- */
static int32_t plan_to(LegMotion *lm, float x, float z, float duration_s)
{
    FootPosition   target;
    LegJointAngles q_now, q_goal;

    target.x = x;
    target.z = z;

    /* 反馈 -> 当前关节角（含腕） */
    q_now = lm->fb;

    /* 逆解：失败（不可达 / 无满足限位的解）-> 放弃本次规划，调用方保持当前位姿。
     * 与旧实现"IK 失败则期望角 = 当前实际角"的语义一致。 */
    q_goal = q_now;
    if (Kinematics_Inverse(&target, &q_goal, -1) != SER_OK) {
        return SER_ERR_LIMIT;
    }

    /* 腕不进轨迹：终点取当前腕角 → Δ3 = 0，腕通道恒等于规划时刻的角度 */
    q_goal.q3 = q_now.q3;

    if (JointTraj_Plan(&lm->traj, &q_now, &q_goal, duration_s) != SER_OK) {
        return SER_ERR_LIMIT;   /* 参数非法 / 终点超机械限位 */
    }

    /* 限速器与轨迹起点对齐：否则限速器内部还停在旧值，会有一段追赶 */
    JointRateLimit_Init(&lm->ratelim, &q_now);

    lm->target_planned = target;
    return SER_OK;
}

/* ---------------------------------------------------------------------------
 * 故障保持：任一 AK 电机报错 -> 放弃轨迹 + 保持当前位姿
 * 与旧实现一致：不刷新 fb / tau_wr，腕部继续沿用上一拍的值。
 * ------------------------------------------------------------------------- */
static void fault_hold(LegMotion *lm)
{
    float q1_hold = motor_to_joint_1(lm->ak[0].pos_rad);
    float q2_hold = motor_to_joint_2(lm->ak[1].pos_rad);

    /* 必须 Abort：否则故障期间 elapsed 仍在推进，恢复后会瞬间跳到轨迹后面的位置 */
    LegMotion_Abort(lm);

    (void)AK_Motor_MIT(&lm->ak[0], joint_to_motor_1(q1_hold), 0.0f,
                       lm->cfg.fault_hold_kp, lm->cfg.fault_hold_kd, 0.0f);
    (void)AK_Motor_MIT(&lm->ak[1], joint_to_motor_2(q2_hold), 0.0f,
                       lm->cfg.fault_hold_kp, lm->cfg.fault_hold_kd, 0.0f);
}

/* ---------------------------------------------------------------------------
 * 关节层：期望关节量 -> 限速 -> 重力补偿 -> MIT 下发
 * ------------------------------------------------------------------------- */
static void joint_output(LegMotion *lm, const LegJointAngles *q_des,
                         const LegJointVec *v_des, float dt)
{
    LegJointAngles q_tmp, q_cmd;
    float tau_sh = 0.0f, tau_el = 0.0f, tau_wr = 0.0f;

    /* 1. 关节空间限速：腕通道指令保持当前角（与旧实现一致） */
    q_tmp    = *q_des;
    q_tmp.q3 = lm->fb.q3;

    JointRateLimit_Apply(&lm->ratelim, &q_tmp, dt, &q_cmd);
    lm->q_cmd = q_cmd;

    /* 2. 重力补偿（杠杆原理）：τ = Σ 质量 × g × 到质心的水平力臂，
     *    再除以 gear 折算到电机侧，并做限幅 + 变化率限制。
     *    dt 由调用方实测传入（旧实现是模块内部自己读 DWT）。 */
    GravityComp_Update(&lm->grav, &lm->link, lm->fb.q1, lm->fb.q2, q_tmp.q3, dt,
                       &tau_sh, &tau_el, &tau_wr);
    lm->tau_wr = tau_wr;

    /* 3. MIT 下发：位置/力矩口径不变；第 2 个参数是轨迹前馈速度 v_des
     *    （配 kd 起阻尼/超前），不是限速指令。
     *    关节↔电机换算是"加偏移、增益 1、方向 +1"，故速度数值 1:1 直接传；
     *    将来若给 joint_to_motor_x / motor_to_joint_x 加负号，这里必须同步取反。 */
    (void)AK_Motor_MIT(&lm->ak[0], joint_to_motor_1(q_cmd.q1), v_des->q1,
                       lm->cfg.kp, lm->cfg.kd, tau_sh);
    (void)AK_Motor_MIT(&lm->ak[1], joint_to_motor_2(q_cmd.q2), v_des->q2,
                       lm->cfg.kp, lm->cfg.kd, tau_el);
}

/* ---------------------------------------------------------------------------
 * 主循环唯一入口
 * ------------------------------------------------------------------------- */
int32_t LegMotion_Step(LegMotion *lm, float dt)
{
    LegJointAngles q_des;
    LegJointVec v_des, a_des;
    uint8_t st;

    if ((lm == 0) || (lm->ak == 0) || (lm->el05 == 0)) { return SER_ERR_PARAM; }
    if ((lm->ak_num < 2u) || (lm->el05_num < 1u))       { return SER_ERR_PARAM; }

    /* 0. 安全检查：任一电机报错 -> 保持当前位姿并放弃轨迹，等故障消失自动恢复 */
    if ((lm->ak[0].err_code != 0u) || (lm->ak[1].err_code != 0u)) {
        lm->fault = 1u;
        fault_hold(lm);
        lm->traj_state = lm->traj.state;
        lm->traj_tau   = (lm->traj.duration > 0.0f)
                       ? (lm->traj.elapsed / lm->traj.duration) : 0.0f;
        return SER_OK;
    }
    lm->fault = 0u;

    /* 1. 反馈 -> 当前关节角 */
    refresh_feedback(lm);

    /* 2. 正解：当前关节角 -> 当前末端位置（调试/观测用） */
    refresh_foot(lm);

    /* 3. 目标管理：目标变了才规划；正在跑就锁存，等本段 DONE 再规划
     *    （避免中途重启时"零初速五次多项式"从运动状态接手造成的速度跳变） */
    if ((fabsf(lm->target.x - lm->target_planned.x) > lm->cfg.target_eps) ||
        (fabsf(lm->target.z - lm->target_planned.z) > lm->cfg.target_eps))
    {
        if (lm->traj.state == JTRAJ_RUNNING) {
            lm->target_pending       = lm->target;   /* 锁存最新目标（后到覆盖先到） */
            lm->target_pending_valid = 1u;
        } else {
            lm->plan_result = plan_to(lm, lm->target.x, lm->target.z, lm->move_duration_s);
            lm->target_pending_valid = 0u;
        }
    }
    else if ((lm->target_pending_valid != 0u) && (lm->traj.state != JTRAJ_RUNNING))
    {
        lm->plan_result = plan_to(lm, lm->target_pending.x, lm->target_pending.z,
                                  lm->move_duration_s);
        lm->target_pending_valid = 0u;
    }

    /* 4. 轨迹推进（IDLE 时出参不被写入 -> 本拍退化为"保持当前位姿"） */
    st = JointTraj_Update(&lm->traj, dt, &q_des, &v_des, &a_des);
    if (st == JTRAJ_IDLE) {
        q_des.q1 = lm->fb.q1;
        q_des.q2 = lm->fb.q2;
        q_des.q3 = lm->fb.q3;
        v_des.q1 = 0.0f; v_des.q2 = 0.0f; v_des.q3 = 0.0f;
        a_des.q1 = 0.0f; a_des.q2 = 0.0f; a_des.q3 = 0.0f;
    }

    /* 5. 观测量快照（app 会映射到 Watch 全局变量） */
    lm->v_cmd      = v_des;
    lm->a_cmd      = a_des;
    lm->traj_state = lm->traj.state;
    lm->traj_tau   = (lm->traj.duration > 0.0f)
                   ? (lm->traj.elapsed / lm->traj.duration) : 0.0f;

    /* 6. 关节层：限速 -> 重力补偿 -> MIT 下发 */
    joint_output(lm, &q_des, &v_des, dt);

    return SER_OK;
}

/* ---------------------------------------------------------------------------
 * 腕部（EL05）下发：位置保持"末端水平"，力矩用重力补偿的腕分量
 * 与旧实现 EL05_MotionControl(joint_to_motor_3(π−(q1+q2)), 0, kp, kd, tau_wr) 一致
 * ------------------------------------------------------------------------- */
int32_t LegMotion_WristStep(LegMotion *lm)
{
    float pos_motor;

    if ((lm == 0) || (lm->el05 == 0)) { return SER_ERR_PARAM; }
    if (lm->el05_num < 1u)            { return SER_ERR_PARAM; }

    pos_motor = joint_to_motor_3(LEG_PI - (lm->fb.q1 + lm->fb.q2));

    return EL05_MotionControl(&lm->el05[0], pos_motor, 0.0f,
                              lm->cfg.wrist_kp, lm->cfg.wrist_kd, lm->tau_wr);
}