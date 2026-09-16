/*****************************************************************************
 * joint_traj.c —— 关节空间五次多项式轨迹实现
 *   s(τ) = 10τ³ - 15τ⁴ + 6τ⁵，τ = t/T，详见 joint_traj.h
 *
 * 与调用方的分工：
 *   调用方（leg_task）负责：IK 求终点、传实际反馈角当起点、限速、重力补偿、CAN 下发；
 *   本模块只负责：时间推进 + 五次多项式求值，输出本拍的位置/速度/加速度。
 *****************************************************************************/
#include "joint_traj.h"
#include <math.h>

/* 峰值系数（用于估算/反推时长；s' 峰值 1.875，s'' 峰值 5.773503） */
#define S_PEAK_DOT    1.875f
#define S_PEAK_DDOT   5.773503f

/* dt 合法性：异常大周期（卡顿/断点）钳到 50ms，习惯与 kinematics.c 重力补偿一致 */
#define JTRAJ_DT_MAX  0.05f

/* 最短规划时长：至少 2 个控制周期，避免 T→0 时 1/T 数值爆炸 */
#define JTRAJ_T_MIN   0.002f

/* ---------------------------------------------------------------------------
 * 规划：写入起点/终点/时长，状态置 RUNNING
 * 终点超限直接拒绝：钳位只改位置不改 Δ，会让终点前馈不为 0，持续顶限位
 * ------------------------------------------------------------------------- */
uint8_t JointTraj_Plan(JointTraj *t, const LegJointAngles *q_now,
                       const LegJointAngles *q_goal, float duration_s)
{
    if ((t == 0) || (q_now == 0) || (q_goal == 0)) { return JTRAJ_ERR_PARAM; }
    if (!(duration_s >= JTRAJ_T_MIN)) {              /* 写 !(x>=y) 同时挡住 NaN */
        t->state = JTRAJ_IDLE;
        return JTRAJ_ERR_PARAM;
    }

    if ((Kinematics_JointInLimit(q_goal->q1, 1) == 0) ||
        (Kinematics_JointInLimit(q_goal->q2, 2) == 0)) {
        t->state = JTRAJ_IDLE;
        return JTRAJ_ERR_LIMIT;
    }

    t->q_start[0] = q_now->q1;
    t->q_start[1] = q_now->q2;
    t->q_start[2] = q_now->q3;

    t->q_end[0]   = q_goal->q1;
    t->q_end[1]   = q_goal->q2;
    t->q_end[2]   = q_goal->q3;

    t->duration = duration_s;
    t->elapsed  = 0.0f;
    t->state    = JTRAJ_RUNNING;
    return JTRAJ_OK;
}

/* ---------------------------------------------------------------------------
 * 推进一拍：位置/速度/加速度一次算全
 *   - elapsed 累加 dt；τ≥1 时钳到 1 并转 DONE（DONE 后继续按 τ=1 输出 → 稳定保持）
 *   - τ=1 处 s=1、s'=0、s''=0 是代数精确的（10-15+6=0 / 30-60+30=0 / 60-180+120=0），
 *     不存在浮点残余 → 终点不会残留速度/加速度指令
 *   - dt<=0：不推进时间，按当前进度输出（不做除零/回退）
 * ------------------------------------------------------------------------- */
uint8_t JointTraj_Update(JointTraj *t, float dt,
                         LegJointAngles *q_des,
                         JointTrajVec *v_des, JointTrajVec *a_des)
{
    float tau, tau2, tau3, inv_T, inv_T2;
    float s, s_dot, s_ddot;
    float d0, d1, d2;

    if (t == 0)                 { return JTRAJ_IDLE; }
    if (t->state == JTRAJ_IDLE) { return JTRAJ_IDLE; }   /* 出参不写 */
    if ((q_des == 0) || (v_des == 0) || (a_des == 0)) { return JTRAJ_IDLE; }
    if (t->duration <= 0.0f)    { t->state = JTRAJ_IDLE; return JTRAJ_IDLE; }

    if (dt > 0.0f) {                     /* dt<=0：不推进，原样输出上一拍位置 */
        if (dt > JTRAJ_DT_MAX) { dt = JTRAJ_DT_MAX; }
        t->elapsed += dt;
    }
    if (t->elapsed < 0.0f) { t->elapsed = 0.0f; }             /* 浮点异常防护 */

    tau = t->elapsed / t->duration;
    if (tau >= 1.0f) {
        tau = 1.0f;
        t->state = JTRAJ_DONE;
    }

    tau2   = tau * tau;
    tau3   = tau2 * tau;
    inv_T  = 1.0f / t->duration;
    inv_T2 = inv_T * inv_T;

    /* s(τ)    = 10τ³ - 15τ⁴ + 6τ⁵          → τ³·(10 + τ·(-15 + 6τ))         */
    s      = tau3 * (10.0f + tau * (-15.0f + 6.0f * tau));
    /* ds/dt   = (30τ² - 60τ³ + 30τ⁴)/T      → τ²·(30 + τ·(-60 + 30τ))/T     */
    s_dot  = tau2 * inv_T * (30.0f + tau * (-60.0f + 30.0f * tau));
    /* d²s/dt² = (60τ - 180τ² + 120τ³)/T²    → τ·(60 + τ·(-180 + 120τ))/T²   */
    s_ddot = tau * inv_T2 * (60.0f + tau * (-180.0f + 120.0f * tau));

    d0 = t->q_end[0] - t->q_start[0];
    d1 = t->q_end[1] - t->q_start[1];
    d2 = t->q_end[2] - t->q_start[2];   /* 腕通道：调用方传 Δ3=0 → 恒为 0 */

    q_des->q1 = t->q_start[0] + d0 * s;
    q_des->q2 = t->q_start[1] + d1 * s;
    q_des->q3 = t->q_start[2] + d2 * s;

    v_des->q1 = d0 * s_dot;
    v_des->q2 = d1 * s_dot;
    v_des->q3 = d2 * s_dot;

    a_des->q1 = d0 * s_ddot;
    a_des->q2 = d1 * s_ddot;
    a_des->q3 = d2 * s_ddot;

    return t->state;   /* JTRAJ_RUNNING 或 JTRAJ_DONE */
}

/* 放弃轨迹 */
void JointTraj_Abort(JointTraj *t)
{
    if (t == 0) { return; }
    t->state   = JTRAJ_IDLE;
    t->elapsed = 0.0f;
}

/* 峰值速度估算：|Δ|·1.875 / T */
float JointTraj_PeakVel(float delta, float duration)
{
    if (!(duration > 0.0f)) { return 0.0f; }
    return S_PEAK_DOT * fabsf(delta) / duration;
}

/* 反推"限速器不会削"的最短时长：取各关节需求与 min_T 的最大值 */
float JointTraj_SuggestDuration(float d1, float d2, float min_T)
{
    float T;
    float t1, t2;

    T = (min_T > JTRAJ_T_MIN) ? min_T : JTRAJ_T_MIN;

    t1 = (JOINT_RATE_Q1_MAX > 0.0f) ? (S_PEAK_DOT * fabsf(d1) / JOINT_RATE_Q1_MAX) : 0.0f;
    t2 = (JOINT_RATE_Q2_MAX > 0.0f) ? (S_PEAK_DOT * fabsf(d2) / JOINT_RATE_Q2_MAX) : 0.0f;

    if (t1 > T) { T = t1; }
    if (t2 > T) { T = t2; }

    return T;
}
