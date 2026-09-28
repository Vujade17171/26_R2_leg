/*****************************************************************************
 * motion_alg.c —— 关节限速器 + 重力补偿实现
 *
 * 分层：纯算法，无 HAL / FreeRTOS / bsp 依赖，无全局可变状态。
 *****************************************************************************/
#include "motion_alg.h"

#include <math.h>   /* cosf：重力补偿的水平力臂 */

/* 力矩变化率限制用的周期上限（卡顿/断点保护） */
#define GRAV_DT_MAX  0.05f

/* 默认重力补偿参数（与拆分前 kinematics.c 中的数值完全一致） */
const GravCfg GRAV_CFG_DEFAULT = {
    .m_arm   = 0.300f,   /* 大臂连杆质量 (kg) */
    .lc_arm  = 0.175f,   /* 大臂质心距肩 (m) */
    .m_elbow = 0.500f,   /* 肘关节等效质量 (kg)，位于 L1 末端 */
    .m_wrist = 0.450f,   /* 腕点等效质量 (kg)，位于 L2 末端 */
    .m_load  = 0.300f,   /* 腕部负载：空载吸盘组件 (kg) */
    .lc_load = 0.080f,   /* 负载质心距腕关节 (m) */
    .g       = 9.81f,
    .dir_sh  = 1.0f,     /* 肩方向 */
    .dir_el  = 1.0f,     /* 肘方向 */
    .dir_wr  = -1.0f,    /* 腕方向：EL05 电机反馈角 = -关节角 */
    .gear_sh = 1.5f,     /* 肩：当前工程数值 */
    .gear_el = 1.5f,     /* 肘：当前工程数值 */
    .gear_wr = 1.0f,     /* 腕：当前工程数值 */
    .max_torque_sh = 6.0f,   /* 肩力矩限幅 (N·m) */
    .max_torque_el = 5.5f,   /* 肘力矩限幅 (N·m) */
    .max_torque_wr = 5.0f,   /* 腕力矩限幅 (N·m) */
    .rate_limit    = 50.0f   /* 力矩变化率限制 (N·m/s) */
};

/* ==========================================================================
 * 关节限速器
 * ========================================================================== */

void JointRateLimit_Init(JointRateLimiter *rl, const LegJointAngles *q_now)
{
    if ((rl == 0) || (q_now == 0)) { return; }

    rl->q_cmd  = *q_now;
    rl->inited = 1u;
}

/* 单关节限速：每拍最多变化 max_rate×dt，落后不足一步则直接吸附到目标 */
static float rate_step(float des, float cmd, float max_rate, float dt, uint8_t inited)
{
    float step, err;

    if ((inited == 0u) || (dt <= 0.0f)) { return des; }  /* 首拍/异常周期：不限速 */

    step = max_rate * dt;
    err  = des - cmd;

    if (err >  step) { return cmd + step; }
    if (err < -step) { return cmd - step; }
    return des;   /* 到位收尾：吸附目标，避免浮点残差永远到不了位 */
}

void JointRateLimit_Apply(JointRateLimiter *rl, const LegJointAngles *q_des,
                          float dt, LegJointAngles *q_cmd)
{
    if ((rl == 0) || (q_des == 0) || (q_cmd == 0)) { return; }

    rl->q_cmd.q1 = rate_step(q_des->q1, rl->q_cmd.q1, JOINT_RATE_Q1_MAX, dt, rl->inited);
    rl->q_cmd.q2 = rate_step(q_des->q2, rl->q_cmd.q2, JOINT_RATE_Q2_MAX, dt, rl->inited);
    rl->q_cmd.q3 = rate_step(q_des->q3, rl->q_cmd.q3, JOINT_RATE_Q3_MAX, dt, rl->inited);

    rl->inited = 1u;
    *q_cmd = rl->q_cmd;
}

/* ==========================================================================
 * 重力补偿
 * ========================================================================== */

void GravityComp_Init(GravityComp *gc, const GravCfg *cfg)
{
    if (gc == 0) { return; }

    gc->cfg         = (cfg != 0) ? *cfg : GRAV_CFG_DEFAULT;
    gc->last_tau_sh = 0.0f;
    gc->last_tau_el = 0.0f;
    gc->last_tau_wr = 0.0f;
    gc->first_run   = 1u;
}

/* 力矩限幅 + 变化率限制（平滑，避免突变） */
static float limit_torque(float target, float last, float max_abs,
                          float rate_limit, float dt)
{
    float step;

    if (target >  max_abs) { target =  max_abs; }
    if (target < -max_abs) { target = -max_abs; }
    if (dt <= 0.0f) { return target; }   /* 首拍或异常周期：只限幅不限速 */

    step = rate_limit * dt;
    if (target - last >  step) { target = last + step; }
    if (target - last < -step) { target = last - step; }
    return target;
}

void GravityComp_Update(GravityComp *gc, const LegLinkParam *link,
                        float q1, float q2, float q3, float dt,
                        float *tau_sh, float *tau_el, float *tau_wr)
{
    float L1, L2;
    float c1, c12, c123;
    float o_sh, o_el, o_wr;
    float t1 = 0.0f, t2 = 0.0f, t3 = 0.0f;

    if ((gc == 0) || (link == 0)) { return; }

    L1 = link->L1;
    L2 = link->L2;

    c1   = cosf(q1);
    c12  = cosf(q1 + q2);
    c123 = cosf(q1 + q2 + q3);

    /* ---- 肩关节：所有质量对肩的水平力臂 ---- */
    t1 += gc->cfg.m_arm   * gc->cfg.g * (gc->cfg.lc_arm * c1);
    t1 += gc->cfg.m_elbow * gc->cfg.g * (L1 * c1);
    t1 += gc->cfg.m_wrist * gc->cfg.g * (L1 * c1 + L2 * c12);
    t1 += gc->cfg.m_load  * gc->cfg.g * (L1 * c1 + L2 * c12 + gc->cfg.lc_load * c123);

    /* ---- 肘关节：肘之后（远端）的质量 ---- */
    t2 += gc->cfg.m_wrist * gc->cfg.g * (L2 * c12);
    t2 += gc->cfg.m_load  * gc->cfg.g * (L2 * c12 + gc->cfg.lc_load * c123);

    /* ---- 腕关节：腕之后（远端）的质量 ---- */
    t3 += gc->cfg.m_load  * gc->cfg.g * (gc->cfg.lc_load * c123);

    /* 折算：除以 gear（用当前工程数值），得到电机侧力矩 */
    o_sh = (gc->cfg.gear_sh > 0.0f) ? (gc->cfg.dir_sh * t1 / gc->cfg.gear_sh) : 0.0f;
    o_el = (gc->cfg.gear_el > 0.0f) ? (gc->cfg.dir_el * t2 / gc->cfg.gear_el) : 0.0f;
    o_wr = (gc->cfg.gear_wr > 0.0f) ? (gc->cfg.dir_wr * t3 / gc->cfg.gear_wr) : 0.0f;

    /* 首拍：直接取目标值作初值（只限幅不限速），此后 dt 由调用方实测传入 */
    if (gc->first_run != 0u) { gc->first_run = 0u; dt = 0.0f; }
    if (dt > GRAV_DT_MAX)    { dt = GRAV_DT_MAX; }

    /* 限幅 + 变化率平滑 */
    gc->last_tau_sh = limit_torque(o_sh, gc->last_tau_sh, gc->cfg.max_torque_sh,
                                   gc->cfg.rate_limit, dt);
    gc->last_tau_el = limit_torque(o_el, gc->last_tau_el, gc->cfg.max_torque_el,
                                   gc->cfg.rate_limit, dt);
    gc->last_tau_wr = limit_torque(o_wr, gc->last_tau_wr, gc->cfg.max_torque_wr,
                                   gc->cfg.rate_limit, dt);

    if (tau_sh != 0) { *tau_sh = gc->last_tau_sh; }
    if (tau_el != 0) { *tau_el = gc->last_tau_el; }
    if (tau_wr != 0) { *tau_wr = gc->last_tau_wr; }
}