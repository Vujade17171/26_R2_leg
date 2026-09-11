/*****************************************************************************
 * kinematics.c —— 腿部运动学解算实现（FK / IK）
 *
 * 本文件为骨架：连杆参数、正解公式、逆解公式待按实际机械构型填入。
 * 使用前务必确认关节定义、正方向、连杆长度与实物一致。
 *****************************************************************************/
#include "kinematics.h"
#include <math.h>
#include "bsp_dwt.h"   /* DWT_GetDeltaT：实测调用周期，用于力矩变化率限制 */

/* 连杆参数（全局，Init 时写入） */
static LegLinkParam g_link;

/* 上一拍解出的关节角，用于连续性择优 */
static float g_q1_prev = 0.0f;
static float g_q2_prev = 0.0f;

/* 关节限位（rad），按实物机械限位调整 */
#define JOINT1_MIN  (-1.25663704f)
#define JOINT1_MAX  ( 3.1415926f)
#define JOINT2_MIN  (-2.82743334f)
#define JOINT2_MAX  ( 2.82743334f)

#define AK_PI        3.14159265358979f
#define AK_TWO_PI    6.28318530717959f

static const grav_comp_cfg_t grav = {
    .m_elbow = 0.500f,
    .m_wrist = 0.450f,
    .g = 9.81f,
    .shoulder_gear = 7.5f,
    .elbow_gear = 7.5f
};

static const dyn_ff_cfg_t ff = {
    .m_link1 = 0.300f,
    .lc_link1 = 0.175f,
    .kd_sh = 0.12f,
    .kd_el = 0.10f,
    .max_torque_sh = 6.0f,
    .max_torque_el = 5.5f,
    .rate_limit = 50.0f,
};

/* 角度归一化到 [-PI, PI] */
static float wrap_pi(float a)
{
    while (a >  AK_PI) { a -= AK_TWO_PI; }
    while (a < -AK_PI) { a += AK_TWO_PI; }
    return a;
}

/* 两角差（归一化到 [-PI, PI]） */
static float ang_diff(float a, float b)
{
    return wrap_pi(a - b);
}

/* 关节角是否在限位内（1=q1，2=q2） */
static int in_limit(float a, int joint)
{
    if (joint == 1) { return (a >= JOINT1_MIN) && (a <= JOINT1_MAX); }
    return (a >= JOINT2_MIN) && (a <= JOINT2_MAX);
}

/* 把关节角钳位到限位内 */
static void clamp_to_joint_limit(float *q1, float *q2)
{
    if (*q1 > JOINT1_MAX) { *q1 = JOINT1_MAX; }
    if (*q1 < JOINT1_MIN) { *q1 = JOINT1_MIN; }
    if (*q2 > JOINT2_MAX) { *q2 = JOINT2_MAX; }
    if (*q2 < JOINT2_MIN) { *q2 = JOINT2_MIN; }
}

/* 初始化：保存连杆参数 */
void Kinematics_Init(const LegLinkParam *param)
{
    if (param == 0) { return; }
    g_link = *param;
}

/* 正解 FK：由关节角度求足端位置 */
void Kinematics_Forward(const LegJointAngles *q, FootPosition *foot)
{
    if ((q == 0) || (foot == 0)) { return; }

    
    foot->x = g_link.L1 * cosf(q->q1) + g_link.L2 * cosf(q->q1 + q->q2) + g_link.L3 * cosf(q->q1 + q->q2 + q->q3); 
    foot->z = g_link.L1 * sinf(q->q1) + g_link.L2 * sinf(q->q1 + q->q2) + g_link.L3 * sinf(q->q1 + q->q2 + q->q3);

}

/* 逆解 IK：XZ 平面二连杆（暂不考虑第三段 L3/q3），
 * elbow_up=0/1 指定膝弯向，其它值则两个构型都试并按限位+连续性择优 */
int8_t Kinematics_Inverse(const FootPosition *foot, LegJointAngles *q, int elbow_up)
{
    float x, z, D_sq, Lsum, Ldiff;
    float cos_q2, sin_q2_abs, sin_cand[2];
    float best_q1 = 0.0f, best_q2 = 0.0f, best_cost = 1e10f;
    int k;

    if ((foot == 0) || (q == 0)) { return -1; }
    if ((g_link.L1 <= 0.0f) || (g_link.L2 <= 0.0f)) { return -1; }

    x = foot->x;      /* 二连杆：直接取带符号 x，忽略 L3 与 y */
    z = foot->z;

    /* 可达性检查（距离平方） */
    D_sq  = x * x + z * z;
    Lsum  = g_link.L1 + g_link.L2;
    Ldiff = fabsf(g_link.L1 - g_link.L2);
    if ((D_sq > Lsum * Lsum + 1e-6f) || (D_sq < Ldiff * Ldiff - 1e-6f)) {
        return -1;
    }

    cos_q2 = (D_sq - g_link.L1 * g_link.L1 - g_link.L2 * g_link.L2)
             / (2.0f * g_link.L1 * g_link.L2);
    if (cos_q2 > 1.0f)  { cos_q2 = 1.0f; }
    if (cos_q2 < -1.0f) { cos_q2 = -1.0f; }
    sin_q2_abs = sqrtf(fmaxf(0.0f, 1.0f - cos_q2 * cos_q2));

    /* 由 elbow_up 决定膝关节正弦符号（构型） */
    if (elbow_up == 0) {
        sin_cand[0] = -sin_q2_abs; sin_cand[1] = -sin_q2_abs;
    } else if (elbow_up == 1) {
        sin_cand[0] = +sin_q2_abs; sin_cand[1] = +sin_q2_abs;
    } else {
        sin_cand[0] = +sin_q2_abs; sin_cand[1] = -sin_q2_abs;
    }

    for (k = 0; k < 2; k++) {
        float s2, q2, q1, phi, psi, penalty, cost;
        s2 = sin_cand[k];
        q2 = atan2f(s2, cos_q2);

        phi = atan2f(z, x);                          /* 足端方向（带符号 x） */
        psi = atan2f(g_link.L2 * s2, g_link.L1 + g_link.L2 * cos_q2);
        q1 = phi - psi;

        q1 = wrap_pi(q1);
        q2 = wrap_pi(q2);

        penalty = (in_limit(q1, 1) && in_limit(q2, 2)) ? 0.0f : 10000.0f;
        cost = penalty + fabsf(ang_diff(q1, g_q1_prev))
              + 0.3f * fabsf(ang_diff(q2, g_q2_prev));

        if (cost < best_cost) {
            best_cost = cost;
            best_q1 = q1;
            best_q2 = q2;
        }
    }

    if (best_cost >= 9999.0f) { return -1; }   /* 无满足限位的解 */

    clamp_to_joint_limit(&best_q1, &best_q2);

    g_q1_prev = best_q1;   /* 记录，供下一拍连续性择优 */
    g_q2_prev = best_q2;

    q->q1 = best_q1;
    q->q2 = best_q2;
    q->q3 = 0.0f;          
    return 0;
}

// ================== 雅可比计算 ==================
void jacobian_rz(float q1, float q2, float* J11, float* J12, float* J21, float* J22) {
    float s1 = sinf(q1), c1 = cosf(q1);
    float s12 = sinf(q1 + q2), c12 = cosf(q1 + q2);

    /* 与 FK/IK/重力补偿统一使用 Kinematics_Init 写入的 g_link 长度 */
    *J11 = -g_link.L1*s1 - g_link.L2*s12;
    *J12 = -g_link.L2*s12;
    *J21 =  g_link.L1*c1 + g_link.L2*c12;
    *J22 =  g_link.L2*c12;
}

/* ================== 关节角 <-> 电机角 零点/方向换算 ================== */

/* 零点偏移（rad）：关节零位时电机角 m 满足 q = offset - m(大臂) / q = m + offset(小臂)
 * 实测：大臂零位原始读数 0.308 → offset_down = 0.308；
 *       小臂零位原始读数 0.756（未取反）→ offset_up = -0.756 */
static float g_offset_down =  0.308f;
static float g_offset_up   =  -1.90f;
/* 设置两个关节的零点偏移（rad） */


/* 关节角 -> 电机角（q1 这条方向相反） */
float joint_to_motor_1(float q1) { return g_offset_down + q1; }

/* 关节角 -> 电机角（q2 这条方向相同） */
float joint_to_motor_2(float q2) { return q2 + g_offset_up; }

/* 电机角 -> 关节角 */
float motor_to_joint_1(float m1) { return m1 - g_offset_down ; }

/* 电机角 -> 关节角 */
float motor_to_joint_2(float m2) { return m2 - g_offset_up; }

/* ================== 重力补偿 ================== */

/* 计算两连杆在重力下的肩/肘关节力矩，并折算成电机侧力矩。
 * 思路：势能 U = m_el·g·L1·sin(q1) + m_wr·g·[L1·sin(q1)+L2·sin(q1+q2)]，
 *       关节重力矩 = ∂U/∂q，再除以减速比得电机侧力矩。 */
void get_gravity_torque_motor_cfg(const grav_comp_cfg_t *cfg,
                                  float q1, float q2,
                                  float *tau_sh, float *tau_el)
{
    float c1, c12, T1, T2;

    if (cfg == 0) { return; }

    c1  = cosf(q1);
    c12 = cosf(q1 + q2);

    T1 = cfg->m_elbow * cfg->g * g_link.L1 * c1
       + cfg->m_wrist * cfg->g * (g_link.L1 * c1 + g_link.L2 * c12);
    T2 = cfg->m_wrist * cfg->g * g_link.L2 * c12;

    if (tau_sh != 0) {
        *tau_sh = (cfg->shoulder_gear > 0.0f) ? (T1 / cfg->shoulder_gear) : 0.0f;
    }
    if (tau_el != 0) {
        *tau_el = (cfg->elbow_gear > 0.0f) ? (T2 / cfg->elbow_gear) : 0.0f;
    }
}

/* 力矩限幅 + 变化率限制（平滑，避免突变） */
static float limit_torque(float target, float last, float max_abs,
                          float rate_limit, float dt)
{
    float step;

    if (target >  max_abs) { target =  max_abs; }
    if (target < -max_abs) { target = -max_abs; }
    if (dt <= 0.0f) { return target; }

    step = rate_limit * dt;
    if (target - last >  step) { target = last + step; }
    if (target - last < -step) { target = last - step; }
    return target;
}

/* 动力学前馈力矩：重力 + 连杆自重 + 粘性阻尼，再限幅/限速平滑 */
void get_dynamic_feedforward_torque(float q1, float q2,
                                    float dq1, float dq2,
                                    float *tau_sh_ff, float *tau_el_ff)
{
    static float last_tau_sh = 0.0f;
    static float last_tau_el = 0.0f;
    static uint32_t dwt_cnt_last = 0;   /* DWT 计数缓存，用于算实际周期 */
    static uint8_t  first_run = 1u;     /* 首拍标记：丢弃异常大的 dt */

    float dt;
    float tg_sh = 0.0f, tg_el = 0.0f;
    float tv_sh, tv_el, T1_link;
    float c1;

    /* ff 与 grav 都是本文件内的常量结构体（非指针），无需判空 */

    /* 变化率限制用的周期：由 DWT 实测（需先 DWT_Init） */
    dt = DWT_GetDeltaT(&dwt_cnt_last);
    if (first_run != 0u) { first_run = 0u; dt = 0.0f; }  /* 首拍：直接取目标值作初值，不做限速 */
    if (dt > 0.05f)      { dt = 0.05f; }                 /* 卡顿/异常大周期上限 50ms */

    /* 1. 空载机械臂重力补偿（电机侧力矩） */
    get_gravity_torque_motor_cfg(&grav, q1, q2, &tg_sh, &tg_el);

    /* 2. 大臂连杆自身重力补偿（其质心近似在大臂中部） */
    c1 = cosf(q1);
    T1_link = ff.m_link1 * grav.g * ff.lc_link1 * c1;
    if (grav.shoulder_gear > 0.0f) {
        tg_sh += T1_link / grav.shoulder_gear;
    }

    /* 3. 速度粘性阻尼补偿 */
    tv_sh = ff.kd_sh * dq1;
    tv_el = ff.kd_el * dq2;

    /* 4. 限幅与变化率平滑（dt 为 DWT 实测周期） */
    last_tau_sh = limit_torque(tg_sh + tv_sh, last_tau_sh,
                               ff.max_torque_sh, ff.rate_limit, dt);
    last_tau_el = limit_torque(tg_el + tv_el, last_tau_el,
                               ff.max_torque_el, ff.rate_limit, dt);

    if (tau_sh_ff != 0) { *tau_sh_ff = last_tau_sh; }
    if (tau_el_ff != 0) { *tau_el_ff = last_tau_el; }
}