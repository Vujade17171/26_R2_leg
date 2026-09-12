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

/* ================= 重力补偿参数（按实测调整） =================
 * 补偿力矩 = dir × (Σ 质量×g×水平力臂) / gear，再做限幅 + 变化率限制
 *   - gear：减速比/补偿强度，用当前工程使用的数值；
 *           实测补偿过大就把对应 gear 调大，不足就调小。
 *   - dir ：关节->电机 方向，同号 +1、反向 -1。
 *   - lc_*：各质量质心到对应关节的距离。
 *   - max_torque_*：力矩限幅 (N·m，电机侧)。
 *   - rate_limit  ：力矩变化率限制 (N·m/s)，周期由 DWT 实测。 */
static const grav_cfg_t grav = {
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
    .gear_el = 1.8f,     /* 肘：当前工程数值 */
    .gear_wr = 1.0f,     /* 腕：当前工程数值 */
    .max_torque_sh = 6.0f,   /* 肩力矩限幅 (N·m) */
    .max_torque_el = 5.5f,   /* 肘力矩限幅 (N·m) */
    .max_torque_wr = 5.0f,   /* 腕力矩限幅 (N·m) */
    .rate_limit    = 50.0f   /* 力矩变化率限制 (N·m/s) */
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

/* ================= 关节空间限速参数（rad/s，按实测调整） =================
 * 摆动速率上限：每拍指令角最多变化 限速值 × dt。
 *   肩/肘 2.0 rad/s ≈ 115°/s；腕 1.0 rad/s ≈ 57°/s。
 * 只限制"期望轨迹"的推进速度：kp≠0 时电机实际摆速≈该上限；
 * 但 kp=0（纯力矩测试）或外力推动时实际速度不受此约束。 */
#define JOINT_RATE_Q1_MAX   3.0f
#define JOINT_RATE_Q2_MAX   3.0f
#define JOINT_RATE_Q3_MAX   1.0f

/* 限速器状态：上一拍限速后的指令关节角 */
static LegJointAngles g_q_cmd = { 0.0f, 0.0f, 0.0f };
static uint8_t g_q_cmd_inited = 0u;

/* 复位限速器：以当前实际关节角作为指令起点（首拍不跳变） */
void joint_rate_limit_reset(const LegJointAngles *q_now)
{
    if (q_now == 0) { return; }

    g_q_cmd = *q_now;
    g_q_cmd_inited = 1u;
}

/* 单关节限速：每拍最多变化 max_rate×dt，落后不足一步则直接吸附到目标 */
static float rate_step(float des, float cmd, float max_rate, float dt)
{
    float step, err;

    if ((g_q_cmd_inited == 0u) || (dt <= 0.0f)) { return des; }  /* 首拍/异常周期：不限速 */

    step = max_rate * dt;
    err  = des - cmd;

    if (err >  step) { return cmd + step; }
    if (err < -step) { return cmd - step; }
    return des;   /* 到位收尾：吸附目标，避免浮点残差永远到不了位 */
}

/* 期望关节角 -> 每拍增量受限的指令关节角 */
void joint_rate_limit(const LegJointAngles *q_des, float dt, LegJointAngles *q_cmd)
{
    if ((q_des == 0) || (q_cmd == 0)) { return; }

    g_q_cmd.q1 = rate_step(q_des->q1, g_q_cmd.q1, JOINT_RATE_Q1_MAX, dt);
    g_q_cmd.q2 = rate_step(q_des->q2, g_q_cmd.q2, JOINT_RATE_Q2_MAX, dt);
    g_q_cmd.q3 = rate_step(q_des->q3, g_q_cmd.q3, JOINT_RATE_Q3_MAX, dt);

    g_q_cmd_inited = 1u;
    *q_cmd = g_q_cmd;
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

/* ================== 重力补偿（简单杠杆原理） ==================
 * 每个关节的重力矩 = Σ (质量 × g × 该关节到该质量质心的水平距离)
 * 水平距离 = 各段长度 × cos(该段绝对角)，即力臂的水平投影。
 *
 * 各质量的位置（从肩算起，q1/q2/q3 为各段绝对角累加）：
 *   大臂杆  m_arm   : 水平距离 = lc_arm·c1
 *   肘部质量 m_elbow : 水平距离 = L1·c1
 *   腕点质量 m_wrist : 水平距离 = L1·c1 + L2·c12
 *   腕部负载 m_load  : 水平距离 = L1·c1 + L2·c12 + lc_load·c123
 * 其中 c1 = cos(q1)、c12 = cos(q1+q2)、c123 = cos(q1+q2+q3)。
 *
 * 输出 = dir × 力矩 / gear，再经【限幅 + 变化率限制】平滑下发。
 * 变化率限制用 DWT 实测周期（需先调用 DWT_Init），首拍不限速。 */
void get_gravity_comp_torque(float q1, float q2, float q3,
                             float *tau_sh, float *tau_el, float *tau_wr)
{
    static float last_tau_sh = 0.0f;
    static float last_tau_el = 0.0f;
    static float last_tau_wr = 0.0f;
    static uint32_t dwt_cnt_last = 0;   /* DWT 计数缓存，用于算实际周期 */
    static uint8_t  first_run = 1u;     /* 首拍标记：不做限速，直接取目标值 */

    const float g = grav.g;
    const float L1 = g_link.L1;
    const float L2 = g_link.L2;
    float c1, c12, c123;
    float dt, o_sh, o_el, o_wr;
    float t1 = 0.0f, t2 = 0.0f, t3 = 0.0f;

    c1   = cosf(q1);
    c12  = cosf(q1 + q2);
    c123 = cosf(q1 + q2 + q3);

    /* ---- 肩关节：所有质量对肩的水平力臂 ---- */
    t1 += grav.m_arm   * g * (grav.lc_arm * c1);
    t1 += grav.m_elbow * g * (L1 * c1);
    t1 += grav.m_wrist * g * (L1 * c1 + L2 * c12);
    t1 += grav.m_load  * g * (L1 * c1 + L2 * c12 + grav.lc_load * c123);

    /* ---- 肘关节：肘之后（远端）的质量 ---- */
    t2 += grav.m_wrist * g * (L2 * c12);
    t2 += grav.m_load  * g * (L2 * c12 + grav.lc_load * c123);

    /* ---- 腕关节：腕之后（远端）的质量 ---- */
    t3 += grav.m_load  * g * (grav.lc_load * c123);

    /* 折算：除以 gear（用当前工程数值），得到电机侧力矩 */
    o_sh = (grav.gear_sh > 0.0f) ? (grav.dir_sh * t1 / grav.gear_sh) : 0.0f;
    o_el = (grav.gear_el > 0.0f) ? (grav.dir_el * t2 / grav.gear_el) : 0.0f;
    o_wr = (grav.gear_wr > 0.0f) ? (grav.dir_wr * t3 / grav.gear_wr) : 0.0f;

    /* 变化率限制用的周期：由 DWT 实测 */
    dt = DWT_GetDeltaT(&dwt_cnt_last);
    if (first_run != 0u) { first_run = 0u; dt = 0.0f; }  /* 首拍：直接取目标值作初值 */
    if (dt > 0.05f)      { dt = 0.05f; }                 /* 卡顿/异常大周期上限 50ms */

    /* 限幅 + 变化率平滑 */
    last_tau_sh = limit_torque(o_sh, last_tau_sh, grav.max_torque_sh, grav.rate_limit, dt);
    last_tau_el = limit_torque(o_el, last_tau_el, grav.max_torque_el, grav.rate_limit, dt);
    last_tau_wr = limit_torque(o_wr, last_tau_wr, grav.max_torque_wr, grav.rate_limit, dt);

    if (tau_sh != 0) { *tau_sh = last_tau_sh; }
    if (tau_el != 0) { *tau_el = last_tau_el; }
    if (tau_wr != 0) { *tau_wr = last_tau_wr; }
}