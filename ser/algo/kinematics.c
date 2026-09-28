/*****************************************************************************
 * kinematics.c —— 腿部运动学解算实现（FK / IK / 雅可比 / 关节-电机换算）
 *
 * 分层：纯算法，不依赖 HAL / FreeRTOS / bsp。
 *   注意：旧版在本文件里调 DWT_GetDeltaT() 取时间（为了给重力补偿做变化率限制），
 *   现在重力补偿已移到 motion_alg.c 且 dt 由调用方传入 —— 本文件不再有任何
 *   芯片时间源依赖，可直接在 PC 上编译单测。
 *
 * 已知约束（单腿工程）：连杆参数 g_link 与 IK 连续性状态 g_q1_prev/g_q2_prev
 *   是模块级状态，即"单腿单实例"。若将来做多腿，需把这三者收进一个句柄结构体。
 *****************************************************************************/
#include "kinematics.h"

#include <math.h>   /* cosf / sinf / atan2f / sqrtf / fabsf / fmaxf */

/* 连杆参数（全局，Init 时写入） */
static LegLinkParam g_link;

/* 上一拍解出的关节角，用于连续性择优 */
static float g_q1_prev = 0.0f;
static float g_q2_prev = 0.0f;

#define AK_PI        3.14159265358979f
#define AK_TWO_PI    6.28318530717959f

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

/* 初始化：保存连杆参数 */
int32_t Kinematics_Init(const LegLinkParam *param)
{
    if (param == 0) { return SER_ERR_PARAM; }
    g_link = *param;
    return SER_OK;
}

/* 正解 FK：由关节角度求足端位置 */
void Kinematics_Forward(const LegJointAngles *q, FootPosition *foot)
{
    if ((q == 0) || (foot == 0)) { return; }

    foot->x = g_link.L1 * cosf(q->q1) + g_link.L2 * cosf(q->q1 + q->q2)
            + g_link.L3 * cosf(q->q1 + q->q2 + q->q3);
    foot->z = g_link.L1 * sinf(q->q1) + g_link.L2 * sinf(q->q1 + q->q2)
            + g_link.L3 * sinf(q->q1 + q->q2 + q->q3);
}

/* 逆解 IK：XZ 平面二连杆（暂不考虑第三段 L3/q3），
 * elbow_up=0/1 指定膝弯向，其它值则两个构型都试并按限位+连续性择优 */
int32_t Kinematics_Inverse(const FootPosition *foot, LegJointAngles *q, int elbow_up)
{
    float x, z, D_sq, Lsum, Ldiff;
    float cos_q2, sin_q2_abs, sin_cand[2];
    float best_q1 = 0.0f, best_q2 = 0.0f, best_cost = 1e10f;
    int k;

    if ((foot == 0) || (q == 0)) { return SER_ERR_PARAM; }
    if ((g_link.L1 <= 0.0f) || (g_link.L2 <= 0.0f)) { return SER_ERR_PARAM; }

    x = foot->x;      /* 二连杆：直接取带符号 x，忽略 L3 与 y */
    z = foot->z;

    /* 可达性检查（距离平方） */
    D_sq  = x * x + z * z;
    Lsum  = g_link.L1 + g_link.L2;
    Ldiff = fabsf(g_link.L1 - g_link.L2);
    if ((D_sq > Lsum * Lsum + 1e-6f) || (D_sq < Ldiff * Ldiff - 1e-6f)) {
        return SER_ERR_LIMIT;
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
        q1  = phi - psi;

        q1 = wrap_pi(q1);
        q2 = wrap_pi(q2);

        penalty = (LegJointInLimit(q1, 1) && LegJointInLimit(q2, 2)) ? 0.0f : 10000.0f;
        cost = penalty + fabsf(ang_diff(q1, g_q1_prev))
              + 0.3f * fabsf(ang_diff(q2, g_q2_prev));

        if (cost < best_cost) {
            best_cost = cost;
            best_q1   = q1;
            best_q2   = q2;
        }
    }

    if (best_cost >= 9999.0f) { return SER_ERR_LIMIT; }   /* 无满足限位的解 */

    /* 最后保险：越限则钳回（best_cost<9999 已保证在限位内，正常不触发） */
    {
        LegJointAngles q_chk;

        q_chk.q1 = best_q1;
        q_chk.q2 = best_q2;
        q_chk.q3 = 0.0f;
        (void)LegJointsClamp(&q_chk);
        best_q1 = q_chk.q1;
        best_q2 = q_chk.q2;
    }

    g_q1_prev = best_q1;   /* 记录，供下一拍连续性择优 */
    g_q2_prev = best_q2;

    q->q1 = best_q1;
    q->q2 = best_q2;
    q->q3 = 0.0f;
    return SER_OK;
}

/* ================== 雅可比计算 ================== */
void jacobian_rz(float q1, float q2, float *J11, float *J12, float *J21, float *J22)
{
    float s1 = sinf(q1), c1 = cosf(q1);
    float s12 = sinf(q1 + q2), c12 = cosf(q1 + q2);

    if ((J11 == 0) || (J12 == 0) || (J21 == 0) || (J22 == 0)) { return; }

    /* 与 FK/IK 统一使用 Kinematics_Init 写入的 g_link 长度 */
    *J11 = -g_link.L1 * s1 - g_link.L2 * s12;
    *J12 = -g_link.L2 * s12;
    *J21 =  g_link.L1 * c1 + g_link.L2 * c12;
    *J22 =  g_link.L2 * c12;
}

/* ================== 关节角 <-> 电机角 零点/方向换算 ==================
 * 零点偏移（rad）：关节零位时电机角读数（实测标定值）。
 * 大臂零位原始读数 0.308 -> offset_down = 0.308；
 * 小臂零位原始读数 -1.90（取反后）-> offset_up = -1.90。 */
static float g_offset_down =  0.308f;
static float g_offset_up   = -1.90f;

/* 关节角 -> 电机角（q1 这条方向相反） */
float joint_to_motor_1(float q1) { return g_offset_down + q1; }

/* 关节角 -> 电机角（q2 这条方向相同） */
float joint_to_motor_2(float q2) { return q2 + g_offset_up; }

float joint_to_motor_3(float q3) { return -q3; }

/* 电机角 -> 关节角 */
float motor_to_joint_1(float m1) { return m1 - g_offset_down; }

/* 电机角 -> 关节角 */
float motor_to_joint_2(float m2) { return m2 - g_offset_up; }

float motor_to_joint_3(float m3) { return -m3; }