/**
  ******************************************************************************
  * @file    arm_kinematics.c
  * @brief   Planar 2R arm kinematics
  ******************************************************************************
  */
#include "arm_kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* motor zero offsets (reference project) */
#define OFFSET_DOWN   3.141593f    /* 肩关节电机偏移*/
#define OFFSET_UP    -2.6511548f   /* 肘关节电机偏移*/


//三个关节电机限位
arm_joint_limit_t g_arm_joint_limit =
{
    .q_min = { -M_PI,        -M_PI*0.4f,  -M_PI*0.9f },
    .q_max = {  M_PI,         M_PI,        M_PI*0.9f  },
};

//功能：限幅函数。把 v 限制在`lo`下限～`hi`上限之间。
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

//角度卷绕函数：**把任意弧度角，映射到 [-π , +π] 区间**
static inline float wrap_pi(float a)
{
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

/* joint -> motor */
float arm_joint_to_motor_1(float q1) { return OFFSET_DOWN - q1; }
float arm_joint_to_motor_2(float q2) { return q2 - OFFSET_UP;   }

/* motor -> joint */
float arm_motor_to_joint_1(float m1) { return OFFSET_DOWN - m1; }
float arm_motor_to_joint_2(float m2) { return m2 + OFFSET_UP;   }


//检查电机是否在限位内
int arm_in_limit(float q, int idx)
{
    if (idx < 0 || idx > 2) { return 0; }
    return (q >= g_arm_joint_limit.q_min[idx] &&
            q <= g_arm_joint_limit.q_max[idx]);
}

//检查三个关节角度是否都在限位内
void arm_clamp_joints(float *q0, float *q1, float *q2)
{
    if (q0) *q0 = clampf(*q0, g_arm_joint_limit.q_min[0], g_arm_joint_limit.q_max[0]);
    if (q1) *q1 = clampf(*q1, g_arm_joint_limit.q_min[1], g_arm_joint_limit.q_max[1]);
    if (q2) *q2 = clampf(*q2, g_arm_joint_limit.q_min[2], g_arm_joint_limit.q_max[2]);
}


//判断目标坐标 (x,z)，2R 平面臂能不能到达。
int arm_reachable(float x, float z)
{
    float D2 = x * x + z * z;
    float Lsum  = ARM_L1 + ARM_L2;
    float Ldiff = fabsf(ARM_L1 - ARM_L2);
    if (D2 > Lsum * Lsum || D2 < Ldiff * Ldiff) { return -1; }
    return 0;
}


//输入坐标，输出角度
int arm_inverse(float x, float z, float yaw,
                float *q0, float *q1, float *q2)
{
    float D2, cosq2, sinq2, q2v, phi, psi, q1v;

    if (!q0 || !q1 || !q2) { return -1; }
    if (arm_reachable(x, z) != 0) { return -1; }

    D2 = x * x + z * z;
    cosq2 = (D2 - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) / (2.0f * ARM_L1 * ARM_L2);
    if (cosq2 >  1.0f) cosq2 =  1.0f;
    if (cosq2 < -1.0f) cosq2 = -1.0f;
    sinq2 = sqrtf(1.0f - cosq2 * cosq2);
    q2v   = atan2f(sinq2, cosq2);            /* elbow-up branch */

    phi = atan2f(z, x);
    psi = atan2f(ARM_L2 * sinq2, ARM_L1 + ARM_L2 * cosq2);
    q1v = phi - psi;

    q1v = wrap_pi(q1v);
    q2v = wrap_pi(q2v);

    arm_clamp_joints(&yaw, &q1v, &q2v);

    *q0 = yaw;
    *q1 = q1v;
    *q2 = q2v;
    return 0;
}
