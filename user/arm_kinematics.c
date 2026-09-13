/**
  ******************************************************************************
  * @file    arm_kinematics.c
  * @brief   Planar 2R arm kinematics
  ******************************************************************************
  */
#include "arm_kinematics.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* motor zero offsets (reference project) */
#define OFFSET_DOWN   0.300000f    /* 肩关节电机偏移*/
#define OFFSET_UP    1.900000f   /* 肘关节电机偏移*/


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
float arm_joint_to_motor_1(float q1) { return q1 + OFFSET_DOWN; }
float arm_joint_to_motor_2(float q2) { return q2 - OFFSET_UP;   }

/* motor -> joint */
float arm_motor_to_joint_1(float m1) { return m1 - OFFSET_DOWN; }
float arm_motor_to_joint_2(float m2) { return m2 + OFFSET_UP;   }

/* Forward kinematics: wrist-centre position from q1/q2 (L1/L2 only). */
void arm_forward(float q1, float q2, float *x, float *z)
{
    float q12 = q1 + q2;

    if (x != NULL)
    {
        *x = ARM_L1 * cosf(q1) + ARM_L2 * cosf(q12);
    }

    if (z != NULL)
    {
        *z = ARM_L1 * sinf(q1) + ARM_L2 * sinf(q12);
    }

}


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


/* Solve one elbow branch. sin_sign = +1: elbow-up, -1: elbow-down. */
static int arm_inverse_branch(float x, float z, float yaw, float sin_sign,
                              float *q0, float *q1, float *q2)
{
    float D2, cosq2, sinq2, q2v, phi, psi, q1v;

    if ((q0 == 0) || (q1 == 0) || (q2 == 0)) { return -1; }
    if (arm_reachable(x, z) != 0) { return -1; }
    if (arm_in_limit(yaw, 0) == 0) { return -1; }

    D2 = x * x + z * z;
    cosq2 = (D2 - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) /
            (2.0f * ARM_L1 * ARM_L2);
    if (cosq2 >  1.0f) cosq2 =  1.0f;
    if (cosq2 < -1.0f) cosq2 = -1.0f;

    sinq2 = sin_sign * sqrtf(1.0f - cosq2 * cosq2);
    q2v = atan2f(sinq2, cosq2);
    phi = atan2f(z, x);
    psi = atan2f(ARM_L2 * sinq2, ARM_L1 + ARM_L2 * cosq2);
    q1v = wrap_pi(phi - psi);
    q2v = wrap_pi(q2v);

    if (arm_in_limit(q1v, 1) == 0) { return -1; }
    if (arm_in_limit(q2v, 2) == 0) { return -1; }

    *q0 = yaw;
    *q1 = q1v;
    *q2 = q2v;
    return 0;
}

/* Choose the valid IK branch nearest to the current joint angles. */
int arm_inverse_nearest(float x, float z, float yaw,
                        float q1_ref, float q2_ref,
                        float *q0, float *q1, float *q2)
{
    float up_q0 = 0.0f, up_q1 = 0.0f, up_q2 = 0.0f;
    float down_q0 = 0.0f, down_q1 = 0.0f, down_q2 = 0.0f;
    float e1, e2, up_cost, down_cost;
    int up_ok, down_ok;

    if ((q0 == 0) || (q1 == 0) || (q2 == 0)) { return -1; }

    up_ok = arm_inverse_branch(x, z, yaw, +1.0f,
                               &up_q0, &up_q1, &up_q2);
    down_ok = arm_inverse_branch(x, z, yaw, -1.0f,
                                 &down_q0, &down_q1, &down_q2);

    if ((up_ok != 0) && (down_ok != 0)) { return -1; }
    if (up_ok != 0) {
        *q0 = down_q0; *q1 = down_q1; *q2 = down_q2;
        return 0;
    }
    if (down_ok != 0) {
        *q0 = up_q0; *q1 = up_q1; *q2 = up_q2;
        return 0;
    }

    e1 = wrap_pi(up_q1 - q1_ref);
    e2 = wrap_pi(up_q2 - q2_ref);
    up_cost = e1 * e1 + e2 * e2;

    e1 = wrap_pi(down_q1 - q1_ref);
    e2 = wrap_pi(down_q2 - q2_ref);
    down_cost = e1 * e1 + e2 * e2;

    if (down_cost <= up_cost) {
        *q0 = down_q0; *q1 = down_q1; *q2 = down_q2;
    } else {
        *q0 = up_q0; *q1 = up_q1; *q2 = up_q2;
    }
    return 0;
}
