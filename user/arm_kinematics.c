/**
  ******************************************************************************
  * @file    arm_kinematics.c
  * @brief   机械臂运动学、关节限位与腕关节水平约束
  ******************************************************************************
  * 代码分区：
  *   1. 通用数学工具；
  *   2. 关节角/电机角映射；
  *   3. 正运动学、限位和可达性；
  *   4. 二连杆逆运动学；
  *   5. 腕关节 L3 水平约束。
  ******************************************************************************
  */
#include "arm_kinematics.h"

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 电机安装零点：关节角 = 电机角 - 偏移 */
#define ARM_SHOULDER_MOTOR_OFFSET   0.300000f
#define ARM_ELBOW_MOTOR_OFFSET      1.900000f

/* q0 = 腕关节，q1 = 肩关节，q2 = 肘关节 */
arm_joint_limit_t g_arm_joint_limit =
{
    .q_min = { -M_PI, -M_PI * 0.4f, -M_PI * 0.9f },
    .q_max = {  M_PI,  M_PI,         M_PI * 0.9f }
};

/* L3 水平约束的内部状态 */
static float   s_l3_q0_cmd = 0.0f;
static uint8_t s_l3_level_active = 0U;

/* ============================== 通用数学工具 ============================== */

static float arm_clamp_value(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float arm_wrap_pi(float angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0f * M_PI;
    }
    return angle;
}

static float arm_move_toward(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step)
    {
        delta = max_step;
    }
    else if (delta < -max_step)
    {
        delta = -max_step;
    }

    return current + delta;
}

/* ========================= 关节角 / 电机角映射 ========================= */

float arm_joint_to_motor_1(float shoulder_joint)
{
    return shoulder_joint + ARM_SHOULDER_MOTOR_OFFSET;
}

float arm_joint_to_motor_2(float elbow_joint)
{
    return elbow_joint - ARM_ELBOW_MOTOR_OFFSET;
}

float arm_motor_to_joint_1(float shoulder_motor)
{
    return shoulder_motor - ARM_SHOULDER_MOTOR_OFFSET;
}

float arm_motor_to_joint_2(float elbow_motor)
{
    return elbow_motor + ARM_ELBOW_MOTOR_OFFSET;
}

/* ========================== 正运动学与关节限位 ========================== */

void arm_forward(float shoulder_joint, float elbow_joint,
                 float *x, float *z)
{
    float shoulder_elbow_angle = shoulder_joint + elbow_joint;

    if (x != NULL)
    {
        *x = ARM_L1 * cosf(shoulder_joint)
           + ARM_L2 * cosf(shoulder_elbow_angle);
    }

    if (z != NULL)
    {
        *z = ARM_L1 * sinf(shoulder_joint)
           + ARM_L2 * sinf(shoulder_elbow_angle);
    }
}

int arm_in_limit(float joint_angle, int joint_id)
{
    if ((joint_id < 0) || (joint_id >= ARM_JOINT_COUNT))
    {
        return 0;
    }

    return (joint_angle >= g_arm_joint_limit.q_min[joint_id])
        && (joint_angle <= g_arm_joint_limit.q_max[joint_id]);
}

void arm_clamp_joints(float *wrist, float *shoulder, float *elbow)
{
    if (wrist != NULL)
    {
        *wrist = arm_clamp_value(*wrist,
                                 g_arm_joint_limit.q_min[ARM_JOINT_WRIST],
                                 g_arm_joint_limit.q_max[ARM_JOINT_WRIST]);
    }

    if (shoulder != NULL)
    {
        *shoulder = arm_clamp_value(*shoulder,
                                    g_arm_joint_limit.q_min[ARM_JOINT_SHOULDER],
                                    g_arm_joint_limit.q_max[ARM_JOINT_SHOULDER]);
    }

    if (elbow != NULL)
    {
        *elbow = arm_clamp_value(*elbow,
                                 g_arm_joint_limit.q_min[ARM_JOINT_ELBOW],
                                 g_arm_joint_limit.q_max[ARM_JOINT_ELBOW]);
    }
}

int arm_reachable(float x, float z)
{
    float distance_squared = x * x + z * z;
    float max_reach = ARM_L1 + ARM_L2;
    float min_reach = fabsf(ARM_L1 - ARM_L2);

    if (distance_squared > (max_reach * max_reach))
    {
        return -1;
    }
    if (distance_squared < (min_reach * min_reach))
    {
        return -1;
    }

    return 0;
}

/* ============================== 逆运动学 =============================== */

/* 求解一种肘部构型：
 * sin_sign = +1 时肘部向上，sin_sign = -1 时肘部向下。 */
static int arm_inverse_branch(float x, float z, float yaw, float sin_sign,
                              float *wrist, float *shoulder, float *elbow)
{
    float distance_squared;
    float cos_elbow;
    float sin_elbow;
    float elbow_angle;
    float target_angle;
    float shoulder_offset;

    if ((wrist == NULL) || (shoulder == NULL) || (elbow == NULL))
    {
        return -1;
    }
    if ((arm_reachable(x, z) != 0)
        || (arm_in_limit(yaw, ARM_JOINT_WRIST) == 0))
    {
        return -1;
    }

    distance_squared = x * x + z * z;
    cos_elbow = (distance_squared - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2)
              / (2.0f * ARM_L1 * ARM_L2);
    cos_elbow = arm_clamp_value(cos_elbow, -1.0f, 1.0f);

    sin_elbow = sin_sign * sqrtf(1.0f - cos_elbow * cos_elbow);
    elbow_angle = atan2f(sin_elbow, cos_elbow);

    target_angle = atan2f(z, x);
    shoulder_offset = atan2f(ARM_L2 * sin_elbow,
                             ARM_L1 + ARM_L2 * cos_elbow);

    shoulder_offset = arm_wrap_pi(target_angle - shoulder_offset);
    elbow_angle = arm_wrap_pi(elbow_angle);

    if ((arm_in_limit(shoulder_offset, ARM_JOINT_SHOULDER) == 0)
        || (arm_in_limit(elbow_angle, ARM_JOINT_ELBOW) == 0))
    {
        return -1;
    }

    *wrist = yaw;
    *shoulder = shoulder_offset;
    *elbow = elbow_angle;
    return 0;
}

int arm_inverse_nearest(float x, float z, float yaw,
                        float shoulder_ref, float elbow_ref,
                        float *wrist, float *shoulder, float *elbow)
{
    float up_wrist = 0.0f;
    float up_shoulder = 0.0f;
    float up_elbow = 0.0f;
    float down_wrist = 0.0f;
    float down_shoulder = 0.0f;
    float down_elbow = 0.0f;
    float up_cost;
    float down_cost;
    float up_shoulder_error;
    float up_elbow_error;
    float down_shoulder_error;
    float down_elbow_error;
    int up_valid;
    int down_valid;

    if ((wrist == NULL) || (shoulder == NULL) || (elbow == NULL))
    {
        return -1;
    }

    up_valid = arm_inverse_branch(x, z, yaw, +1.0f,
                                  &up_wrist, &up_shoulder, &up_elbow);
    down_valid = arm_inverse_branch(x, z, yaw, -1.0f,
                                    &down_wrist, &down_shoulder, &down_elbow);

    if ((up_valid != 0) && (down_valid != 0))
    {
        return -1;
    }
    if (up_valid != 0)
    {
        *wrist = down_wrist;
        *shoulder = down_shoulder;
        *elbow = down_elbow;
        return 0;
    }
    if (down_valid != 0)
    {
        *wrist = up_wrist;
        *shoulder = up_shoulder;
        *elbow = up_elbow;
        return 0;
    }

    /* 两种构型都有效时，选择与当前姿态最接近的一组。 */
    up_shoulder_error = arm_wrap_pi(up_shoulder - shoulder_ref);
    up_elbow_error = arm_wrap_pi(up_elbow - elbow_ref);
    up_cost = up_shoulder_error * up_shoulder_error
            + up_elbow_error * up_elbow_error;

    down_shoulder_error = arm_wrap_pi(down_shoulder - shoulder_ref);
    down_elbow_error = arm_wrap_pi(down_elbow - elbow_ref);
    down_cost = down_shoulder_error * down_shoulder_error
              + down_elbow_error * down_elbow_error;

    if (down_cost <= up_cost)
    {
        *wrist = down_wrist;
        *shoulder = down_shoulder;
        *elbow = down_elbow;
    }
    else
    {
        *wrist = up_wrist;
        *shoulder = up_shoulder;
        *elbow = up_elbow;
    }

    return 0;
}

/* ========================== 腕关节 L3 水平约束 ========================== */

void arm_l3_level_init(float q0_current)
{
    s_l3_level_active = 0U;
    s_l3_q0_cmd = q0_current;
}

void arm_l3_level_start(float q0_current)
{
    s_l3_level_active = 1U;
    s_l3_q0_cmd = q0_current;
}

void arm_l3_level_stop(void)
{
    s_l3_level_active = 0U;
}

uint8_t arm_l3_level_is_active(void)
{
    return s_l3_level_active;
}

void arm_l3_level_set_q0_cmd(float q0)
{
    s_l3_q0_cmd = q0;
}

float arm_l3_level_target(float shoulder_joint, float elbow_joint,
                          float q0_ref)
{
    float wrist_target = ARM_L3_LEVEL_C - (shoulder_joint + elbow_joint);

    /* 选择最接近 q0_ref 的等效角，避免跨越 ±π 时绕一整圈。 */
    while ((wrist_target - q0_ref) > M_PI)
    {
        wrist_target -= 2.0f * M_PI;
    }
    while ((wrist_target - q0_ref) < -M_PI)
    {
        wrist_target += 2.0f * M_PI;
    }

    arm_clamp_joints(&wrist_target, NULL, NULL);
    return wrist_target;
}

float arm_l3_level_update(float shoulder_joint, float elbow_joint,
                          float shoulder_vel, float elbow_vel,
                          float max_step, float *wrist_vel)
{
    float wrist_target;

    if (s_l3_level_active == 0U)
    {
        if (wrist_vel != NULL)
        {
            *wrist_vel = 0.0f;
        }
        return s_l3_q0_cmd;
    }

    wrist_target = arm_l3_level_target(shoulder_joint, elbow_joint,
                                       s_l3_q0_cmd);
    wrist_target = arm_move_toward(s_l3_q0_cmd, wrist_target, max_step);
    arm_clamp_joints(&wrist_target, NULL, NULL);
    s_l3_q0_cmd = wrist_target;

    if (wrist_vel != NULL)
    {
        *wrist_vel = arm_clamp_value(-(shoulder_vel + elbow_vel),
                                     -ARM_L3_MAX_SPEED,
                                      ARM_L3_MAX_SPEED);
    }

    return s_l3_q0_cmd;
}