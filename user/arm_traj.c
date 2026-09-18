/**
  ******************************************************************************
  * @file    arm_traj.c
  * @brief   五次多项式点到点关节轨迹
  ******************************************************************************
  * 归一化时间 tau = elapsed / duration，位置比例：
  *   s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
  *
  * 该多项式满足起止位置、速度和加速度边界条件。
  * 本模块只生成 3 个关节的目标位置和速度，不参与运动学计算。
  ******************************************************************************
  */
#include "arm_traj.h"

#include <stddef.h>

#define ARM_TRAJ_JOINT_COUNT  3
#define ARM_TRAJ_MIN_DURATION 0.01f

/* 轨迹内部状态 */
static float s_start[ARM_TRAJ_JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static float s_end[ARM_TRAJ_JOINT_COUNT] = {0.0f, 0.0f, 0.0f};
static float s_duration_s = 1.0f;
static float s_elapsed_s = 0.0f;
static int   s_active = 0;

void arm_traj_start(float q0_start, float q1_start, float q2_start,
                    float q0_end, float q1_end, float q2_end,
                    float duration_s)
{
    s_start[0] = q0_start;
    s_start[1] = q1_start;
    s_start[2] = q2_start;

    s_end[0] = q0_end;
    s_end[1] = q1_end;
    s_end[2] = q2_end;

    s_duration_s = (duration_s > 0.0f) ? duration_s : ARM_TRAJ_MIN_DURATION;
    s_elapsed_s = 0.0f;
    s_active = 1;
}

int arm_traj_update(float dt,
                    float *q0, float *q1, float *q2,
                    float *v0, float *v1, float *v2)
{
    float tau;
    float tau2;
    float position_scale;
    float velocity_scale;
    float inv_duration;
    float delta[ARM_TRAJ_JOINT_COUNT];

    if (s_active == 0)
    {
        return 1;
    }

    s_elapsed_s += dt;
    tau = s_elapsed_s / s_duration_s;

    if (tau >= 1.0f)
    {
        tau = 1.0f;
        s_active = 0;
    }

    tau2 = tau * tau;
    inv_duration = 1.0f / s_duration_s;

    position_scale = tau * tau2 * (10.0f + tau * (-15.0f + 6.0f * tau));
    velocity_scale = tau2 * inv_duration
                   * (30.0f + tau * (-60.0f + 30.0f * tau));

    delta[0] = s_end[0] - s_start[0];
    delta[1] = s_end[1] - s_start[1];
    delta[2] = s_end[2] - s_start[2];

    if (q0 != NULL)
    {
        *q0 = s_start[0] + delta[0] * position_scale;
    }
    if (q1 != NULL)
    {
        *q1 = s_start[1] + delta[1] * position_scale;
    }
    if (q2 != NULL)
    {
        *q2 = s_start[2] + delta[2] * position_scale;
    }

    if (v0 != NULL)
    {
        *v0 = delta[0] * velocity_scale;
    }
    if (v1 != NULL)
    {
        *v1 = delta[1] * velocity_scale;
    }
    if (v2 != NULL)
    {
        *v2 = delta[2] * velocity_scale;
    }

    return (s_active != 0) ? 0 : 1;
}

void arm_traj_stop(void)
{
    s_active = 0;
}

int arm_traj_is_active(void)
{
    return s_active;
}
