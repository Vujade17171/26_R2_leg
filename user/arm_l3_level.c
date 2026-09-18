#include "arm_l3_level.h"
#include "arm_kinematics.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float   s_q0_cmd = 0.0f;
static uint8_t s_l3_level_active = 0U;

static float arm_l3_level_limit_value(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static float arm_l3_level_slew_limit(float target, float current, float max_step)
{
    float d = target - current;

    if (d >  max_step) { d =  max_step; }
    if (d < -max_step) { d = -max_step; }
    return current + d;
}

void arm_l3_level_init(float q0_current)
{
    s_l3_level_active = 0U;
    s_q0_cmd = q0_current;
}

void arm_l3_level_start(float q0_current)
{
    s_l3_level_active = 1U;
    s_q0_cmd = q0_current;
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
    s_q0_cmd = q0;
}

float arm_l3_level_target(float q1, float q2, float q0_ref)
{
    float q0 = ARM_L3_LEVEL_C - (q1 + q2);

    while ((q0 - q0_ref) > M_PI)  { q0 -= 2.0f * M_PI; }
    while ((q0 - q0_ref) < -M_PI) { q0 += 2.0f * M_PI; }

    arm_clamp_joints(&q0, NULL, NULL);
    return q0;
}

float arm_l3_level_update(float q1, float q2,
                          float v1, float v2,
                          float max_step,
                          float *v0)
{
    float q0_next;

    if (s_l3_level_active == 0U)
    {
        if (v0 != NULL) { *v0 = 0.0f; }
        return s_q0_cmd;
    }

    q0_next = arm_l3_level_target(q1, q2, s_q0_cmd);
    q0_next = arm_l3_level_slew_limit(q0_next, s_q0_cmd, max_step);
    arm_clamp_joints(&q0_next, NULL, NULL);
    s_q0_cmd = q0_next;

    if (v0 != NULL)
    {
        *v0 = arm_l3_level_limit_value(-(v1 + v2),
                                       -ARM_L3_MAX_SPEED,
                                        ARM_L3_MAX_SPEED);
    }

    return s_q0_cmd;
}
