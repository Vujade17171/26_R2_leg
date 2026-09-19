/**
  ******************************************************************************
  * @file    arm_math.h
  * @brief   机械臂公共数学工具
  ******************************************************************************
  */
#ifndef __ARM_MATH_H
#define __ARM_MATH_H

/* 将 value 限制在 [min_value, max_value] 范围内。 */
static inline float arm_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) { return min_value; }
    if (value > max_value) { return max_value; }
    return value;
}

/* current 每次最多向 target 靠近 max_step，避免目标变化过快。 */
static inline float arm_move_toward(float current,
                                    float target,
                                    float max_step)
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

#endif /* __ARM_MATH_H */
