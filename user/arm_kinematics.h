/**
  ******************************************************************************
  * @file    arm_kinematics.h
  * @brief   平面二连杆机械臂运动学（逆运动学 / 可达性判断 / 关节-电机角度映射）
  ******************************************************************************
  * 说明：
  *   - XZ 平面二连杆逆运动学（肘部构型选择）。
  *   - 使用配置零点偏移进行机械臂关节角与电机角之间的转换。
  *   - 保存三个关节的限位参数（参考工程参数）。
  *   - q0 = 腕关节（偏航），q1 = 肩关节，q2 = 肘关节。
  ******************************************************************************
  */
#ifndef __ARM_KINEMATICS_H
#define __ARM_KINEMATICS_H

#include <stdint.h>

#define ARM_L1  0.35f   /* 大臂长度（m） */
#define ARM_L2  0.25f   /* 小臂长度（m） */

/* 关节限位表（q0=腕关节，q1=肩关节，q2=肘关节） */
typedef struct
{
    float q_min[3];
    float q_max[3];
} arm_joint_limit_t;

extern arm_joint_limit_t g_arm_joint_limit;

/* 机械臂关节角 -> 电机角（包含方向和零点偏移） */
float arm_joint_to_motor_1(float q1);   /* 肩关节 */
float arm_joint_to_motor_2(float q2);   /* 肘关节 */

/* 电机角 -> 机械臂关节角（用于反馈显示） */
float arm_motor_to_joint_1(float m1);
float arm_motor_to_joint_2(float m2);

/* 正运动学：由前两轴 q1/q2 计算腕关节中心 (x,z)，不包含末端 L3。 */
void arm_forward(float q1, float q2, float *x, float *z);

/* 可达性判断：目标点 (x,z) 可达返回 0，不可达返回 -1 */
int arm_reachable(float x, float z);

/* 将三个关节角限制到各自的上下限内 */
void arm_clamp_joints(float *q0, float *q1, float *q2);

/* 判断关节角 q 是否在编号 idx（0~2）对应的限位内 */
int arm_in_limit(float q, int idx);

/* 逆运动学求解：同时计算两种肘部构型，并选择最接近当前关节角
 * 且满足限位的一组解。该方式可避免机械臂运动时突然翻肘。
 * 成功返回 0，两种构型都无有效解时返回 -1。 */
int arm_inverse_nearest(float x, float z, float yaw,
                        float q1_ref, float q2_ref,
                        float *q0, float *q1, float *q2);
#endif /* __ARM_KINEMATICS_H */
