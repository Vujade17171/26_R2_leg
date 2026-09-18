/**
  ******************************************************************************
  * @file    arm_kinematics.h
  * @brief   机械臂运动学、关节限位与腕关节水平约束
  ******************************************************************************
  * 模块职责：
  *   1. 平面二连杆正/逆运动学；
  *   2. 关节限位与角度归一化；
  *   3. 关节角和电机角之间的零点映射；
  *   4. 腕关节 L3 水平约束的计算与周期更新。
  *
  * 坐标和关节顺序：
  *   q0 = 腕关节偏航角，q1 = 肩关节角，q2 = 肘关节角；
  *   x,z = 平面二连杆腕关节中心位置，单位为 m；
  *   所有角度单位为 rad。
  *
  * 本模块不发送 CAN 报文，也不生成五次多项式轨迹。
  ******************************************************************************
  */
#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdint.h>

/* 平面二连杆几何参数，单位：m */
#define ARM_L1                  0.35f
#define ARM_L2                  0.25f

/* 腕关节保持水平时的标定常数，来源于两个水平姿态的实测值 */
#define ARM_L3_LEVEL_C          2.9202114f

/* 腕关节水平补偿的最大速度，单位：rad/s */
#define ARM_L3_MAX_SPEED        4.0f

/* 统一关节编号，避免代码中出现含义不明的 0/1/2 */
typedef enum
{
    ARM_JOINT_WRIST = 0,
    ARM_JOINT_SHOULDER = 1,
    ARM_JOINT_ELBOW = 2,
    ARM_JOINT_COUNT = 3
} arm_joint_id_t;

/* 三个关节的上下限，单位为 rad */
typedef struct
{
    float q_min[ARM_JOINT_COUNT];
    float q_max[ARM_JOINT_COUNT];
} arm_joint_limit_t;

extern arm_joint_limit_t g_arm_joint_limit;

/* ---------- 关节角与电机角映射 ---------- */

/* 机械臂关节角 -> 电机角（包含零点和方向修正） */
float arm_joint_to_motor_1(float shoulder_joint);
float arm_joint_to_motor_2(float elbow_joint);

/* 电机角 -> 机械臂关节角（用于反馈同步） */
float arm_motor_to_joint_1(float shoulder_motor);
float arm_motor_to_joint_2(float elbow_motor);

/* ---------- 平面二连杆运动学 ---------- */

/* 正运动学：由肩、肘关节角计算腕关节中心位置 (x,z) */
void arm_forward(float shoulder_joint, float elbow_joint,
                 float *x, float *z);

/* 可达性判断：目标可行返回 0，不可行返回 -1 */
int arm_reachable(float x, float z);

/* 逆运动学：同时检查肘部向上/向下两种构型，
 * 选择最接近当前肩、肘角且满足限位的一组解。 */
int arm_inverse_nearest(float x, float z, float yaw,
                        float shoulder_ref, float elbow_ref,
                        float *wrist, float *shoulder, float *elbow);

/* ---------- 关节限位 ---------- */

/* 判断指定关节角是否位于限位内 */
int arm_in_limit(float joint_angle, int joint_id);

/* 将三个关节角分别限制到上下限；指针为 NULL 时跳过该关节 */
void arm_clamp_joints(float *wrist, float *shoulder, float *elbow);

/* ---------- 腕关节 L3 水平约束 ---------- */

/* 复位 L3 水平约束模块，并以 q0_current 作为初始命令 */
void arm_l3_level_init(float q0_current);

/* 启动 L3 水平约束，并锁存当前腕关节命令 */
void arm_l3_level_start(float q0_current);

/* 停止 L3 水平约束 */
void arm_l3_level_stop(void);

/* L3 水平约束是否处于激活状态 */
uint8_t arm_l3_level_is_active(void);

/* 更新保存的腕关节命令，不改变激活状态 */
void arm_l3_level_set_q0_cmd(float q0);

/* 根据肩、肘角计算保持 L3 水平的腕关节目标角；
 * q0_ref 用于选择最接近的等效角度。 */
float arm_l3_level_target(float shoulder_joint, float elbow_joint,
                          float q0_ref);

/* 周期更新 L3 腕关节命令。
 * max_step 为本周期 q0 最大变化量，v0 可输出腕关节速度前馈。 */
float arm_l3_level_update(float shoulder_joint, float elbow_joint,
                          float shoulder_vel, float elbow_vel,
                          float max_step, float *wrist_vel);

#endif /* ARM_KINEMATICS_H */
