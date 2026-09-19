/**
  ******************************************************************************
  * @file    arm_control.h
  * @brief   机械臂控制器接口（3 个电机，共用 FDCAN 总线）
  ******************************************************************************
  * 模块职责：
  *   1. 初始化电机、FDCAN 和控制系统；
  *   2. 完成逆运动学、五次多项式轨迹和电机控制；
  *   3. 提供调试变量与目标命令接口。
  *
  * 调试方法：
  *   - arm_target[0..2] = (x, z, 偏航角)，单位：m、m、rad；
  *   - arm_cmd_new = 1 时，执行一次新的笛卡尔目标；
  *   - arm_kp_* / arm_kd_* 可在 Watch 中在线修改电机增益；
  *   - arm_dbg.joint[] 的顺序为：肩、肘、腕。
  ******************************************************************************
  */
#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ARM_MOTOR_NUM   3

/* 固定控制周期和执行步长，单位分别为 ms、s。 */
#define ARM_CONTROL_PERIOD_MS   2
#define ARM_CONTROL_DT          ((float)ARM_CONTROL_PERIOD_MS * 0.001f)

/* 每个关节的调试信息（在 Keil Watch 中查看 arm_dbg）。 */
typedef struct
{
    float angle;    /* 当前关节角，单位 rad */
} arm_joint_dbg_t;

typedef struct
{
    arm_joint_dbg_t joint[ARM_MOTOR_NUM];  /* [0] 肩，[1] 肘，[2] 腕 */
    float x;                               /* 笛卡尔目标 x */
    float z;                               /* 笛卡尔目标 z */
    float x_actual;                        /* 正运动学计算的腕关节中心 x */
    float z_actual;                        /* 正运动学计算的腕关节中心 z */
    uint8_t reached;                       /* 1 表示五次多项式执行完成 */
    int16_t last_err;                      /* 0=正常，-1=不可达，-2=安全停机，-3=力矩过载，-4=连续发送失败 */
} arm_dbg_t;

extern arm_dbg_t arm_dbg;

/* Keil 调试输入变量（可在 Watch 窗口修改）。 */
extern volatile float   arm_target[3];   /* 笛卡尔目标：(x, z, 偏航角) */
extern volatile uint8_t arm_cmd_new;     /* 置 1 以执行新目标 */

/* 电机增益（可在 Watch 窗口在线修改，修改后下一控制周期生效）。 */
extern volatile float arm_kp_shoulder;
extern volatile float arm_kd_shoulder;
extern volatile float arm_kp_elbow;
extern volatile float arm_kd_elbow;
extern volatile float arm_kp_wrist;
extern volatile float arm_kd_wrist;

/* 在 RTOS 调度器启动前初始化电机、FDCAN 和控制器状态。 */
void arm_control_init(FDCAN_HandleTypeDef *hfdcan);

/* 执行一次固定周期控制更新。 */
void arm_control_step(float dt);

/* 请求控制器进入安全停机状态。 */
void arm_control_fail_safe(int16_t error_code);

/* 提交一个新的笛卡尔目标；成功返回 0，失败返回 -1。 */
int arm_goto(float x, float z, float yaw);

#endif /* ARM_CONTROL_H */
