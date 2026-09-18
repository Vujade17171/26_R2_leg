/**
  ******************************************************************************
  * @file    arm_task.h
  * @brief   机械臂 FreeRTOS 控制任务（3 个电机，共用 FDCAN 总线）
  ******************************************************************************
  * 模块职责：
  *   1. 初始化电机、FDCAN 和控制系统；
  *   2. 创建并运行 FreeRTOS 周期控制任务；
  *   3. 在任务中完成逆运动学、五次多项式轨迹和电机控制；
  *   4. 提供调试变量与目标命令接口。
  *
  * 调试方法：
  *   - arm_target[0..2] = (x, z, 偏航角)，单位：m、m、rad；
  *   - arm_cmd_new = 1 时，任务执行一次新的笛卡尔目标；
  *   - arm_dbg.joint[] 的顺序为：肩、肘、腕。
  ******************************************************************************
  */
#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ARM_MOTOR_NUM   3

/* 每个关节的调试信息（在 Keil Watch 中查看 arm_dbg）。 */
typedef struct
{
    float angle;    /* 当前关节角，单位 rad */
    float target;   /* 当前下发目标，单位 rad */
} arm_joint_dbg_t;

typedef struct
{
    arm_joint_dbg_t joint[ARM_MOTOR_NUM];  /* [0] 肩，[1] 肘，[2] 腕 */
    float x;                               /* 笛卡尔目标 x */
    float z;                               /* 笛卡尔目标 z */
    float x_actual;                        /* 正运动学计算的腕关节中心 x */
    float z_actual;                        /* 正运动学计算的腕关节中心 z */
    uint8_t reached;                       /* 1 表示已到达目标 */
    int16_t last_err;                      /* 0=正常，-1=不可达，其他为错误码 */
} arm_dbg_t;

extern arm_dbg_t arm_dbg;

/* Keil 调试输入变量（可在 Watch 窗口修改）。 */
extern volatile float   arm_target[3];   /* 笛卡尔目标：(x, z, 偏航角) */
extern volatile uint8_t arm_cmd_new;     /* 置 1 以执行新目标 */

/* 在 RTOS 调度器启动前初始化电机、FDCAN 和控制器状态。 */
void arm_task_hardware_init(FDCAN_HandleTypeDef *hfdcan);

/* 在 RTOS 调度器启动前创建机械臂控制任务。 */
void arm_task_start(void);

/* TIM6 每次中断调用一次，用于记录一个新的控制节拍。 */
void arm_task_tick(void);

/* 提交一个新的笛卡尔目标；成功返回 0，失败返回 -1。 */
int arm_goto(float x, float z, float yaw);

#endif /* ARM_TASK_H */