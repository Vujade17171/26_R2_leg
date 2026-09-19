/**
  ******************************************************************************
  * @file    arm_task.h
  * @brief   机械臂 FreeRTOS 任务入口
  ******************************************************************************
  * 本模块只负责 FreeRTOS 调度和公开接口，具体控制逻辑位于 arm_control.c。
  ******************************************************************************
  */
#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "arm_control.h"

/* 在 RTOS 调度器启动前初始化机械臂控制器。 */
void arm_task_hardware_init(FDCAN_HandleTypeDef *hfdcan);

/* FreeRTOS 机械臂控制任务入口，由 freertos.c 创建任务时调用。 */
void arm_task(void *argument);

/* TIM6 每次中断调用一次，用于记录一个新的控制节拍。 */
void arm_task_tick(void);

#endif /* ARM_TASK_H */
