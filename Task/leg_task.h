/**
  ******************************************************************************
  * @file    leg_task.h
  * @brief   Leg_Task FreeRTOS 任务接口
  *
  * 说明：
  *  - CubeMX 已在 freertos.c 中生成 __weak void leg_task(void *argument)
  *    本文件提供强定义版本，编译时自动覆盖弱函数，无需改动 freertos.c；
  *  - CubeMX 重新生成工程不影响本文件。
  ******************************************************************************
  */
#ifndef __LEG_TASK_H
#define __LEG_TASK_H

#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS 任务句柄（由 CubeMX 生成的 freertos.c 创建） */
extern osThreadId_t Leg_TaskHandle;

/* FreeRTOS 任务入口函数（强符号，覆盖 freertos.c 中的弱函数） */
void leg_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __LEG_TASK_H */
