/**
  ******************************************************************************
  * @file    leg_task.c
  * @brief   Leg_Task FreeRTOS 任务实现（空任务骨架）
  *
  * 说明：
  *  - 本文件提供 leg_task() 的强定义，覆盖 CubeMX 在 freertos.c 中生成的
  *    __weak void leg_task(void *argument)；
  *  - 任务配置（CubeMX .ioc）：名称 Leg_Task，优先级 osPriorityLow(8)，
  *    堆栈 256*4 字节，CMSIS-RTOS v2；
  *  - 在下方两个 USER CODE 区域中填写实际控制逻辑即可。
  ******************************************************************************
  */
#include "leg_task.h"

/**
  * @brief  实现 Leg_Task 线程
  * @param  argument: 未使用（创建时传 NULL）
  * @retval 无
  */
void leg_task(void *argument)
{
  (void)argument;

  /* USER CODE BEGIN leg_task_Init */
  /* TODO: 任务启动前的一次性初始化，如等待 FDCAN/外设就绪等 */

  /* USER CODE END leg_task_Init */

  /* Infinite loop */
  for (;;)
  {
    /* USER CODE BEGIN leg_task_Loop */
    /* TODO: 腿控主逻辑；如需固定频率控制（如 1 kHz），
     *       可改用信号量/事件等待或 osDelay(1) 调度 */

    /* USER CODE END leg_task_Loop */
    osDelay(1);
  }
}
