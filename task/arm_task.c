/**
  ******************************************************************************
  * @file    arm_task.c
  * @brief   机械臂 FreeRTOS 周期任务
  ******************************************************************************
  * 本模块只负责任务调度，控制算法和硬件逻辑统一由 arm_control.c 完成。
  ******************************************************************************
  */
#include "arm_task.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

static volatile uint32_t s_control_tick = 0U;
static uint32_t s_control_tick_seen = 0U;

void arm_task_hardware_init(FDCAN_HandleTypeDef *hfdcan)
{
    arm_control_init(hfdcan);
}

/* 记录一次由 TIM6 产生的控制节拍。 */
void arm_task_tick(void)
{
    s_control_tick++;
}

/* FreeRTOS 机械臂控制任务入口：启动 TIM6 并执行周期(2 ms)控制循环。 */
void arm_task(void *argument)
{
    uint32_t tick_now;

    (void)argument;

    /* 进入任务后再启动 TIM6，避免调度器启动前产生控制节拍。 */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        /* 控制尚未初始化时，由控制器内部状态处理后续安全逻辑。 */
        arm_control_fail_safe(-2);
    }

    for (;;)
    {
        /* 每 1 ms 检查一次，有新节拍时执行一次控制更新。 */
        (void)osDelay(1);

        /* 先保存本次节拍，避免判断与赋值之间被 TIM6 中断更新。 */
        tick_now = s_control_tick;

        if (tick_now != s_control_tick_seen)
        {
            s_control_tick_seen = tick_now;
            arm_control_step(ARM_CONTROL_DT);
        }
    }
}
