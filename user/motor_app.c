/**
  ******************************************************************************
  * @file    motor_app.c
  * @brief   Application layer: robot-arm end-position control
  ******************************************************************************
  * Description:
  *   - Wraps arm_control for the main loop. Only calls arm_init / arm_run.
  *   - Three motors on FDCAN1: AK80-9(1), AK45-10(2), Lingzu-05(3).
  *   - Modify arm_target / arm_cmd_mode / arm_cmd_new in Keil debug.
  ******************************************************************************
  */
#include "motor_app.h"
#include "main.h"
#include "arm_control.h"

extern FDCAN_HandleTypeDef hfdcan1;

void motor_app_init(void)
{
    arm_init(&hfdcan1);
}

void motor_app_run(void)
{
    arm_run();
}
