/**
  ******************************************************************************
  * @file    motor_app.h
  * @brief   Application layer interface
  ******************************************************************************
  * Description:
  *   - Entry point for the robot-arm end-position control.
  *   - Real work is in arm_control.c; here we only wire it into main loop.
  ******************************************************************************
  */
#ifndef __MOTOR_APP_H
#define __MOTOR_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void motor_app_init(void);
void motor_app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_APP_H */
