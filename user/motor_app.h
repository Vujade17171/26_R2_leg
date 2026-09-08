/**
  ******************************************************************************
  * @file    motor_app.h
  * @brief   Application layer interface
  ******************************************************************************
  */
#ifndef __MOTOR_APP_H
#define __MOTOR_APP_H

#include <stdint.h>   /* for uint32_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Simple RX diagnostic global.
 * Watch 'g_rx_fifo_level' in the Keil debugger Watch window.
 * If it stays 0, frames never reached the FDCAN RX FIFO0 (physical-layer issue). */
extern volatile uint32_t g_rx_fifo_level;

void motor_app_init(void);
void motor_app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_APP_H */