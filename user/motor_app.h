/**
  ******************************************************************************
  * @file    motor_app.h
  * @brief   Application layer interface
  ******************************************************************************
  */
#ifndef __MOTOR_APP_H
#define __MOTOR_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t g_rx_fifo_level;   /* RX FIFO0 fill level diag */
extern uint32_t          g_mit_tx_cnt;      /* total MIT control frames sent */

void motor_app_init(void);
void motor_app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_APP_H */