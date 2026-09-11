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

typedef struct
{
    volatile uint32_t rx_total;       /* all frames received */
    volatile uint32_t rx_std;         /* standard frames */
    volatile uint32_t rx_ext;         /* extended frames */
    volatile uint32_t rx_com2;        /* RobStride feedback frames */
    volatile uint32_t last_id;
    volatile uint32_t last_ext_id;
    volatile uint32_t last_com2_id;
    volatile uint8_t  last_id_type;
    volatile uint8_t  last_ext_com;
    volatile uint8_t  last_ext_fid;
    volatile uint8_t  last_ext_dlc;
    volatile uint8_t  last_ext_data[8];
    volatile uint8_t  com2_fid;
    volatile uint8_t  com2_data[8];

    /* Mode-2 send diagnostics: counts are incremented before a frame is queued. */
    volatile uint32_t tx_el05_enable_cnt;
    volatile uint32_t tx_el05_control_cnt;
    volatile uint32_t tx_ak1_control_cnt;
    volatile uint32_t tx_ak2_control_cnt;
    volatile uint32_t tx_el05_skipped;
} motor_app_can_dbg_t;

extern motor_app_can_dbg_t motor_app_can_dbg;

void motor_app_init(void);
void motor_app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_APP_H */
