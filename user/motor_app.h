/**
  ******************************************************************************
  * @file    motor_app.h
  * @brief   Application layer interface
  ******************************************************************************
  */
#ifndef __MOTOR_APP_H
#define __MOTOR_APP_H

#include <stdint.h>   /* for int8_t, uint8_t, uint32_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Debug-monitor structure.
 * Watch the global variable 'g_motor_mon' in the Keil debugger Watch window
 * to see the motor parameters updating in real time. */
typedef struct
{
    /* ---- motor feedback (auto-updated from ak80_9_state) ---- */
    float    pos;          /* actual position rad */
    float    vel;          /* actual velocity rad/s */
    float    torque;       /* actual torque Nm */
    int8_t   temp;         /* temperature C */
    uint8_t  error;        /* error code */
    uint8_t  online;       /* online flag */
    uint32_t last_rx_ms;   /* last feedback time ms */

    /* ---- last commanded values ---- */
    float    cmd_pos;      /* commanded position rad */
    float    cmd_vel;      /* commanded velocity rad/s */
    float    cmd_kp;       /* commanded Kp */
    float    cmd_kd;       /* commanded Kd */
    float    cmd_tq;       /* commanded feed-forward torque Nm */

    /* ---- counters ---- */
    uint32_t tx_cnt;       /* control frames sent */
} motor_monitor_t;

extern motor_monitor_t g_motor_mon;

void motor_app_init(void);
void motor_app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_APP_H */
