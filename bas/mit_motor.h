/**
  ******************************************************************************
  * @file    mit_motor.h
  * @brief   Unified MIT-mode motor driver (AK80-9 / AK45-10 / ...)
  ******************************************************************************
  * Description:
  *   - One driver handles ALL MIT-mode motors on the same FDCAN bus.
  *   - Uses a config table (manual Table 30) so adding a motor is one more row.
  *   - Only parses feedback frames whose ID belongs to a configured motor.
  ******************************************************************************
  */
#ifndef __MIT_MOTOR_H
#define __MIT_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define MIT_MOTOR_MAX_NUM   4     /* max MIT motors supported */

/* ---- one config row per motor ---- */
//cfg配置的意思：电机配置参数
typedef struct
{
    uint8_t  id;             /* CAN ID == motor ID */
    float    p_min, p_max;   /* position limits rad */
    float    v_min, v_max;   /* velocity limits rad/s */
    float    t_min, t_max;   /* torque limits Nm */
    float    kp_min, kp_max; /* Kp limits */
    float    kd_min, kd_max; /* Kd limits */
    int8_t   sign;           /* +1 normal, -1 reverse direction */
} mit_motor_cfg_t;

/* ---- live state per motor ---- */
typedef struct
{
    float    pos;         /* position rad */
    float    vel;         /* velocity rad/s */
    float    torque;      /* torque Nm */
    int8_t   temp;        /* temperature C */
    uint8_t  error;       /* error code */
    uint32_t last_rx_ms;  /* last feedback time ms */
    uint8_t  online;      /* online flag */
    uint8_t  id;          /* motor ID */
} mit_motor_state_t;

extern mit_motor_state_t mit_motor_state[MIT_MOTOR_MAX_NUM];
extern uint32_t          g_mit_motor_n;   /* number of configured motors */

/* Init: register the single rx callback on the bus. */
uint8_t mit_motor_init(FDCAN_HandleTypeDef *hfdcan);

/* Add one motor config. Returns index (0..n-1) or -1 if full/invalid. */
int8_t mit_motor_add(const mit_motor_cfg_t *cfg);

/* Find motor index by ID, or -1. */
int8_t mit_motor_find(uint8_t id);

/* MIT run / idle / zero commands. */
void mit_motor_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void mit_motor_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void mit_motor_zero(FDCAN_HandleTypeDef *hfdcan, uint8_t id);

/* MIT control command:
 *   p_des : target position rad
 *   v_des : target velocity rad/s
 *   kp    : position gain
 *   kd    : velocity gain
 *   t_ff  : feed-forward torque Nm
 */
void mit_motor_set_control(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                           float p_des, float v_des,
                           float kp, float kd, float t_ff);

/* Rx parse (called by bus layer callback). Only handles configured MIT IDs. */
void mit_motor_unpack(FDCAN_HandleTypeDef *hfdcan,
                      FDCAN_RxHeaderTypeDef *rx_header,
                      uint8_t *rx_data);

/* Convenience: get state by ID, or NULL. */
mit_motor_state_t *mit_motor_get_state(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __MIT_MOTOR_H */