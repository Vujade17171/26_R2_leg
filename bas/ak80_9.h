/**
  ******************************************************************************
  * @file    ak80_9.h
  * @brief   AK80-9 driver layer (MIT mode)
  ******************************************************************************
  * Description:
  *   - Based on bas/fdcan_drv universal bus layer.
  *   - MIT mode, standard frame (11-bit ID, ID == motor ID).
  *   - Supports up to 3 motors.
  ******************************************************************************
  */
#ifndef __AK80_9_H
#define __AK80_9_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* ============ AK80-9 MIT protocol limits ============ */
#define AK80_9_P_MIN   (-12.5f)   /* position min rad */
#define AK80_9_P_MAX   (12.5f)    /* position max rad */
#define AK80_9_V_MIN   (-65.0f)   /* velocity min rad/s */
#define AK80_9_V_MAX   (65.0f)    /* velocity max rad/s */
#define AK80_9_T_MIN   (-18.0f)   /* torque min Nm */
#define AK80_9_T_MAX   (18.0f)    /* torque max Nm */
#define AK80_9_KP_MIN  (0.0f)     /* Kp min */
#define AK80_9_KP_MAX  (500.0f)   /* Kp max */
#define AK80_9_KD_MIN  (0.0f)     /* Kd min */
#define AK80_9_KD_MAX  (5.0f)     /* Kd max */

#define AK80_9_MAX_NUM 3          /* max number of supported motors */

/* Motor state */
typedef struct
{
    float    pos;          /* position rad */
    float    vel;          /* velocity rad/s */
    float    torque;       /* torque Nm */
    int8_t   temp;         /* temperature C */
    uint8_t  error;        /* error code */
    uint32_t last_rx_ms;   /* last feedback time */
    uint8_t  online;       /* online flag */
    uint8_t  id;           /* motor ID */
} ak80_9_state_t;

extern ak80_9_state_t ak80_9_state[AK80_9_MAX_NUM];

/* Init: register rx callback to bus layer */
uint8_t ak80_9_init(FDCAN_HandleTypeDef *hfdcan);

/* MIT enable (enter run state) */
void ak80_9_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);

/* MIT disable (enter idle state) */
void ak80_9_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);

/* MIT zero (set current position as 0) */
void ak80_9_zero(FDCAN_HandleTypeDef *hfdcan, uint8_t id);

/* MIT control command
 *   p_des : target position rad
 *   v_des : target velocity rad/s
 *   kp    : position gain
 *   kd    : velocity gain
 *   t_ff  : feed-forward torque Nm
 */
void ak80_9_set_control(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                        float p_des, float v_des,
                        float kp, float kd, float t_ff);

/* Rx parse (called by bus layer callback) */
void ak80_9_unpack(FDCAN_HandleTypeDef *hfdcan,
                   FDCAN_RxHeaderTypeDef *rx_header,
                   uint8_t *rx_data);

#ifdef __cplusplus
}
#endif

#endif /* __AK80_9_H */