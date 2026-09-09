/**
  ******************************************************************************
  * @file    robstride.h
  * @brief   RobStride (Lingzu-05) motor driver - CAN 2.0 extended frame
  ******************************************************************************
  * Description:
  *   - Wrist motor "Lingzu-05", private RobStride protocol.
  *   - 29-bit extended ID: [com_type(5b)][mid/torque(16b)][motor_id(8b)].
  *   - NOTE: for control frame (0x01) the target TORQUE is carried in the
  *           ID bits 23..8 (NOT in the data area). Data area = angle/speed/kp/kd.
  ******************************************************************************
  */
#ifndef __ROBSTRIDE_H
#define __ROBSTRIDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define ROBSTRIDE_MAX_NUM   4     /* max wrist motors supported */

/* ---- one config row per motor ---- */
typedef struct
{
    uint8_t  id;            /* motor CAN ID (0x00..0x7F) */
    float    p_min, p_max;  /* position limits rad */
    float    v_min, v_max;  /* velocity limits rad/s */
    float    t_min, t_max;  /* torque limits Nm */
    float    kp_min, kp_max;/* Kp limits */
    float    kd_min, kd_max;/* Kd limits */
    int8_t   sign;          /* +1 normal, -1 reverse direction */
    uint16_t master_id;     /* host CAN ID (bits 15..8) */
} robstride_cfg_t;

/* ---- live state per motor ---- */
typedef struct
{
    float    angle;      /* position rad */
    float    speed;      /* velocity rad/s */
    float    torque;     /* torque Nm */
    float    temp;       /* temperature C (raw*0.1) */
    uint8_t  error;      /* error code (from ID bits 21..16) */
    uint8_t  pattern;    /* run mode flag (from ID bits 23..22) */
    uint32_t last_rx_ms; /* last feedback time ms */
    uint8_t  online;     /* online flag */
    uint8_t  id;         /* motor ID */
} robstride_state_t;

extern robstride_state_t robstride_state[ROBSTRIDE_MAX_NUM];
extern uint32_t          g_robstride_n;   /* number of configured motors */

/* Init: register the single rx callback on the bus. */
uint8_t robstride_init(FDCAN_HandleTypeDef *hfdcan);

/* Add one motor config. Returns index (0..n-1) or -1 if full/invalid. */
int8_t robstride_add(const robstride_cfg_t *cfg);

/* Find motor index by ID, or -1. */
int8_t robstride_find(uint8_t id);

/* Enable (0x03) / stop (0x04) / zero (0x06). */
void robstride_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void robstride_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t clear_error);
void robstride_zero(FDCAN_HandleTypeDef *hfdcan, uint8_t id);

/* Control command (0x01):
 *   torque : target torque Nm        -> carried in ID bits 23..8
 *   angle  : target position rad
 *   speed  : target velocity rad/s
 *   kp     : position gain
 *   kd     : velocity gain
 */
void robstride_set_control(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                           float torque, float angle,
                           float speed, float kp, float kd);

/* Rx parse: only handles com_type==2 (motor feedback). */
void robstride_unpack(FDCAN_HandleTypeDef *hfdcan,
                      FDCAN_RxHeaderTypeDef *rx_header,
                      uint8_t *rx_data);

/* Convenience: get state by ID, or NULL. */
robstride_state_t *robstride_get_state(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __ROBSTRIDE_H */