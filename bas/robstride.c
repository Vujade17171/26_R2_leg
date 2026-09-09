/**
  ******************************************************************************
  * @file    robstride.c
  * @brief   RobStride (Lingzu-05) motor driver - CAN 2.0 extended frame
  ******************************************************************************
  */
#include "robstride.h"
#include "fdcan_drv.h"
#include <math.h>

robstride_state_t robstride_state[ROBSTRIDE_MAX_NUM] = {0};
uint32_t          g_robstride_n = 0;



static robstride_cfg_t s_cfg[ROBSTRIDE_MAX_NUM] = {0};

/* float -> unsigned int quantization */
static uint32_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x < x_min) { x = x_min; }
    if (x > x_max) { x = x_max; }
    return (uint32_t)((x - x_min) * (float)((1 << bits) - 1) / span);
}

/* unsigned int -> float de-quantization */
static float uint_to_float(uint32_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (float)x * span / (float)((1 << bits) - 1) + x_min;
}

/* Build 29-bit extended ID: [com_type<<24][mid<<8][id] */
static uint32_t robstride_build_id(uint32_t com_type, uint32_t mid, uint8_t id)
{
    return ((com_type & 0x1F) << 24) | ((mid & 0xFFFF) << 8) | (uint32_t)id;
}

/* Register the single rx callback on the bus. */
uint8_t robstride_init(FDCAN_HandleTypeDef *hfdcan)
{
    g_robstride_n = 0;
    fdcan_drv_reg_rx_cb(hfdcan, robstride_unpack);
    return 0;
}

/* Find motor index by CAN ID, or -1. */
int8_t robstride_find(uint8_t id)
{
    uint32_t i;
    for (i = 0; i < g_robstride_n; i++)
    {
        if (s_cfg[i].id == id) { return (int8_t)i; }
    }
    return -1;
}

/* Add one motor config. */
int8_t robstride_add(const robstride_cfg_t *cfg)
{
    if (g_robstride_n >= ROBSTRIDE_MAX_NUM) { return -1; }
    if (cfg == NULL) { return -1; }
    if (robstride_find(cfg->id) >= 0) { return -1; }

    s_cfg[g_robstride_n] = *cfg;
    return (int8_t)g_robstride_n++;
}

/* Enable (com_type 0x03): data area all zero. */
void robstride_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    int8_t idx = robstride_find(id);
    uint8_t data[8] = {0};
    uint32_t ident;
    if (idx < 0) { return; }
    ident = robstride_build_id(0x03, s_cfg[idx].master_id, id);
    fdcan_drv_send(hfdcan, ident, FDCAN_EXTENDED_ID, data, 8);
}

/* Stop (com_type 0x04): byte0 = clear_error. */
void robstride_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t clear_error)
{
    int8_t idx = robstride_find(id);
    uint8_t data[8] = {0};
    uint32_t ident;
    if (idx < 0) { return; }
    data[0] = clear_error;
    ident = robstride_build_id(0x04, s_cfg[idx].master_id, id);
    fdcan_drv_send(hfdcan, ident, FDCAN_EXTENDED_ID, data, 8);
}

/* Zero (com_type 0x06): byte0 = 1. */
void robstride_zero(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    int8_t idx = robstride_find(id);
    uint8_t data[8] = {0};
    uint32_t ident;
    if (idx < 0) { return; }
    data[0] = 1;
    ident = robstride_build_id(0x06, s_cfg[idx].master_id, id);
    fdcan_drv_send(hfdcan, ident, FDCAN_EXTENDED_ID, data, 8);
}

/* Control command (com_type 0x01). */
void robstride_set_control(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                           float torque, float angle,
                           float speed, float kp, float kd)
{
    int8_t idx = robstride_find(id);
    const robstride_cfg_t *cfg;
    uint16_t tq, p, v, kpi, kdi;
    uint8_t data[8] = {0};
    uint32_t ident;

    if (idx < 0) { return; }
    cfg = &s_cfg[idx];

    /* direction sign */
    torque *= (float)cfg->sign;
    angle  *= (float)cfg->sign;
    speed  *= (float)cfg->sign;

    /* limit */
    if (torque < cfg->t_min) torque = cfg->t_min;
    if (torque > cfg->t_max) torque = cfg->t_max;
    if (angle  < cfg->p_min) angle  = cfg->p_min;
    if (angle  > cfg->p_max) angle  = cfg->p_max;
    if (speed  < cfg->v_min) speed  = cfg->v_min;
    if (speed  > cfg->v_max) speed  = cfg->v_max;
    if (kp < cfg->kp_min)    kp     = cfg->kp_min;
    if (kp > cfg->kp_max)    kp     = cfg->kp_max;
    if (kd < cfg->kd_min)    kd     = cfg->kd_min;
    if (kd > cfg->kd_max)    kd     = cfg->kd_max;

    /* quantize (all 16-bit) */
    tq  = (uint16_t)float_to_uint(torque, cfg->t_min, cfg->t_max, 16);
    p   = (uint16_t)float_to_uint(angle,  cfg->p_min, cfg->p_max, 16);
    v   = (uint16_t)float_to_uint(speed,  cfg->v_min, cfg->v_max, 16);
    kpi = (uint16_t)float_to_uint(kp,     cfg->kp_min,cfg->kp_max,16);
    kdi = (uint16_t)float_to_uint(kd,     cfg->kd_min,cfg->kd_max,16);

    /* data area: angle / speed / kp / kd */
    data[0] = (uint8_t)(p >> 8);   data[1] = (uint8_t)(p & 0xFF);
    data[2] = (uint8_t)(v >> 8);   data[3] = (uint8_t)(v & 0xFF);
    data[4] = (uint8_t)(kpi >> 8); data[5] = (uint8_t)(kpi & 0xFF);
    data[6] = (uint8_t)(kdi >> 8); data[7] = (uint8_t)(kdi & 0xFF);

    /* torque goes into the ID bits 23..8 */
    ident = robstride_build_id(0x01, (uint32_t)tq, id);
    fdcan_drv_send(hfdcan, ident, FDCAN_EXTENDED_ID, data, 8);
}

/* Rx parse: only handle com_type == 2 (motor feedback). */
void robstride_unpack(FDCAN_HandleTypeDef *hfdcan,
                      FDCAN_RxHeaderTypeDef *rx_header,
                      uint8_t *rx_data)
{
    uint32_t ident;
    uint32_t com;
    uint8_t  fid;
    int8_t   idx;
    const robstride_cfg_t *cfg;
    uint16_t a, v, t;
    int16_t  temp_raw;

    (void)hfdcan;

    if (rx_header->IdType != FDCAN_EXTENDED_ID) { return; }
    ident = rx_header->Identifier;
    com   = (ident & 0x3F000000) >> 24;
    if (com != 2) { return; }               /* only motor feedback */

    fid = (uint8_t)((ident & 0xFF00) >> 8); /* motor ID in bits 15..8 */
    idx = robstride_find(fid);
    if (idx < 0) { return; }
    cfg = &s_cfg[idx];

    a = (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
    v = (uint16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
    t = (uint16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
    temp_raw = (int16_t)(uint16_t)(((uint16_t)rx_data[6] << 8) | rx_data[7]);

    robstride_state[idx].id      = fid;
    robstride_state[idx].angle   = uint_to_float(a, cfg->p_min, cfg->p_max, 16) * (float)cfg->sign;
    robstride_state[idx].speed   = uint_to_float(v, cfg->v_min, cfg->v_max, 16) * (float)cfg->sign;
    robstride_state[idx].torque  = uint_to_float(t, cfg->t_min, cfg->t_max, 16) * (float)cfg->sign;
    robstride_state[idx].temp    = (float)temp_raw * 0.1f;
    robstride_state[idx].error   = (uint8_t)((ident & 0x3F0000) >> 16);
    robstride_state[idx].pattern = (uint8_t)((ident & 0xC00000) >> 22);
    robstride_state[idx].last_rx_ms = HAL_GetTick();
    robstride_state[idx].online  = 1;
}

/* Convenience: get state by ID, or NULL. */
robstride_state_t *robstride_get_state(uint8_t id)
{
    int8_t idx = robstride_find(id);
    if (idx < 0) { return NULL; }
    return &robstride_state[idx];
}