/**
  ******************************************************************************
  * @file    ak80_9.c
  * @brief   AK80-9 driver layer (MIT mode)
  ******************************************************************************
  */
#include "ak80_9.h"
#include "fdcan_drv.h"
#include <math.h>

ak80_9_state_t ak80_9_state[AK80_9_MAX_NUM] = {0};

/* float -> unsigned int quantization */
static uint32_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x < x_min) { x = x_min; }
    if (x > x_max) { x = x_max; }
    return (uint32_t)((x - x_min) * (float)((1 << bits) - 1) / span);
}

/* unsigned int -> float de-quantization */
//前面小数转无符号整数逆过程
static float uint_to_float(uint32_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (float)x * span / (float)((1 << bits) - 1) + x_min;
}

/* Send 8-byte command (standard frame, ID == motor ID) */
static void ak80_9_send_cmd(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t *data)
{
    fdcan_drv_send(hfdcan, (uint32_t)id, FDCAN_STANDARD_ID, data, 8);
}

uint8_t ak80_9_init(FDCAN_HandleTypeDef *hfdcan)
{
    /* register rx callback */
    fdcan_drv_reg_rx_cb(hfdcan, ak80_9_unpack);
    return 0;
}

void ak80_9_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    ak80_9_send_cmd(hfdcan, id, data);
}

void ak80_9_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    ak80_9_send_cmd(hfdcan, id, data);
}

void ak80_9_zero(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    ak80_9_send_cmd(hfdcan, id, data);
}

void ak80_9_set_control(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                        float p_des, float v_des,
                        float kp, float kd, float t_ff)
{
    uint32_t p_int, v_int, kp_int, kd_int, t_int;
    uint8_t data[8] = {0};

    if (id < 1 || id > AK80_9_MAX_NUM) { return; }

    /* limit */
    if (p_des < AK80_9_P_MIN) p_des = AK80_9_P_MIN;
    if (p_des > AK80_9_P_MAX) p_des = AK80_9_P_MAX;
    if (v_des < AK80_9_V_MIN) v_des = AK80_9_V_MIN;
    if (v_des > AK80_9_V_MAX) v_des = AK80_9_V_MAX;
    if (kp < AK80_9_KP_MIN)   kp = AK80_9_KP_MIN;
    if (kp > AK80_9_KP_MAX)   kp = AK80_9_KP_MAX;
    if (kd < AK80_9_KD_MIN)   kd = AK80_9_KD_MIN;
    if (kd > AK80_9_KD_MAX)   kd = AK80_9_KD_MAX;
    if (t_ff < AK80_9_T_MIN)  t_ff = AK80_9_T_MIN;
    if (t_ff > AK80_9_T_MAX)  t_ff = AK80_9_T_MAX;

    /* quantize */
    p_int  = float_to_uint(p_des, AK80_9_P_MIN, AK80_9_P_MAX, 16);
    v_int  = float_to_uint(v_des, AK80_9_V_MIN, AK80_9_V_MAX, 12);
    kp_int = float_to_uint(kp,    AK80_9_KP_MIN, AK80_9_KP_MAX, 12);
    kd_int = float_to_uint(kd,    AK80_9_KD_MIN, AK80_9_KD_MAX, 12);
    t_int  = float_to_uint(t_ff,  AK80_9_T_MIN,  AK80_9_T_MAX,  12);

    /* pack */
    data[0] = (uint8_t)((p_int >> 8) & 0xFF);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)((v_int >> 4) & 0xFF);
    data[3] = (uint8_t)(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    data[4] = (uint8_t)(kp_int & 0xFF);
    data[5] = (uint8_t)((kd_int >> 4) & 0xFF);
    data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    data[7] = (uint8_t)(t_int & 0xFF);

    ak80_9_send_cmd(hfdcan, id, data);
}

void ak80_9_unpack(FDCAN_HandleTypeDef *hfdcan,
                   FDCAN_RxHeaderTypeDef *rx_header,
                   uint8_t *rx_data)
{
    uint8_t id;
    int32_t p_int, v_int, t_int;
    int32_t idx;

    (void)hfdcan;

    if (rx_header->IdType != FDCAN_STANDARD_ID) { return; }

    /* feedback frame byte0 is motor ID */
    id = rx_data[0];
    if (id < 1 || id > AK80_9_MAX_NUM) { return; }
    idx = id - 1;

    p_int = (int32_t)(((uint16_t)rx_data[1] << 8) | rx_data[2]);
    v_int = (int32_t)(((uint16_t)rx_data[3] << 4) | ((rx_data[4] >> 4) & 0x0F));
    t_int = (int32_t)(((uint16_t)(rx_data[4] & 0x0F) << 8) | rx_data[5]);

    ak80_9_state[idx].id         = id;
    ak80_9_state[idx].pos        = uint_to_float((uint32_t)p_int, AK80_9_P_MIN, AK80_9_P_MAX, 16);
    ak80_9_state[idx].vel        = uint_to_float((uint32_t)v_int, AK80_9_V_MIN, AK80_9_V_MAX, 12);
    ak80_9_state[idx].torque     = uint_to_float((uint32_t)t_int, AK80_9_T_MIN, AK80_9_T_MAX, 12);
    ak80_9_state[idx].temp       = (int8_t)(rx_data[6]) - 40;
    ak80_9_state[idx].error      = rx_data[7];
    ak80_9_state[idx].last_rx_ms = HAL_GetTick();
    ak80_9_state[idx].online     = 1;
}