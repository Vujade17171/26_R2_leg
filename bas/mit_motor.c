/**
  ******************************************************************************
  * @file    mit_motor.c
  * @brief   通用 MIT 模式电机驱动（AK80-9 / AK45-10 / ...）
  ******************************************************************************
  * 说明：
  *   - 使用同一个驱动管理同一 FDCAN 总线上的所有 MIT 模式电机。
  *   - 使用配置表；新增电机只需增加一行配置。
  *   - 只解析反馈 ID 属于已配置电机的帧。
  ******************************************************************************
  */
#include "mit_motor.h"
#include "fdcan_drv.h"
#include <math.h>

mit_motor_state_t mit_motor_state[MIT_MOTOR_MAX_NUM] = {0};
//当前已经添加配置的电机数量
uint32_t          g_mit_motor_n = 0;

static mit_motor_cfg_t s_cfg[MIT_MOTOR_MAX_NUM] = {0};

/* float 转无符号整数的量化 */
static uint32_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x < x_min) { x = x_min; }
    if (x > x_max) { x = x_max; }
    return (uint32_t)((x - x_min) * (float)((1 << bits) - 1) / span);
}

/* 无符号整数转 float 的反量化 */
static float uint_to_float(uint32_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (float)x * span / (float)((1 << bits) - 1) + x_min;
}

/* 发送 8 字节命令（标准帧，ID 等于电机 ID） */
static void mit_motor_send_cmd(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t *data)
{
    fdcan_drv_send(hfdcan, (uint32_t)id, FDCAN_STANDARD_ID, data, 8);
}

/* 在总线上注册唯一的接收回调。 */
uint8_t mit_motor_init(FDCAN_HandleTypeDef *hfdcan)
{
    g_mit_motor_n = 0;
    fdcan_drv_reg_rx_cb(hfdcan, mit_motor_unpack);
    return 0;
}

/* 根据 CAN ID 查找电机索引；未配置时返回 -1。 */
int8_t mit_motor_find(uint8_t id)
{
    uint32_t i;
    for (i = 0; i < g_mit_motor_n; i++)
    {
        if (s_cfg[i].id == id) { return (int8_t)i; }
    }
    return -1;
}

/* 添加一个电机配置。 */
int8_t mit_motor_add(const mit_motor_cfg_t *cfg)
{
    if (g_mit_motor_n >= MIT_MOTOR_MAX_NUM) { return -1; }
    if (cfg == NULL || cfg->id < 1) { return -1; }
    if (mit_motor_find(cfg->id) >= 0) { return -1; }

    s_cfg[g_mit_motor_n] = *cfg;
    return (int8_t)g_mit_motor_n++;
}

/* MIT 运行 / 空闲 / 清零命令。 */
void mit_motor_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    mit_motor_send_cmd(hfdcan, id, data);
}

void mit_motor_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    mit_motor_send_cmd(hfdcan, id, data);
}

void mit_motor_zero(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    mit_motor_send_cmd(hfdcan, id, data);
}

/* MIT 控制命令。参数含义见 mit_motor.h。 */
void mit_motor_set_control(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                           float p_des, float v_des,
                           float kp, float kd, float t_ff)
{
    int8_t idx = mit_motor_find(id);
    const mit_motor_cfg_t *cfg;
    uint32_t p_int, v_int, kp_int, kd_int, t_int;
    uint8_t data[8] = {0};

    if (idx < 0) { return; }
    cfg = &s_cfg[idx];

    /* 应用方向符号（安装补偿） */
    p_des *= (float)cfg->sign;
    v_des *= (float)cfg->sign;
    t_ff  *= (float)cfg->sign;

    /* 限幅 */
    if (p_des < cfg->p_min) p_des = cfg->p_min;
    if (p_des > cfg->p_max) p_des = cfg->p_max;
    if (v_des < cfg->v_min) v_des = cfg->v_min;
    if (v_des > cfg->v_max) v_des = cfg->v_max;
    if (kp < cfg->kp_min)   kp = cfg->kp_min;
    if (kp > cfg->kp_max)   kp = cfg->kp_max;
    if (kd < cfg->kd_min)   kd = cfg->kd_min;
    if (kd > cfg->kd_max)   kd = cfg->kd_max;
    if (t_ff < cfg->t_min)  t_ff = cfg->t_min;
    if (t_ff > cfg->t_max)  t_ff = cfg->t_max;

    /* 量化 */
    p_int  = float_to_uint(p_des, cfg->p_min, cfg->p_max, 16);
    v_int  = float_to_uint(v_des, cfg->v_min, cfg->v_max, 12);
    kp_int = float_to_uint(kp,    cfg->kp_min, cfg->kp_max, 12);
    kd_int = float_to_uint(kd,    cfg->kd_min, cfg->kd_max, 12);
    t_int  = float_to_uint(t_ff,  cfg->t_min,  cfg->t_max,  12);

    /* 打包 MIT 帧 */
    data[0] = (uint8_t)((p_int >> 8) & 0xFF);
    data[1] = (uint8_t)(p_int & 0xFF);
    data[2] = (uint8_t)((v_int >> 4) & 0xFF);
    data[3] = (uint8_t)(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    data[4] = (uint8_t)(kp_int & 0xFF);
    data[5] = (uint8_t)((kd_int >> 4) & 0xFF);
    data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    data[7] = (uint8_t)(t_int & 0xFF);

    mit_motor_send_cmd(hfdcan, id, data);
}

/* 接收解析：只处理反馈 ID 属于已配置电机的帧。 */
void mit_motor_unpack(FDCAN_HandleTypeDef *hfdcan,
                      FDCAN_RxHeaderTypeDef *rx_header,
                      uint8_t *rx_data)
{
    int8_t idx;
    const mit_motor_cfg_t *cfg;
    uint16_t p_int, v_int, t_int;

    (void)hfdcan;

    if (rx_header->IdType != FDCAN_STANDARD_ID) { return; }

    /* 反馈帧第 0 字节为电机 ID */
    idx = mit_motor_find(rx_data[0]);
    if (idx < 0) { return; }
    cfg = &s_cfg[idx];

    p_int = (uint16_t)(((uint16_t)rx_data[1] << 8) | rx_data[2]);
    v_int = (uint16_t)(((uint16_t)rx_data[3] << 4) | ((rx_data[4] >> 4) & 0x0F));
    t_int = (uint16_t)(((uint16_t)(rx_data[4] & 0x0F) << 8) | rx_data[5]);

    mit_motor_state[idx].id         = cfg->id;
    mit_motor_state[idx].pos        = uint_to_float(p_int, cfg->p_min, cfg->p_max, 16) * (float)cfg->sign;
    mit_motor_state[idx].vel        = uint_to_float(v_int, cfg->v_min, cfg->v_max, 12) * (float)cfg->sign;
    mit_motor_state[idx].torque     = uint_to_float(t_int, cfg->t_min, cfg->t_max, 12) * (float)cfg->sign;
    mit_motor_state[idx].temp       = (int8_t)rx_data[6] - 40;
    mit_motor_state[idx].error      = rx_data[7];
    mit_motor_state[idx].last_rx_ms = HAL_GetTick();
    mit_motor_state[idx].online     = 1;
}

/* 便捷接口：按 ID 获取状态，找不到返回 NULL。 */
//电机状态查看函数
mit_motor_state_t *mit_motor_get_state(uint8_t id)
{
    int8_t idx = mit_motor_find(id);
    if (idx < 0) { return NULL; }
    return &mit_motor_state[idx];
}
