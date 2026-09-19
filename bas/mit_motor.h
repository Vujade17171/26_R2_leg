/**
  ******************************************************************************
  * @file    mit_motor.h
  * @brief   通用 MIT 模式电机驱动（AK80-9 / AK45-10 / ...）
  ******************************************************************************
  * 说明：
  *   - 一个驱动处理同一 FDCAN 总线上的所有 MIT 模式电机。
  *   - 使用配置表（手册 Table 30），新增电机只需增加一行。
  *   - 只解析 ID 属于已配置电机的反馈帧。
  ******************************************************************************
  */
#ifndef __MIT_MOTOR_H
#define __MIT_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define MIT_MOTOR_MAX_NUM   4     /* 支持的最大 MIT 电机数量 */

/* ---- 每个电机一行配置 ---- */
//cfg配置的意思：电机配置参数
typedef struct
{
    uint8_t  id;             /* CAN ID 等于电机 ID */
    float    p_min, p_max;   /* 位置限幅，单位 rad */
    float    v_min, v_max;   /* 速度限幅，单位 rad/s */
    float    t_min, t_max;   /* 力矩限幅，单位 Nm */
    float    kp_min, kp_max; /* Kp 限幅 */
    float    kd_min, kd_max; /* Kd 限幅 */
    int8_t   sign;           /* +1 正向，-1 反向 */
} mit_motor_cfg_t;

/* ---- 每个电机的实时状态 ---- */
typedef struct
{
    float    pos;         /* 位置，单位 rad */
    float    vel;         /* 速度，单位 rad/s */
    float    torque;      /* 力矩，单位 Nm */
    int16_t  temp;        /* 温度，单位摄氏度 */
    uint8_t  error;       /* 错误码 */
    uint32_t last_rx_ms;  /* 最近一次反馈时间，单位 ms */
    uint8_t  online;      /* 在线标志 */
    uint8_t  id;          /* 电机 ID */
} mit_motor_state_t;

extern mit_motor_state_t mit_motor_state[MIT_MOTOR_MAX_NUM];
extern uint32_t          g_mit_motor_n;   /* 已配置电机数量 */

/* 初始化：在总线上注册唯一的接收回调。 */
uint8_t mit_motor_init(FDCAN_HandleTypeDef *hfdcan);

/* 添加一个电机配置。返回索引 0..n-1；已满或无效时返回 -1。 */
int8_t mit_motor_add(const mit_motor_cfg_t *cfg);

/* 根据 ID 查找电机索引；找不到返回 -1。 */
int8_t mit_motor_find(uint8_t id);

/* MIT 运行 / 空闲命令。 */
void mit_motor_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void mit_motor_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);

/* MIT 控制命令：
 *   p_des : 目标位置，单位 rad
 *   v_des : 目标速度，单位 rad/s
 *   kp    : 位置增益
 *   kd    : 速度增益
 *   t_ff  : 前馈力矩，单位 Nm
 */
uint8_t mit_motor_set_control(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                              float p_des, float v_des,
                              float kp, float kd, float t_ff);

/* 接收解析（由总线层回调调用）。只处理已配置的 MIT 电机 ID。 */
void mit_motor_unpack(FDCAN_HandleTypeDef *hfdcan,
                      FDCAN_RxHeaderTypeDef *rx_header,
                      uint8_t *rx_data);

/* 便捷接口：按 ID 获取状态，找不到返回 NULL。 */
mit_motor_state_t *mit_motor_get_state(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __MIT_MOTOR_H */
