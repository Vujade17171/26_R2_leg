/**
  ******************************************************************************
  * @file    robstride.h
  * @brief   RobStride（灵足-05）电机驱动 —— CAN 2.0 扩展帧
  ******************************************************************************
  * 说明：
  *   - 腕关节电机“灵足-05”，使用 RobStride 私有协议。
  *   - 29 位扩展 ID：[com_type(5b)][mid/torque(16b)][motor_id(8b)]。
  *   - 注意：控制帧（0x01）的目标力矩位于
  *           ID 的第 23..8 位（不在数据区）。数据区为 angle/speed/kp/kd。
  ******************************************************************************
  */
#ifndef __ROBSTRIDE_H
#define __ROBSTRIDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define ROBSTRIDE_MAX_NUM   4     /* 支持的最大腕关节电机数量 */

/* ---- 每个电机一行配置 ---- */
typedef struct
{
    uint8_t  id;            /* 电机 CAN ID（0x00..0x7F） */
    float    p_min, p_max;  /* 位置限幅，单位 rad */
    float    v_min, v_max;  /* 速度限幅，单位 rad/s */
    float    t_min, t_max;  /* 力矩限幅，单位 Nm */
    float    kp_min, kp_max;/* Kp 限幅 */
    float    kd_min, kd_max;/* Kd 限幅 */
    int8_t   sign;          /* +1 正向，-1 反向 */
    uint16_t master_id;     /* 主机 CAN ID（第 15..8 位） */
} robstride_cfg_t;

/* ---- 每个电机的实时状态 ---- */
typedef struct
{
    float    angle;      /* 位置，单位 rad */
    float    speed;      /* 速度，单位 rad/s */
    float    torque;     /* 力矩，单位 Nm */
    float    temp;       /* 温度，单位摄氏度（原始值乘 0.1） */
    uint8_t  error;      /* 错误码（来自 ID 第 21..16 位） */
    uint8_t  pattern;    /* 运行模式标志（来自 ID 第 23..22 位） */
    uint32_t last_rx_ms; /* 最近一次反馈时间，单位 ms */
    uint8_t  online;     /* 在线标志 */
    uint8_t  id;         /* 电机 ID */
} robstride_state_t;

extern robstride_state_t robstride_state[ROBSTRIDE_MAX_NUM];
extern uint32_t          g_robstride_n;   /* 已配置电机数量 */

/* 初始化：在总线上注册唯一的接收回调。 */
uint8_t robstride_init(FDCAN_HandleTypeDef *hfdcan);

/* 添加一个电机配置。返回索引 0..n-1；已满或无效时返回 -1。 */
int8_t robstride_add(const robstride_cfg_t *cfg);

/* 根据 ID 查找电机索引；找不到返回 -1。 */
int8_t robstride_find(uint8_t id);

/* 使能（0x03）/ 停止（0x04）/ 清零（0x06）。 */
void robstride_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id);
void robstride_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t clear_error);
void robstride_zero(FDCAN_HandleTypeDef *hfdcan, uint8_t id);

/* 控制命令（0x01）：
 *   torque : 目标力矩，单位 Nm        -> 放在 ID 第 23..8 位
 *   angle  : 目标位置，单位 rad
 *   speed  : 目标速度，单位 rad/s
 *   kp     : 位置增益
 *   kd     : 速度增益
 */
void robstride_set_control(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                           float torque, float angle,
                           float speed, float kp, float kd);

/* 接收解析：只处理 com_type==2（电机反馈）。 */
void robstride_unpack(FDCAN_HandleTypeDef *hfdcan,
                      FDCAN_RxHeaderTypeDef *rx_header,
                      uint8_t *rx_data);

/* 便捷接口：按 ID 获取状态，找不到返回 NULL。 */
robstride_state_t *robstride_get_state(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __ROBSTRIDE_H */
