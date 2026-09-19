/**
  ******************************************************************************
  * @file    fdcan_drv.h
  * @brief   通用 FDCAN（经典 CAN）总线驱动层
  ******************************************************************************
  */
#ifndef __FDCAN_DRV_H
#define __FDCAN_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

typedef void (*fdcan_rx_cb_t)(FDCAN_HandleTypeDef *hfdcan,
                              FDCAN_RxHeaderTypeDef *rx_header,
                              uint8_t *rx_data);

/* FDCAN 驱动返回状态。 */
#define FDCAN_DRV_OK             0U
#define FDCAN_DRV_ERR_PARAM      1U
#define FDCAN_DRV_ERR_STATE      2U
#define FDCAN_DRV_ERR_TX_FULL    3U
#define FDCAN_DRV_ERR_TX         4U

/* 配置滤波器、使能接收中断并启动总线。 */
uint8_t fdcan_drv_init(FDCAN_HandleTypeDef *hfdcan);

/* 发送一个经典 CAN 帧。返回 FDCAN_DRV_* 状态码。 */
uint8_t fdcan_drv_send(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t id, uint32_t id_type,
                       uint8_t *data, uint8_t len);

/* 注册接收回调。同一回调不会重复注册。 */
void fdcan_drv_reg_rx_cb(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_DRV_H */
