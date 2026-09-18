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

/* 配置滤波器、使能接收/错误中断并启动总线。 */
uint8_t fdcan_drv_init(FDCAN_HandleTypeDef *hfdcan);

/* 发送一个经典 CAN 帧。非零表示该帧未能入队。 */
uint8_t fdcan_drv_send(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t id, uint32_t id_type,
                       uint8_t *data, uint8_t len);

/* 在主循环中周期性调用（建议 10 ms）。
   它在 ISR 外执行 Bus-Off 恢复。 */
void fdcan_drv_service(FDCAN_HandleTypeDef *hfdcan);

/* 注册接收回调。同一回调不会重复注册。 */
void fdcan_drv_reg_rx_cb(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_cb_t cb);


#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_DRV_H */
