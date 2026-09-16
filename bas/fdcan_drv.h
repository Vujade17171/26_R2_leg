/**
  ******************************************************************************
  * @file    fdcan_drv.h
  * @brief   Universal FDCAN (classic CAN) bus driver layer
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

/* Configure filters, enable Rx/error interrupts and start the bus. */
uint8_t fdcan_drv_init(FDCAN_HandleTypeDef *hfdcan);

/* Send one classic-CAN frame. Non-zero means the frame was not queued. */
uint8_t fdcan_drv_send(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t id, uint32_t id_type,
                       uint8_t *data, uint8_t len);

/* Register a receive callback. The same callback is never registered twice. */
void fdcan_drv_reg_rx_cb(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_cb_t cb);


#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_DRV_H */
