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

/* Call periodically from the main loop (10 ms is recommended).
   It updates diagnostics and performs Bus-Off recovery outside the ISR. */
void fdcan_drv_service(FDCAN_HandleTypeDef *hfdcan);

/* Register a receive callback. The same callback is never registered twice. */
void fdcan_drv_reg_rx_cb(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_cb_t cb);

/* ---- debug counters: watch these in Keil ---- */
extern volatile uint32_t g_fdcan_rx_irq_cnt;
extern volatile uint32_t g_fdcan_rx_frame_cnt;
extern volatile uint32_t g_fdcan_rx_dispatched;
extern volatile uint32_t g_fdcan_rx_fifo_full_cnt;
extern volatile uint32_t g_fdcan_rx_fifo_lost_cnt;
extern volatile uint32_t g_fdcan_rx_get_fail_cnt;
extern volatile uint32_t g_fdcan_rx_no_cb_cnt;
extern volatile uint32_t g_fdcan_rx_cb_overflow_cnt;
extern volatile uint32_t g_fdcan_bus_off_cnt;
extern volatile uint32_t g_fdcan_error_warning_cnt;
extern volatile uint32_t g_fdcan_error_passive_cnt;
extern volatile uint32_t g_fdcan_bus_off_recovered_cnt;
extern volatile uint32_t g_fdcan_bus_off_recover_fail_cnt;
extern volatile uint32_t g_fdcan_tx_ok_cnt;
extern volatile uint32_t g_fdcan_tx_full_cnt;
extern volatile uint32_t g_fdcan_tx_fail_cnt;
extern volatile uint32_t g_fdcan_last_error_its;
extern volatile uint32_t g_fdcan_last_ecr;
extern volatile uint32_t g_fdcan_last_psr;
extern volatile uint32_t g_fdcan_last_hal_error;
extern volatile uint32_t g_fdcan_last_state;
extern volatile uint32_t g_fdcan_rx_fifo_level;
extern volatile uint32_t g_fdcan_rx_last_id;
extern volatile uint32_t g_fdcan_rx_last_id_type;
extern volatile uint32_t g_fdcan_rx_last_dlc;

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_DRV_H */