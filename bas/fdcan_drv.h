/**
  ******************************************************************************
  * @file    fdcan_drv.h
  * @brief   Universal FDCAN (classic CAN) bus driver layer
  ******************************************************************************
  * Description:
  *   - Hide low-level bus details, expose only "send / register rx callback".
  *   - Reusable for other motors (DaMiao, ROBSTRIDE, etc.) later.
  ******************************************************************************
  */
#ifndef __FDCAN_DRV_H
#define __FDCAN_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* Rx callback type
 *   hfdcan    : FDCAN handle that triggered interrupt
 *   rx_header : received frame header
 *   rx_data   : 8-byte payload
 */
typedef void (*fdcan_rx_cb_t)(FDCAN_HandleTypeDef *hfdcan,
                              FDCAN_RxHeaderTypeDef *rx_header,
                              uint8_t *rx_data);

/* Init and start FDCAN (configure filters, enable FIFO0 rx notification)
 * Return 0 success, non-zero fail.
 */
uint8_t fdcan_drv_init(FDCAN_HandleTypeDef *hfdcan);

/* Send one classic CAN frame
 *   hfdcan  : FDCAN handle
 *   id      : identifier
 *   id_type : FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
 *   data    : payload pointer
 *   len     : length (<=8)
 * Return 0 success, non-zero fail.
 */
uint8_t fdcan_drv_send(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t id, uint8_t id_type,
                       uint8_t *data, uint8_t len);

/* Register an rx callback (multiple drivers may register on the same bus) */
void fdcan_drv_reg_rx_cb(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_cb_t cb);

/* ---- debug counters (watch these in Keil to diagnose Rx path) ---- */
extern volatile uint32_t g_fdcan_rx_irq_cnt;    /* Rx fifo callback entered (notify flag set) */
extern volatile uint32_t g_fdcan_rx_frame_cnt;  /* frames actually read out of FIFO0 */
extern volatile uint32_t g_fdcan_rx_dispatched; /* frames dispatched to a registered callback */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_DRV_H */
