/**
  ******************************************************************************
  * @file    fdcan_drv.c
  * @brief   Universal FDCAN (classic CAN) bus driver layer
  ******************************************************************************
  */
#include "fdcan_drv.h"
#include <string.h>

#define FDCAN_DRV_MAX_RX_CB   8   /* 接收回调函数数量 */

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    fdcan_rx_cb_t        cb;
    uint8_t              used;
} fdcan_drv_node_t;

static fdcan_drv_node_t s_rx_cbs[FDCAN_DRV_MAX_RX_CB];

/* debug counters (exposed for Keil debugger / diagnosis) */
volatile uint32_t g_fdcan_rx_irq_cnt    = 0;
volatile uint32_t g_fdcan_rx_frame_cnt  = 0;
volatile uint32_t g_fdcan_rx_dispatched = 0;

/* Init and start FDCAN */
uint8_t fdcan_drv_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    /* Standard frame filter: full match -> FIFO0 */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x00000000;
    filter.FilterID2    = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        return 1;
    }

    /* Extended frame filter: full match -> FIFO0 */
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x00000000;
    filter.FilterID2    = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        return 1;
    }

    /* Global filter: accept all frames into FIFO0 */
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    /* Enable FIFO0 new-message interrupt */
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        return 1;
    }

    /* Start FDCAN */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        return 1;
    }

    return 0;
}

/* Send one frame */
uint8_t fdcan_drv_send(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t id, uint8_t id_type,
                       uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txh = {0};

    if (len > 8) { len = 8; }

    txh.Identifier           = id;
    txh.IdType               = (id_type == FDCAN_EXTENDED_ID) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    txh.TxFrameType          = FDCAN_DATA_FRAME;
    txh.DataLength           = (uint32_t)len;
    txh.ErrorStateIndicator  = FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch        = FDCAN_BRS_OFF;
    txh.FDFormat             = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl   = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker        = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, data) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/* Register an rx callback */
//相当于注册表的作用
void fdcan_drv_reg_rx_cb(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_cb_t cb)
{
    uint32_t i;

    for (i = 0; i < FDCAN_DRV_MAX_RX_CB; i++)
    {
        if (s_rx_cbs[i].used == 0)
        {
            s_rx_cbs[i].hfdcan = hfdcan;
            s_rx_cbs[i].cb     = cb;
            s_rx_cbs[i].used   = 1;
            return;
        }
    }
}

/* Override the HAL weak rx interrupt callback */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
    {
        return;
    }

    g_fdcan_rx_irq_cnt++;

    /* Drain FIFO0 fully and dispatch to every registered driver */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        uint32_t i;

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        g_fdcan_rx_frame_cnt++;

        for (i = 0; i < FDCAN_DRV_MAX_RX_CB; i++)
        {
            if (s_rx_cbs[i].used && (s_rx_cbs[i].hfdcan == hfdcan) && (s_rx_cbs[i].cb != NULL))
            {
                s_rx_cbs[i].cb(hfdcan, &rx_header, rx_data);
                g_fdcan_rx_dispatched++;
            }
        }
    }
}
