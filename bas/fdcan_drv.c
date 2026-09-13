/**
  ******************************************************************************
  * @file    fdcan_drv.c
  * @brief   Universal FDCAN classic-CAN bus driver layer
  ******************************************************************************
  * This file is the only place that owns the FDCAN receive interrupt.
  * Motor drivers only register a callback and parse the frames for their IDs.
  ******************************************************************************
  */
#include "fdcan_drv.h"
#include <string.h>

#define FDCAN_DRV_MAX_RX_CB                 8U
#define FDCAN_DRV_RECOVER_PERIOD_MS        100U

#define FDCAN_DRV_RX_NOTIFY_ITS \
    (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | \
     FDCAN_IT_RX_FIFO0_FULL | \
     FDCAN_IT_RX_FIFO0_MESSAGE_LOST)

#define FDCAN_DRV_ERROR_NOTIFY_ITS \
    (FDCAN_IT_ERROR_WARNING | \
     FDCAN_IT_ERROR_PASSIVE | \
     FDCAN_IT_BUS_OFF)

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    fdcan_rx_cb_t        cb;
    uint8_t              used;
} fdcan_drv_node_t;

static fdcan_drv_node_t s_rx_cbs[FDCAN_DRV_MAX_RX_CB];
static volatile uint8_t s_bus_off_pending = 0U;
static uint32_t         s_last_recover_tick = 0U;

/* Debug counters. All are safe to watch in Keil. */
volatile uint32_t g_fdcan_rx_irq_cnt             = 0U;
volatile uint32_t g_fdcan_rx_frame_cnt           = 0U;
volatile uint32_t g_fdcan_rx_dispatched          = 0U;
volatile uint32_t g_fdcan_rx_fifo_full_cnt       = 0U;
volatile uint32_t g_fdcan_rx_fifo_lost_cnt       = 0U;
volatile uint32_t g_fdcan_rx_get_fail_cnt        = 0U;
volatile uint32_t g_fdcan_rx_no_cb_cnt           = 0U;
volatile uint32_t g_fdcan_rx_cb_overflow_cnt     = 0U;
volatile uint32_t g_fdcan_bus_off_cnt            = 0U;
volatile uint32_t g_fdcan_error_warning_cnt      = 0U;
volatile uint32_t g_fdcan_error_passive_cnt      = 0U;
volatile uint32_t g_fdcan_bus_off_recovered_cnt  = 0U;
volatile uint32_t g_fdcan_bus_off_recover_fail_cnt = 0U;
volatile uint32_t g_fdcan_tx_ok_cnt              = 0U;
volatile uint32_t g_fdcan_tx_full_cnt            = 0U;
volatile uint32_t g_fdcan_tx_fail_cnt            = 0U;
volatile uint32_t g_fdcan_last_error_its         = 0U;
volatile uint32_t g_fdcan_last_ecr               = 0U;
volatile uint32_t g_fdcan_last_psr               = 0U;
volatile uint32_t g_fdcan_last_hal_error         = 0U;
volatile uint32_t g_fdcan_last_state             = 0U;
volatile uint32_t g_fdcan_rx_fifo_level          = 0U;
volatile uint32_t g_fdcan_rx_last_id             = 0U;
volatile uint32_t g_fdcan_rx_last_id_type        = 0U;
volatile uint32_t g_fdcan_rx_last_dlc            = 0U;

/* Configure filters and interrupt sources. The peripheral must be in READY. */
static uint8_t fdcan_drv_configure(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL)
    {
        return 1U;
    }

    /* Standard classic-CAN frames go to FIFO0. */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x00000000U;
    filter.FilterID2    = 0x00000000U;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        return 1U;
    }

    /* Extended frames (EL05) also go to FIFO0. */
    filter.IdType       = FDCAN_EXTENDED_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x00000000U;
    filter.FilterID2    = 0x00000000U;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        return 1U;
    }

    /* Also accept non-matching frames in FIFO0. This makes the bus layer
       independent of motor IDs. Each motor callback does its own ID check. */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        return 1U;
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_DRV_RX_NOTIFY_ITS,
                                       0U) != HAL_OK)
    {
        return 1U;
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_DRV_ERROR_NOTIFY_ITS,
                                       0U) != HAL_OK)
    {
        return 1U;
    }

    return 0U;
}

/* Init, configure and start FDCAN. */
uint8_t fdcan_drv_init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL)
    {
        return 1U;
    }

    s_bus_off_pending   = 0U;
    s_last_recover_tick = HAL_GetTick();

    if (fdcan_drv_configure(hfdcan) != 0U)
    {
        return 1U;
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        return 1U;
    }

    g_fdcan_last_state = (uint32_t)HAL_FDCAN_GetState(hfdcan);
    return 0U;
}

/* Send one classic-CAN frame. */
uint8_t fdcan_drv_send(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t id, uint32_t id_type,
                       uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txh = {0};

    if ((hfdcan == NULL) || (data == NULL))
    {
        g_fdcan_tx_fail_cnt++;
        return 1U;
    }

    if (len > 8U)
    {
        len = 8U;
    }

    if (hfdcan->State != HAL_FDCAN_STATE_BUSY)
    {
        g_fdcan_tx_fail_cnt++;
        g_fdcan_last_hal_error = HAL_FDCAN_GetError(hfdcan);
        return 1U;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
    {
        g_fdcan_tx_full_cnt++;
        return 1U;
    }

    txh.Identifier          = id;
    txh.IdType              = (id_type == FDCAN_EXTENDED_ID) ?
                              FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    txh.TxFrameType         = FDCAN_DATA_FRAME;
    txh.DataLength          = (uint32_t)len;
    txh.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txh.BitRateSwitch       = FDCAN_BRS_OFF;
    txh.FDFormat            = FDCAN_CLASSIC_CAN;
    txh.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txh.MessageMarker       = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txh, data) != HAL_OK)
    {
        g_fdcan_tx_fail_cnt++;
        g_fdcan_last_hal_error = HAL_FDCAN_GetError(hfdcan);
        return 1U;
    }

    g_fdcan_tx_ok_cnt++;
    return 0U;
}

/* Register one receive callback. Duplicate registration is ignored. */
void fdcan_drv_reg_rx_cb(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_cb_t cb)
{
    uint32_t i;

    if ((hfdcan == NULL) || (cb == NULL))
    {
        g_fdcan_rx_cb_overflow_cnt++;
        return;
    }

    for (i = 0U; i < FDCAN_DRV_MAX_RX_CB; i++)
    {
        if ((s_rx_cbs[i].used != 0U) &&
            (s_rx_cbs[i].hfdcan == hfdcan) &&
            (s_rx_cbs[i].cb == cb))
        {
            return;
        }
    }

    for (i = 0U; i < FDCAN_DRV_MAX_RX_CB; i++)
    {
        if (s_rx_cbs[i].used == 0U)
        {
            s_rx_cbs[i].hfdcan = hfdcan;
            s_rx_cbs[i].cb     = cb;
            s_rx_cbs[i].used   = 1U; /* publish last, ISR may read it */
            return;
        }
    }

    g_fdcan_rx_cb_overflow_cnt++;
}

/* Main-loop service: update diagnostics and recover from Bus-Off outside ISR. */
void fdcan_drv_service(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t now;

    if (hfdcan == NULL)
    {
        return;
    }

    g_fdcan_last_state     = (uint32_t)HAL_FDCAN_GetState(hfdcan);
    g_fdcan_last_hal_error = HAL_FDCAN_GetError(hfdcan);
    g_fdcan_last_ecr       = hfdcan->Instance->ECR;
    g_fdcan_last_psr       = hfdcan->Instance->PSR;
    g_fdcan_rx_fifo_level  = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);

    now = HAL_GetTick();

    if ((g_fdcan_last_psr & FDCAN_PSR_BO) != 0U)
    {
        if ((uint32_t)(now - s_last_recover_tick) >= FDCAN_DRV_RECOVER_PERIOD_MS)
        {
            s_bus_off_pending = 1U;
        }
    }

    if (s_bus_off_pending == 0U)
    {
        return;
    }

    if ((uint32_t)(now - s_last_recover_tick) < FDCAN_DRV_RECOVER_PERIOD_MS)
    {
        return;
    }

    s_last_recover_tick = now;

    if (hfdcan->State == HAL_FDCAN_STATE_BUSY)
    {
        if (HAL_FDCAN_Stop(hfdcan) != HAL_OK)
        {
            g_fdcan_bus_off_recover_fail_cnt++;
            return;
        }
    }
    else if (hfdcan->State != HAL_FDCAN_STATE_READY)
    {
        g_fdcan_bus_off_recover_fail_cnt++;
        return;
    }

    if (fdcan_drv_configure(hfdcan) != 0U)
    {
        g_fdcan_bus_off_recover_fail_cnt++;
        return;
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        g_fdcan_bus_off_recover_fail_cnt++;
        return;
    }

    s_bus_off_pending = 0U;
    g_fdcan_bus_off_recovered_cnt++;
}

/* HAL weak callback override: drain FIFO0, then dispatch every frame. */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint32_t active;

    if (hfdcan == NULL)
    {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != 0U)
    {
        g_fdcan_rx_fifo_full_cnt++;
    }
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        g_fdcan_rx_fifo_lost_cnt++;
    }

    active = RxFifo0ITs & FDCAN_DRV_RX_NOTIFY_ITS;
    if (active == 0U)
    {
        return;
    }

    g_fdcan_rx_irq_cnt++;

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8] = {0};
        uint32_t i;
        uint8_t dispatched = 0U;

        if (HAL_FDCAN_GetRxMessage(hfdcan,
                                   FDCAN_RX_FIFO0,
                                   &rx_header,
                                   rx_data) != HAL_OK)
        {
            g_fdcan_rx_get_fail_cnt++;
            break;
        }

        g_fdcan_rx_frame_cnt++;
        g_fdcan_rx_last_id      = rx_header.Identifier;
        g_fdcan_rx_last_id_type = rx_header.IdType;
        g_fdcan_rx_last_dlc     = rx_header.DataLength;

        for (i = 0U; i < FDCAN_DRV_MAX_RX_CB; i++)
        {
            if ((s_rx_cbs[i].used != 0U) &&
                (s_rx_cbs[i].hfdcan == hfdcan) &&
                (s_rx_cbs[i].cb != NULL))
            {
                s_rx_cbs[i].cb(hfdcan, &rx_header, rx_data);
                g_fdcan_rx_dispatched++;
                dispatched = 1U;
            }
        }

        if (dispatched == 0U)
        {
            g_fdcan_rx_no_cb_cnt++;
        }
    }

    g_fdcan_rx_fifo_level = HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0);
}

/* HAL weak callback override: only record status. Recovery runs in service. */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
    if (hfdcan == NULL)
    {
        return;
    }

    g_fdcan_last_error_its = ErrorStatusITs;
    g_fdcan_last_ecr       = hfdcan->Instance->ECR;
    g_fdcan_last_psr       = hfdcan->Instance->PSR;

    if ((ErrorStatusITs & FDCAN_IT_ERROR_WARNING) != 0U)
    {
        g_fdcan_error_warning_cnt++;
    }

    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U)
    {
        g_fdcan_error_passive_cnt++;
    }

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
        g_fdcan_bus_off_cnt++;
        s_bus_off_pending = 1U;
    }
}