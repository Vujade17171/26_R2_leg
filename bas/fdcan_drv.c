/**
  ******************************************************************************
  * @file    fdcan_drv.c
  * @brief   通用 FDCAN 经典 CAN 总线驱动层
  ******************************************************************************
  * 本文件是唯一负责 FDCAN 接收中断的位置。
  * 电机驱动只注册回调，并按各自 ID 解析帧。
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

/* 配置滤波器和中断源。外设必须处于 READY 状态。 */
static uint8_t fdcan_drv_configure(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == NULL)
    {
        return 1U;
    }

    /* 标准经典 CAN 帧进入 FIFO0。 */
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

    /* 扩展帧（EL05）也进入 FIFO0。 */
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

    /* 同时在 FIFO0 接收不匹配的帧。这样总线层
       与电机 ID 解耦，各电机回调自行检查 ID。 */
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

/* 初始化、配置并启动 FDCAN。 */
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
    return 0U;
}

/* 发送一个经典 CAN 帧。 */
uint8_t fdcan_drv_send(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t id, uint32_t id_type,
                       uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef txh = {0};

    if ((hfdcan == NULL) || (data == NULL))
    {
        return 1U;
    }

    if (len > 8U)
    {
        len = 8U;
    }

    if (hfdcan->State != HAL_FDCAN_STATE_BUSY)
    {
        return 1U;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
    {
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
        return 1U;
    }
    return 0U;
}

/* 注册一个接收回调。重复注册会被忽略。 */
void fdcan_drv_reg_rx_cb(FDCAN_HandleTypeDef *hfdcan, fdcan_rx_cb_t cb)
{
    uint32_t i;

    if ((hfdcan == NULL) || (cb == NULL))
    {
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
            s_rx_cbs[i].used   = 1U; /* 最后发布，ISR 可能读取它 */
            return;
        }
    }
}

/* 主循环服务：在 ISR 外恢复 Bus-Off。 */
void fdcan_drv_service(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t now;

    if (hfdcan == NULL)
    {
        return;
    }

    now = HAL_GetTick();

    if ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U)
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
            return;
        }
    }
    else if (hfdcan->State != HAL_FDCAN_STATE_READY)
    {
        return;
    }

    if (fdcan_drv_configure(hfdcan) != 0U)
    {
        return;
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        return;
    }

    s_bus_off_pending = 0U;
}

/* HAL 弱回调重写：清空 FIFO0，然后分发每一帧。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint32_t active;

    if (hfdcan == NULL)
    {
        return;
    }

    active = RxFifo0ITs & FDCAN_DRV_RX_NOTIFY_ITS;
    if (active == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8] = {0};
        uint32_t i;
        if (HAL_FDCAN_GetRxMessage(hfdcan,
                                   FDCAN_RX_FIFO0,
                                   &rx_header,
                                   rx_data) != HAL_OK)
        {
            break;
        }

        for (i = 0U; i < FDCAN_DRV_MAX_RX_CB; i++)
        {
            if ((s_rx_cbs[i].used != 0U) &&
                (s_rx_cbs[i].hfdcan == hfdcan) &&
                (s_rx_cbs[i].cb != NULL))
            {
                s_rx_cbs[i].cb(hfdcan, &rx_header, rx_data);
            }
        }
    }
}

/* HAL 弱回调重写：标记 Bus-Off；恢复在主循环服务中执行。 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
    if (hfdcan == NULL)
    {
        return;
    }

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
        s_bus_off_pending = 1U;
    }
}
