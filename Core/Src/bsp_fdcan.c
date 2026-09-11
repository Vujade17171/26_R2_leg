#include "bsp_fdcan.h"
#include "Joint_EL05.h"
uint8_t CAN_Rxbuf[8];
HAL_StatusTypeDef a;
AK_Handle_t g_ak80 = {0};
AK_Handle_t g_ak45 = {0};

//函数1：CAN过滤器（全局"全收"：三个电机共用一套）
void can_filter_init(void)
{
    /* 用全局过滤器"全收"：标准帧（AK80/AK45）和扩展帧（EL05）都直接进 RX FIFO0，
     * 不再依赖单独的过滤器元素，最简洁也最不容易出错。 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,   /* 非匹配标准帧 → RX FIFO0 */
        FDCAN_ACCEPT_IN_RX_FIFO0,   /* 非匹配扩展帧 → RX FIFO0 */
        FDCAN_REJECT_REMOTE,        /* 拒收远程标准帧 */
        FDCAN_REJECT_REMOTE);       /* 拒收远程扩展帧 */

    HAL_FDCAN_Start(&hfdcan1);//使能can通信
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能接收完成中断
	//    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter_st);
	//    HAL_FDCAN_Start(&hfdcan2);
	//    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
//函数2：数据发送函数AK80
HAL_StatusTypeDef can_send_data(AK_Handle_t*ak,uint32_t cob_id, uint8_t *data, uint8_t len)
{
    if (ak->pcan_handle == NULL) {
        return HAL_ERROR;
    }
    
    if (len > 8) len = 8;
    if (len == 0) return HAL_ERROR;
    
    FDCAN_TxHeaderTypeDef txHeader;
    
    txHeader.Identifier          = cob_id;                    /* 标准ID */
    txHeader.IdType              = FDCAN_STANDARD_ID;         /* 标准帧 */
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;          /* 数据帧 */
    txHeader.DataLength          = len;                       /* 数据长度 */
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;          /* 错误状态指示 */
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;             /* 波特率切换关闭 */
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;         /* 经典CAN模式 */
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;        /* 不存储发送事件 */
    txHeader.MessageMarker       = 0;                         /* 消息标记 */
    
    return HAL_FDCAN_AddMessageToTxFifoQ(ak->pcan_handle, &txHeader, data);
}
//函数3：数据发送函数（扩展帧，EL05）
HAL_StatusTypeDef can_send_ext_data(FDCAN_HandleTypeDef *pcan, uint32_t ext_id, uint8_t *data, uint8_t len)
{
    if (pcan == NULL) {
        return HAL_ERROR;
    }

    if (len > 8) len = 8;
    if (len == 0) return HAL_ERROR;

    FDCAN_TxHeaderTypeDef txHeader;

    txHeader.Identifier          = ext_id;                    /* 扩展ID */
    txHeader.IdType              = FDCAN_EXTENDED_ID;         /* 扩展帧 */
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;          /* 数据帧 */
    txHeader.DataLength          = len;                       /* 数据长度 */
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;          /* 错误状态指示 */
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;             /* 波特率切换关闭 */
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;         /* 经典CAN模式 */
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;        /* 不存储发送事件 */
    txHeader.MessageMarker       = 0;                         /* 消息标记 */

    return a=HAL_FDCAN_AddMessageToTxFifoQ(pcan, &txHeader, data);
}
//函数4：FDCAN接收回调 - 处理电机反馈数据AK80
void AK_Motion_FDCAN_RxCallback(AK_Handle_t*ak,FDCAN_HandleTypeDef *pcan)
{
    if (pcan != ak->pcan_handle) {
        return;
    }
    
    if (!ak->is_initialized) {
        return;
    }
    
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rx_data[8];
    
    if (HAL_FDCAN_GetRxMessage(pcan, FDCAN_RX_FIFO0, &rxHeader, rx_data) != HAL_OK) {
        return;
    }
    parse_motion_feedback(ak,rx_data);
}

/* 重写 HAL weak 接收回调：统一取帧，按帧类型分发到对应电机 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rx_data[8];

    /* 一次中断可能堆积多帧（三个电机都在回传），必须循环读空 FIFO。
     * 若每次只读 1 帧，三个电机 1kHz 回传时 FIFO 会溢出丢帧，
     * EL05 的扩展帧优先级最低、最先被丢掉，表现为"EL05 失联"。 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rx_data) != HAL_OK) {
            break;
        }

        if (rxHeader.IdType == FDCAN_EXTENDED_ID) {
            /* 扩展帧 → EL05 */
            EL05_Parse_Feedback(&g_el05, rxHeader.Identifier, rx_data);
        } else {
            /* 标准帧 → AK 系列，按 CAN ID 分发 */
            if (rxHeader.Identifier == g_ak80.motor_id) {
                parse_motion_feedback(&g_ak80, rx_data);
            } else if (rxHeader.Identifier == g_ak45.motor_id) {
                parse_motion_feedback(&g_ak45, rx_data);
            }
        }
    }
}
