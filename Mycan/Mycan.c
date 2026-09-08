#include "Mycan.h"
extern FDCAN_HandleTypeDef hfdcan1;
uint8_t rx_data[8];
uint32_t rx_id = 0;


		
void can_filter_init(void)
{
    FDCAN_FilterTypeDef can_filter_st;  // 定义FDCAN过滤器结构体

    // 配置过滤器参数：AK 系列电机协议使用 29 位扩展帧
    can_filter_st.IdType = FDCAN_EXTENDED_ID;       // 扩展帧ID元素
    can_filter_st.FilterIndex = 0;                  // 过滤器索引（扩展滤波器第 0 个）
    can_filter_st.FilterType = FDCAN_FILTER_MASK;   // 掩码模式：FilterID1=ID, FilterID2=掩码
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 命中报文存入 RX FIFO0

    // 掩码全 0 = 接收所有扩展帧（反馈 ID 0x29xx/0x2Axx 由软件按 ID>>8 分发）
    can_filter_st.FilterID1 = 0x00000000u;          // 待匹配 ID（掩码为 0 时不影响结果）
    can_filter_st.FilterID2 = 0x00000000u;          // 掩码全 0：全部接收
    
    // 配置过滤器
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_st) != HAL_OK)
    {
        Error_Handler();
    }
    
    // ★★★ FDCAN必须额外配置全局滤波器 ★★★
    // 拒绝所有不匹配的报文，使上述过滤器生效
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,         // 标准帧不匹配时拒绝
                                     FDCAN_REJECT,         // 扩展帧不匹配时拒绝
                                     FDCAN_REJECT_REMOTE,  // 拒收远程帧(标准)
                                     FDCAN_REJECT_REMOTE) != HAL_OK) // 拒收远程帧(扩展)
    {
        Error_Handler();
    }
    
    // 启动FDCAN（对应原HAL_CAN_Start）
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    
    // 激活接收中断（对应原HAL_CAN_ActivateNotification）
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,  // FIFO0有新消息中断
                                       0) != HAL_OK)
    {
        Error_Handler();
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header = {0};   /* FDCAN接收头 */
    uint8_t len;

    // 检查是否是FIFO0有新消息的中断
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
    {
        return;
    }

    // 从FIFO0接收消息
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        rx_id = rx_header.Identifier;                 // 29位扩展ID
        len = (uint8_t)(rx_header.DataLength & 0x0Fu); // 经典帧数据长度 0~8
        if (len > 8u)
        {
            len = 8u;
        }
        Mycan_OnRxFrame(rx_id, rx_data, len);         // 分发（弱钩子，用户可覆盖）
    }
}

/* 接收分发弱钩子：默认空实现。用户定义同名强函数覆盖即可，
 * 例如在其内按 (ext_id >> 8) 匹配 0x29/0x2A，再调用
 * AK_Motor_RxDispatch(&motors[0], N, ext_id, data, len)。 */
#if defined(__GNUC__)
__attribute__((weak))
#elif defined(__CC_ARM)
__weak
#endif
void Mycan_OnRxFrame(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    (void)ext_id;
    (void)data;
    (void)len;
}
