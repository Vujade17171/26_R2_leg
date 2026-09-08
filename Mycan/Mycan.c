#include "Mycan.h"
#include "ak_motor.h"          /* AK 电机驱动：提供 AK_Motor_RxDispatch */
extern FDCAN_HandleTypeDef hfdcan1;
uint8_t rx_data[8];
uint32_t rx_id = 0;
volatile uint32_t g_rx_frames = 0;   /* 收到的扩展帧计数（调试用） */

/* 电机句柄数组及其个数：在 Task/leg_task.c 中定义 */
extern AK_Motor motors[];
extern const uint8_t g_ak_motor_num;


		
void can_filter_init(void)
{
    FDCAN_FilterTypeDef can_filter_st;  // 定义FDCAN过滤器结构体

    // 配置过滤器参数：运控模式使用 11 位标准帧，ID=电机ID(默认1)
    can_filter_st.IdType = FDCAN_STANDARD_ID;       // 标准帧ID元素
    can_filter_st.FilterIndex = 0;                  // 过滤器索引（标准滤波器第 0 个）
    can_filter_st.FilterType = FDCAN_FILTER_MASK;   // 掩码模式：FilterID1=ID, FilterID2=掩码
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 命中报文存入 RX FIFO0

    // 掩码全 0 = 接收所有标准帧，软件按 std_id 匹配电机 ID 分发
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
        g_rx_frames++;                                // 收到帧计数+1
        Mycan_OnRxFrame(rx_id, rx_data, len);         // 分发到电机句柄
    }
}

/* ================================================================
 * 接收分发实现（覆盖头文件声明的钩子）：
 * 把收到的标准帧按 ID(=电机ID) 自动路由到 AK 电机句柄并解析。
 *  运控反馈 -> motors[i].pos_rad / vel_rads / torque_nm / temp_c / err_code
 * 在中断上下文执行，函数体保持轻量（不做打印/延时/OS 调用）。
 * ================================================================ */
void Mycan_OnRxFrame(uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    /* 内部按 std_id 匹配 motors[] 中对应 id 的句柄 */
    (void)AK_Motor_RxDispatch(motors, g_ak_motor_num, ext_id, data, len);
}
