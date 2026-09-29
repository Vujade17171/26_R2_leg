/*****************************************************************************
 * bsp_can.c —— FDCAN 板级驱动实现（过滤器 / 收发原语 / 上行注册分发）
 *
 * 与 CubeMX 的分工：
 *   Core/Src/fdcan.c 负责 MX_FDCAN1_Init()（时钟 / GPIO / 位时序）；
 *   本文件负责过滤器、启动、接收中断与收发原语。
 *****************************************************************************/
#include "bsp_can.h"

#include "fdcan.h"   /* hfdcan1、FDCAN_* HAL 定义 */
#include <stddef.h>

/* 总线编号 -> HAL 句柄（HAL 类型只在本文件内出现） */
static FDCAN_HandleTypeDef *const g_bus[BSP_CAN_BUS_NUM] = { &hfdcan1, &hfdcan2 };

/* 上行接收者（由中断读取） */
static volatile BSP_CAN_RxHandler g_rx_handler[BSP_CAN_BUS_NUM];

/* 统计与调试观测 */
static volatile uint32_t g_rx_frames[BSP_CAN_BUS_NUM];
static volatile uint32_t g_last_id[BSP_CAN_BUS_NUM];
static volatile uint8_t  g_last_data[BSP_CAN_BUS_NUM][8];

static int32_t bus_ok(bsp_can_bus_t bus)
{
    return (bus < (bsp_can_bus_t)BSP_CAN_BUS_NUM) ? SER_OK : SER_ERR_PARAM;
}

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */
int32_t BSP_CAN_Init(bsp_can_bus_t bus)
{
    FDCAN_FilterTypeDef f = {0};
    FDCAN_HandleTypeDef *h;

    if (bus_ok(bus) != SER_OK) { return SER_ERR_PARAM; }
    h = g_bus[bus];

    /* 标准帧全收：AK 电机反馈，ID = 电机ID */
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0u;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x00000000u;   /* 掩码为 0 时不影响匹配结果 */
    f.FilterID2    = 0x00000000u;   /* 掩码全 0：全部接收 */
    if (HAL_FDCAN_ConfigFilter(h, &f) != HAL_OK) { return SER_ERR_BUS; }

    /* 扩展帧全收：EL05 私有协议反馈（需 ExtFiltersNbr >= 1） */
    f.IdType       = FDCAN_EXTENDED_ID;
    f.FilterIndex  = 0u;
    if (HAL_FDCAN_ConfigFilter(h, &f) != HAL_OK) { return SER_ERR_BUS; }

    /* FDCAN 必须配置全局滤波器：不匹配即拒绝，否则上面的过滤器不生效 */
    if (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        return SER_ERR_BUS;
    }

    if (HAL_FDCAN_Start(h) != HAL_OK) { return SER_ERR_BUS; }
    if (HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u) != HAL_OK) {
        return SER_ERR_BUS;
    }
    return SER_OK;
}

/* ---------------------------------------------------------------------------
 * 上行：注册 / 注销
 * ------------------------------------------------------------------------- */
int32_t BSP_CAN_RegisterRx(bsp_can_bus_t bus, BSP_CAN_RxHandler handler)
{
    if (bus_ok(bus) != SER_OK) { return SER_ERR_PARAM; }
    if (handler == 0) { return SER_ERR_PARAM; }
    g_rx_handler[bus] = handler;
    return SER_OK;
}

int32_t BSP_CAN_UnregisterRx(bsp_can_bus_t bus)
{
    if (bus_ok(bus) != SER_OK) { return SER_ERR_PARAM; }
    g_rx_handler[bus] = 0;
    return SER_OK;
}

/* ---------------------------------------------------------------------------
 * 下行：发送
 * ------------------------------------------------------------------------- */
static int32_t can_send(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                        const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    FDCAN_HandleTypeDef *h;
    uint8_t buf[8];
    uint8_t i;
    uint8_t is_ext;

    if (bus_ok(bus) != SER_OK) { return SER_ERR_PARAM; }
    if ((data == 0) || (len > 8u)) { return SER_ERR_PARAM; }

    h      = g_bus[bus];
    is_ext = (id_type == (uint32_t)BSP_CAN_ID_EXT) ? 1u : 0u;

    for (i = 0u; i < 8u; i++) { buf[i] = (i < len) ? data[i] : 0u; }

    tx.Identifier          = is_ext ? (id & 0x1FFFFFFFu) : (id & 0x7FFu);
    tx.IdType              = is_ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = (uint32_t)len;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;   /* 经典 CAN，非 FD */

    if (HAL_FDCAN_AddMessageToTxFifoQ(h, &tx, buf) != HAL_OK) { return SER_ERR_BUS; }
    return SER_OK;
}

int32_t vesc_can_send(bsp_can_bus_t bus, uint32_t id, 
                        const uint8_t *data, uint8_t len){

    FDCAN_HandleTypeDef *h;
    FDCAN_TxHeaderTypeDef tx_header;
    // 注意：FDCAN 的发送确认通常不需要单独的 box 变量，或者需要 FDCAN_TxEventFifoTypeDef
    h = g_bus[bus];
    // VESC 使用扩展帧
    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_EXTENDED_ID; // 对应原来的 CAN_ID_EXT
    tx_header.TxFrameType = FDCAN_DATA_FRAME; // 对应原来的 CAN_RTR_DATA
    tx_header.DataLength = (uint8_t)len; // 对应原来的 DLC = 8
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;   // 经典 CAN 模式
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(h, &tx_header, data) != HAL_OK) { return SER_ERR_BUS; }
    return SER_OK;
}


int32_t BSP_CAN_SendStd(bsp_can_bus_t bus, uint32_t std_id,
                        const uint8_t *data, uint8_t len)
{
    return can_send(bus, std_id, (uint32_t)BSP_CAN_ID_STD, data, len);
}

int32_t BSP_CAN_SendExt(bsp_can_bus_t bus, uint32_t ext_id,
                        const uint8_t *data, uint8_t len)
{
    return can_send(bus, ext_id, (uint32_t)BSP_CAN_ID_EXT, data, len);
}

/* ---------------------------------------------------------------------------
 * 统计 / 观测
 * ------------------------------------------------------------------------- */
uint32_t BSP_CAN_GetRxFrameCount(bsp_can_bus_t bus)
{
    if (bus_ok(bus) != SER_OK) { return 0u; }
    return g_rx_frames[bus];
}

void BSP_CAN_GetLastRx(bsp_can_bus_t bus, uint32_t *id, uint8_t *data8)
{
    uint8_t i;

    if (bus_ok(bus) != SER_OK) { return; }
    if (id != 0) { *id = g_last_id[bus]; }
    if (data8 != 0) {
        for (i = 0u; i < 8u; i++) { data8[i] = g_last_data[bus][i]; }
    }
}

/* ---------------------------------------------------------------------------
 * HAL 接收回调（中断上下文）：取帧 -> 统计 -> 交给已注册的接收者
 * 保持轻量：只做取帧与分发，不打印、不延时、不调用 OS API。
 * ------------------------------------------------------------------------- */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx = {0};
    BSP_CAN_RxHandler handler;
    uint8_t data[8] = {0};
    uint32_t id_type;
    uint8_t len;
    uint8_t i;
    uint8_t bus;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) { return; }

    /* 反查总线编号：bsp 内部事务，不暴露给上层 */
    for (bus = 0u; bus < (uint8_t)BSP_CAN_BUS_NUM; bus++) {
        if (g_bus[bus] == hfdcan) { break; }
    }
    if (bus >= (uint8_t)BSP_CAN_BUS_NUM) { return; }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) { return; }

    len = (uint8_t)(rx.DataLength & 0x0Fu);   /* 经典帧数据长度 0~8 */
    if (len > 8u) { len = 8u; }

    g_rx_frames[bus]++;
    g_last_id[bus] = rx.Identifier;
    for (i = 0u; i < 8u; i++) { g_last_data[bus][i] = data[i]; }

    handler = g_rx_handler[bus];
    if (handler != 0) {
        id_type = (rx.IdType == FDCAN_EXTENDED_ID) ? (uint32_t)BSP_CAN_ID_EXT
                                                   : (uint32_t)BSP_CAN_ID_STD;
        handler((bsp_can_bus_t)bus, rx.Identifier, id_type, data, len);
    }
}

