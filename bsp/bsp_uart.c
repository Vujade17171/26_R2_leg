/*****************************************************************************
 * bsp_uart.c —— UART 板级驱动实现（单缓冲 + 环形 DMA + 空闲中断）
 *
 * ======================= 机制（不依赖任何不确定的硬件语义）=======================
 *   环形 DMA(CIRC)：硬件自动重装 NDTR、流永不停止 —— 语义是确定的。
 *   每次空闲中断只做一件事：读 NDTR 算出写位置，和上次位置相减得到本帧长度。
 *
 *     空闲中断 -> pos = buf_size - NDTR          （只读）
 *              -> got = pos - last_pos            （回绕则 + buf_size）
 *              -> 交付 &buf[last_pos] 起 got 字节
 *              -> last_pos = pos
 *
 *   ★ 中断里不写任何 DMA 寄存器，所以也避开了 STM32H7 Errata 2.22
 *     「UART/USART - DMA transfer lock」。
 *
 * ======================= 踩过的坑，别再改回去 =======================
 *   ① 【不要】改回 DBM 双缓冲 + "假设收满一帧硬件会重装 NDTR 并把 CT 翻块"。
 *      这个假设一旦不成立就是"一帧都收不到"（曾经真的这样）。
 *      环形 DMA + 自己算位置差没有这个假设，且帧长是实测的、能自愈错位。
 *   ② 【不要】把交付改成 DMA 传输完成中断：参考实现 bsp_rc.c 全程不用
 *      DMA 中断（连 XferCpltCallback 都不注册），空闲中断才是验证过的触发源。
 *   ③ CubeMX 把这条流配成 DMA_NORMAL（Core/Src/usart.c:119），
 *      必须靠软件设 DMA_SxCR_CIRC，否则收完一轮硬件就停了。
 *
 * 仍然保留的 H7/F4 差异适配：
 *   · DMA_HandleTypeDef.Instance 是 void*，访问寄存器必须转型（见 dma_reg()）；
 *   · USART 的数据寄存器是 RDR/TDR，不是 F4 的 DR；
 *   · 空闲标志要用 ICR.IDLECF 清，不能用 __HAL_UART_CLEAR_PEFLAG。
 *****************************************************************************/
#include "bsp_uart.h"

#include "usart.h"   /* huart1 */
#include "dma.h"

/* CubeMX 生成的 usart.h 里只 extern 了 huart1，没有 hdma_usart1_rx，
 * 这里补上（Core/Src/stm32h7xx_it.c:60 也是这么做的）。 */
extern DMA_HandleTypeDef hdma_usart1_rx;

/* ==================== 总线接线表（换串口只改这里） ==================== */
typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;
} BspUartMap;

static const BspUartMap g_bus[BSP_UART_BUS_NUM] = {
    { &huart1, &hdma_usart1_rx },   /* BSP_UART_BUS_1 */
};

/* ==================== 每条总线的运行时状态 ==================== */
typedef struct {
    uint8_t  *buf;         /* 环形缓冲区 */
    uint16_t  size;        /* 缓冲区大小（帧长的整数倍，建议 2 倍）*/
    uint16_t  last_pos;    /* 上次空闲中断时 DMA 的写位置 */
    uint8_t   inited;
    volatile BSP_UART_RxHandler handler;
    volatile uint32_t rx_frames;
    volatile uint32_t rx_errors;
} BspUartState;

static BspUartState g_uart[BSP_UART_BUS_NUM];

static int32_t bus_ok(bsp_uart_bus_t bus)
{
    return (bus < (bsp_uart_bus_t)BSP_UART_BUS_NUM) ? SER_OK : SER_ERR_PARAM;
}

/* H7 的 DMA_HandleTypeDef.Instance 是 void*（同一个句柄要覆盖 DMA 流、BDMA 通道
 * 与 MDMA），所以直接访问寄存器必须显式转型 —— HAL 自己的宏也是这么做的
 * （stm32h7xx_hal_dma.h:1096）。 */
static DMA_Stream_TypeDef *dma_reg(const DMA_HandleTypeDef *hdma)
{
    return (DMA_Stream_TypeDef *)hdma->Instance;
}

static void deliver(bsp_uart_bus_t bus, const uint8_t *data, uint16_t len)
{
    BSP_UART_RxHandler h = g_uart[bus].handler;

    if (len == 0u) { return; }

    g_uart[bus].rx_frames++;
    if (h != 0) { h(bus, data, len); }
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */
int32_t BSP_UART_Init(bsp_uart_bus_t bus, uint8_t *buf, uint16_t buf_size)
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma;

    if (bus_ok(bus) != SER_OK)          { return SER_ERR_PARAM; }
    if ((buf == 0) || (buf_size == 0u)) { return SER_ERR_PARAM; }

    huart = g_bus[bus].huart;
    hdma  = g_bus[bus].hdma_rx;
    if ((huart == 0) || (hdma == 0)) { return SER_ERR_STATE; }   /* 该总线没配 DMA */

    /* 1) 使能串口的 DMA 接收请求（DMA 请求，不是中断） */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* 2) 使能空闲中断 —— 唯一的交付触发源（参考实现验证过） */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    /* 3) 关 DMA，并等 EN 位真正落下去（否则后面写的寄存器不生效） */
    __HAL_DMA_DISABLE(hdma);
    while ((dma_reg(hdma)->CR & DMA_SxCR_EN) != 0u) {
        __HAL_DMA_DISABLE(hdma);
    }

    /* 4) 关掉全部 DMA 中断：交付不走 DMA 中断，避免和空闲中断重复交付 */
    dma_reg(hdma)->CR &= ~(uint32_t)(DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME);

    /* 5) 明确关掉双缓冲：本方案是单缓冲环形，不用 DBM */
    dma_reg(hdma)->CR &= ~(uint32_t)DMA_SxCR_DBM;

    /* 6) 外设地址 = 串口数据寄存器（H7 是 RDR） */
    dma_reg(hdma)->PAR  = (uint32_t)(&(huart->Instance->RDR));

    /* 7) 内存地址 = 环形缓冲区，长度 = 缓冲区大小 */
    dma_reg(hdma)->M0AR = (uint32_t)buf;
    dma_reg(hdma)->NDTR = (uint32_t)buf_size;

    /* 8) ★ 环形模式：硬件自动重装 NDTR、流永不停止。
     *      CubeMX 配的是 DMA_NORMAL，这里必须补上 CIRC。 */
    SET_BIT(dma_reg(hdma)->CR, DMA_SxCR_CIRC);

    /* 9) 使能 DMA */
    __HAL_DMA_ENABLE(hdma);

    g_uart[bus].buf       = buf;
    g_uart[bus].size      = buf_size;
    g_uart[bus].last_pos  = 0u;
    g_uart[bus].inited    = 1u;
    g_uart[bus].rx_frames = 0u;
    g_uart[bus].rx_errors = 0u;

    return SER_OK;
}

/* ==========================================================================
 * 中断入口（USART 空闲中断）—— 交付路径，只读寄存器
 * ========================================================================== */
void BSP_UART_IRQHandler(bsp_uart_bus_t bus)
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma;
    uint16_t cnt, pos, last, got;

    if (bus_ok(bus) != SER_OK)    { return; }
    if (g_uart[bus].inited == 0u) { return; }

    huart = g_bus[bus].huart;
    hdma  = g_bus[bus].hdma_rx;
    if ((huart == 0) || (hdma == 0)) { return; }

    /* 只认空闲中断 */
    if ((huart->Instance->ISR & USART_ISR_IDLE) == 0u) { return; }

    /* 清空闲标志：H7 必须清 ICR 的 IDLECF 位
     * （不能用 __HAL_UART_CLEAR_PEFLAG，它只清 PE/FE/NE/ORE，清不掉 IDLE） */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* ---- 只读 NDTR，算出当前写位置 ---- */
    cnt = (uint16_t)__HAL_DMA_GET_COUNTER(hdma);
    if (cnt > g_uart[bus].size) { cnt = 0u; }          /* 防御：异常读数 */
    pos = (uint16_t)(g_uart[bus].size - cnt);

    last = g_uart[bus].last_pos;
    got  = (pos >= last) ? (uint16_t)(pos - last)
                         : (uint16_t)(pos + g_uart[bus].size - last);
    g_uart[bus].last_pos = pos;

    /* 空闲了但没收到新字节 */
    if (got == 0u) { g_uart[bus].rx_errors++; return; }

    /* 跨回绕边界的一帧不连续：丢弃。
     * buf_size 给成帧长整数倍（推荐 2 倍）时不会走到这里。 */
    if (((uint32_t)last + (uint32_t)got) > (uint32_t)g_uart[bus].size) {
        g_uart[bus].rx_errors++;
        return;
    }

    /* 交付本帧（长度是实测值，帧头/校验交给 ser 层） */
    deliver(bus, &g_uart[bus].buf[last], got);
}

/* ==========================================================================
 * 停止接收
 * ========================================================================== */
int32_t BSP_UART_Deinit(bsp_uart_bus_t bus)
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma;

    if (bus_ok(bus) != SER_OK) { return SER_ERR_PARAM; }

    huart = g_bus[bus].huart;
    hdma  = g_bus[bus].hdma_rx;

    if (huart != 0) { __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE); }
    if (hdma != 0) {
        __HAL_DMA_DISABLE(hdma);
        while ((dma_reg(hdma)->CR & DMA_SxCR_EN) != 0u) { __HAL_DMA_DISABLE(hdma); }
    }

    g_uart[bus].inited = 0u;
    return SER_OK;
}

/* ==================== 注册 / 注销 ==================== */
int32_t BSP_UART_RegisterRx(bsp_uart_bus_t bus, BSP_UART_RxHandler handler)
{
    if (bus_ok(bus) != SER_OK) { return SER_ERR_PARAM; }
    if (handler == 0)          { return SER_ERR_PARAM; }
    g_uart[bus].handler = handler;
    return SER_OK;
}

int32_t BSP_UART_UnregisterRx(bsp_uart_bus_t bus)
{
    if (bus_ok(bus) != SER_OK) { return SER_ERR_PARAM; }
    g_uart[bus].handler = 0;
    return SER_OK;
}

uint32_t BSP_UART_GetRxFrameCount(bsp_uart_bus_t bus)
{
    return (bus_ok(bus) != SER_OK) ? 0u : g_uart[bus].rx_frames;
}

uint32_t BSP_UART_GetRxErrorCount(bsp_uart_bus_t bus)
{
    return (bus_ok(bus) != SER_OK) ? 0u : g_uart[bus].rx_errors;
}
