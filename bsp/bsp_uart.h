/*****************************************************************************
 * bsp_uart.h —— UART 板级驱动接口（bsp 层）
 *
 * 分层位置：bsp —— 只做"外设原语"：串口接收底座。
 *   允许依赖：HAL / CMSIS / common（leg_types.h）
 *   禁止依赖：FreeRTOS、ser 与 app 目录
 *
 * ============ 传输机制：单缓冲 + 环形 DMA(CIRC) + 空闲中断 ============
 *   思路：环形 DMA 的语义是【确定的】—— NDTR 由硬件自动重装、流永不停；
 *   所以每次空闲中断只要读一下 NDTR 就能算出"当前写到了缓冲区的哪个位置"，
 *   和上一次的位置相减，就得到【本帧实际收到的字节数】。
 *
 *     空闲中断 -> 读 NDTR -> pos = size - NDTR
 *              -> got = pos - last_pos（回绕就 + size）
 *              -> 交付 &buf[last_pos] 起 got 个字节
 *              -> last_pos = pos
 *
 *   ★ 中断里【一个 DMA 寄存器都不写】，只读 CT/NDTR。
 *     写 NDTR / 关-开 DMA 流会踩 STM32H7 Errata 2.22
 *     「UART/USART - DMA transfer lock」（HAL 自己都要在
 *     stm32h7xx_hal_dma.c:299-308 给 USART 的流加 DMA_SxCR_TRBUFF 规避）。
 *
 *   ⚠️ 再三踩过的坑，别再改回去：
 *     ① 【不要】用 DBM 双缓冲 + 假设"收满一帧硬件会重装 NDTR、
 *        同时把 CT 翻到另一块"。这个假设不一定成立，一旦不成立
 *        就是"一帧都收不到"。环形 DMA + 自己算位置差没有这个假设。
 *     ② 【不要】把交付改成 DMA 传输完成中断：参考实现 bsp_rc.c
 *        全程不用 DMA 中断，空闲中断才是验证过的触发源。
 *     ③ CubeMX 把这条流配成 DMA_NORMAL（Core/Src/usart.c:119），
 *        必须靠软件设 CIRC，否则收完一轮就停。
 *****************************************************************************/
#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include "leg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 总线编号 */
typedef uint8_t bsp_uart_bus_t;

#define BSP_UART_BUS_1   0u   /* USART1（CubeMX 已配 DMA1_Stream0 + Normal 模式）*/
#define BSP_UART_BUS_NUM 1u

/* 接收回调：data 指向本层内部的环形缓冲区，len = 本帧【实测】字节数。
 * 【中断上下文】调用：必须轻量（不打印 / 不延时 / 不调 OS API）。
 *   · 数据在环形缓冲里，下一帧可能覆盖同一片区域 —— 约定上"本帧必须在回调
 *     内解析或拷走"。当前 ser 层的 RC_OnFrame 是当场解包的，所以单缓冲足够。
 *   · len 是实测值，可能不等于帧长（被截断/受干扰）—— 帧头与校验由 ser 层判断。*/
typedef void (*BSP_UART_RxHandler)(bsp_uart_bus_t bus, const uint8_t *data, uint16_t len);

/* 初始化接收：
 *   buf / buf_size：环形缓冲区。要求 buf_size 是【帧长的整数倍】且 >= 2 倍帧长
 *                   （例：帧 32 字节 -> buf_size 给 64）。
 *                   给 2 倍是为了让一帧在缓冲里始终连续、不会跨回绕边界。
 *   内部动作：使能 CR3.DMAR、使能空闲中断、把 DMA 配成环形
 *             （PAR=RDR、M0AR=buf、NDTR=buf_size、CIRC=1）并启动。
 * 返回 SER_OK / SER_ERR_PARAM / SER_ERR_STATE（该总线未配 DMA 句柄）
 * 【任务上下文调用】：内部有等待 DMA 关闭的阻塞循环，禁止在中断里调。 */
int32_t BSP_UART_Init(bsp_uart_bus_t bus, uint8_t *buf, uint16_t buf_size);

/* 停止接收：关空闲中断与 DMA 流 */
int32_t BSP_UART_Deinit(bsp_uart_bus_t bus);

/* 注册 / 注销接收回调（同一总线只保留一个接收者） */
int32_t BSP_UART_RegisterRx(bsp_uart_bus_t bus, BSP_UART_RxHandler handler);
int32_t BSP_UART_UnregisterRx(bsp_uart_bus_t bus);

/* 统计：交付给上层的帧数 / 异常次数（空闲但无新字节、跨回绕边界） */
uint32_t BSP_UART_GetRxFrameCount(bsp_uart_bus_t bus);
uint32_t BSP_UART_GetRxErrorCount(bsp_uart_bus_t bus);

/* 中断入口（★ 交付路径）：由 Core/Src/stm32h7xx_it.c 的 USARTx_IRQHandler 调用。
 * 只读寄存器，不写任何 DMA 寄存器。 */
void BSP_UART_IRQHandler(bsp_uart_bus_t bus);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
