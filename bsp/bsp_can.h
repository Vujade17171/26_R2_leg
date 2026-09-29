/*****************************************************************************
 * bsp_can.h —— FDCAN 板级驱动接口（bsp 层）
 *
 * 分层位置：bsp —— 只做"外设原语"，不含任何电机协议语义，也不认识上层对象。
 *   允许依赖：HAL / CMSIS / common 公共头
 *   禁止依赖：FreeRTOS、ser 与 app 目录
 *
 * 上行（中断 -> 上层）用【注册回调】：bsp 不知道 motors[] 在哪，
 *   由 ser/dev/can_router 在初始化时把自己注册进来 —— 这是切断
 *   "bsp 反向 extern app 全局变量"的关键。
 * 下行（发送）走 BSP_CAN_SendStd / BSP_CAN_SendExt。
 *
 * 注意：上层只认总线编号与本文件的 BSP_CAN_ID_*，【不暴露 FDCAN_HandleTypeDef*】，
 *   这样换 MCU / 换 CAN 外设时只需改 bsp_can.c。
 *****************************************************************************/
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include <stdint.h>
#include "leg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 总线编号 */
typedef uint8_t bsp_can_bus_t;

#define BSP_CAN_BUS_1   0u
#define BSP_CAN_BUS_2   1u
#define BSP_CAN_BUS_NUM 2u

/* 帧格式标记：本层自定义的抽象，【与 HAL 的 FDCAN_*_ID 数值无关】，
 * 上层只用这两个宏做区分。 */
#define BSP_CAN_ID_STD  0u
#define BSP_CAN_ID_EXT  1u

/* 接收回调：在【中断上下文】被调用，实现必须轻量
 * （不得打印、不得延时、不得调用 OS API） */
typedef void (*BSP_CAN_RxHandler)(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                                  const uint8_t *data, uint8_t len);

/* 初始化：配置过滤器（标准帧 + 扩展帧全收）、启动外设、使能 RX FIFO0 中断。
 * 与 CubeMX 生成的 MX_FDCAN1_Init() 配对：先 MX_FDCAN1_Init()，再本函数。
 * 返回 SER_OK / SER_ERR_PARAM / SER_ERR_BUS */
int32_t BSP_CAN_Init(bsp_can_bus_t bus);

/* 注册 / 注销接收回调（同一总线同时只保留一个接收者） */
int32_t BSP_CAN_RegisterRx(bsp_can_bus_t bus, BSP_CAN_RxHandler handler);
int32_t BSP_CAN_UnregisterRx(bsp_can_bus_t bus);

/* 发送：标准帧 / 扩展帧，经典 CAN、DLC<=8（长度不足补 0） */
int32_t BSP_CAN_SendStd(bsp_can_bus_t bus, uint32_t std_id,
                        const uint8_t *data, uint8_t len);
int32_t BSP_CAN_SendExt(bsp_can_bus_t bus, uint32_t ext_id,
                        const uint8_t *data, uint8_t len);
//vesc发送接口
int32_t vesc_can_send(bsp_can_bus_t bus, uint32_t id, 
                        const uint8_t *data, uint8_t len);

/* 收到并成功解析的帧总数（上电等待驱动板反馈时用它判断"是否已有通信"） */
uint32_t BSP_CAN_GetRxFrameCount(bsp_can_bus_t bus);

/* 最近一帧的 ID 与数据（调试观测用；data8 传 NULL 表示不要数据） */
void BSP_CAN_GetLastRx(bsp_can_bus_t bus, uint32_t *id, uint8_t *data8);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H */