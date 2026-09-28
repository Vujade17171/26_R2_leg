/*****************************************************************************
 * can_router.h —— CAN 帧路由（ser/dev）
 *
 * 分层位置：ser/dev —— 把 bsp 的中断上行"翻译"成具体设备的分发。
 *   允许依赖：bsp（注册接收者）、ser/dev 的各驱动、common
 *   禁止依赖：任何 app 全局变量
 *
 * 这是本次分层最关键的一处改动：
 *   旧工程的 Mycan.c 属于 bsp 层，却 extern 引用 app 层（Task/leg_task.c）的
 *   电机句柄数组，形成 bsp -> app 的反向依赖与环状依赖。
 *   现在改为【依赖注入 + 注册回调】：
 *     1) CAN_Router_Init()        把自己注册为 bsp 的接收者；
 *     2) CAN_Router_BindAk/El05() 由 app 在初始化时把句柄数组交进来；
 *   路由器从此不认识任何全局变量，bsp 也不再认识上层。
 *****************************************************************************/
#ifndef __CAN_ROUTER_H
#define __CAN_ROUTER_H

#include <stdint.h>
#include "leg_types.h"
#include "bsp_can.h"
#include "ak_motor.h"
#include "el05_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：注册为 bsp_can 的接收者（应在 BSP_CAN_Init 之后调用） */
int32_t CAN_Router_Init(bsp_can_bus_t bus);

/* 绑定电机句柄数组（依赖注入）。绑定之前收到的帧会被丢弃，仅计数。 */
int32_t CAN_Router_BindAk(AK_Motor *motors, uint8_t count);
int32_t CAN_Router_BindEl05(EL05_Motor *motors, uint8_t count);

/* 收到的帧总数：供上层判断"驱动板是否已有通信"，
 * 这样 app 不必为了一个计数器去直接调用 bsp 层接口。 */
uint32_t CAN_Router_GetFrameCount(void);

/* 接收分发（由 bsp 的中断回调调用，处于【中断上下文】，必须保持轻量）：
 *   BSP_CAN_ID_STD -> AK 电机（标准帧，ID = 电机ID）
 *   BSP_CAN_ID_EXT -> EL05 电机（扩展帧私有协议） */
void CAN_Router_OnFrame(bsp_can_bus_t bus, uint32_t id, uint32_t id_type,
                        const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_ROUTER_H */