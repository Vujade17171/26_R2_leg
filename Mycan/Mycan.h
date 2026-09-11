#ifndef MYCAN_H
#define MYCAN_H
#include "struct_typedef.h"
#include "main.h"
#include "fdcan.h"

extern void can_filter_init(void);

/* FDCAN 接收到的帧 ID 与数据缓冲（在回调内更新） */
extern uint32_t rx_id;
extern uint8_t  rx_data[8];
extern volatile uint32_t g_rx_frames;   /* 收到帧的总计数（调试/统计用） */

/* 接收分发钩子：Mycan.c 中已实现——按帧格式分流
 *   FDCAN_STANDARD_ID  -> AK 电机（标准帧，ID=电机ID）
 *   FDCAN_EXTENDED_ID  -> EL05 电机（私有协议扩展帧）
 * 句柄数组定义见 Task/leg_task.c。 */
void Mycan_OnRxFrame(uint32_t ext_id, uint32_t id_type,
                     const uint8_t *data, uint8_t len);

#endif
