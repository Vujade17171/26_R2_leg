#ifndef MYCAN_H
#define MYCAN_H
#include "struct_typedef.h"
#include "main.h"
#include "fdcan.h"

extern void can_filter_init(void);

/* FDCAN 接收到的扩展帧 ID 与数据缓冲（在回调内更新） */
extern uint32_t rx_id;
extern uint8_t  rx_data[8];

/* 接收分发弱钩子：Mycan.c 提供默认空实现，
 * 用户可在任意文件定义同名强函数覆盖，在函数内按 (ext_id >> 8)
 * 分发到 AK_Motor_RxDispatch() 等。 */
void Mycan_OnRxFrame(uint32_t ext_id, const uint8_t *data, uint8_t len);

#endif
