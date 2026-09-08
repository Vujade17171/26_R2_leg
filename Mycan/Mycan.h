#ifndef MYCAN_H
#define MYCAN_H
#include "struct_typedef.h"
#include "main.h"
#include "fdcan.h"

extern void can_filter_init(void);

/* FDCAN 接收到的标准帧 ID 与数据缓冲（在回调内更新） */
extern uint32_t rx_id;
extern uint8_t  rx_data[8];
extern volatile uint32_t g_rx_frames;   /* 收到标准帧的总计数（调试/统计用） */

/* 接收分发钩子：Mycan.c 中已实现——把帧按标准帧 ID(=电机ID) 路由到
 * AK 电机句柄（motors[]，见 Task/leg_task.c）并解析反馈，
 * 结果写入 motors[i].pos_rad / vel_rads / torque_nm / temp_c / err_code。 */
void Mycan_OnRxFrame(uint32_t ext_id, const uint8_t *data, uint8_t len);

#endif
