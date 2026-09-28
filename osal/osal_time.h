/*****************************************************************************
 * osal_time.h —— 时间与延时抽象（osal 层）
 *
 * 分层位置：osal —— 把"RTOS 与时间源"包成一层稳定 API。
 *   允许依赖：cmsis_os2（RTOS）、bsp（时间源）、common
 *   禁止依赖：HAL 具体外设、ser 与 app 目录
 *
 * 关键约定：ser 算法层【不得自己取时间】，dt 一律由调用方当参数传进去。
 *   时间只在这里（以及 app 的节拍）出现，算法层才能在 PC 上单测。
 *****************************************************************************/
#ifndef __OSAL_TIME_H
#define __OSAL_TIME_H

#include <stdint.h>
#include "leg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化时间基准（内部走 bsp 的 DWT 计数器）
 * cpu_mhz：CPU 主频 MHz（本工程 480） */
int32_t OSAL_Time_Init(uint32_t cpu_mhz);

/* 距上次调用的时间增量 (s)。last_cnt 是调用方持有的计数缓存。
 * 首次调用返回 0；异常大值由调用方自行钳位（见 app 中的 dt 保护）。 */
float OSAL_Time_Dt(uint32_t *last_cnt);

/* 预置计数缓存：避免首拍算出巨大的 dt */
void OSAL_Time_ResetDt(uint32_t *last_cnt);

/* 内核 tick 数与毫秒数（本工程 1 tick = 1ms，configTICK_RATE_HZ = 1000） */
uint32_t OSAL_Time_GetTick(void);
uint32_t OSAL_Time_GetMs(void);

/* 毫秒级阻塞延时 */
void OSAL_Time_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __OSAL_TIME_H */