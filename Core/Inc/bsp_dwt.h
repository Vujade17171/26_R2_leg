#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#include "main.h"
#include "bsp_structure.h"

extern DWT_Time_t SysTime;   /* 全局时间，DWT_SysTimeUpdate() 刷新 */

/* ==================== 函数声明 ==================== */

/*
 * 作用：初始化 DWT 时钟。使能内核周期计数器 CYCCNT（每过一个 CPU 时钟就 +1），
 *       并记录主频，供后续把"周期数"换算成"真实时间"。
 * 入参：cpu_freq_mhz = CPU 主频，单位 MHz（本项目是 240，就传 240）。
 * 返回：无。
 * 用法：在开循环之前调用一次即可：
 *           DWT_Init(240);
 */
void     DWT_Init(uint32_t cpu_freq_mhz);

/*
 * 作用：测量"距离上一次调用"经过了多少时间。力控循环里用来算 dt（积分、微分都要用）。
 * 入参：cnt_last = 指向一个 uint32_t 计数值的指针，函数会把本次计数值写回给它。
 * 返回：时间间隔，单位【秒】(float)。
 * 用法：
 *           uint32_t last = DWT->CYCCNT;        // 进循环前先初始化一次
 *           while (1) {
 *               float dt = DWT_GetDeltaT(&last); // 得到本周期时长（秒）
 *               // ... 用 dt 做积分/微分 ...
 *           }
 * 注意：last 第一次必须先初始化，否则第一帧 dt 是垃圾值。
 */
float    DWT_GetDeltaT(uint32_t *cnt_last);

/*
 * 作用：同 DWT_GetDeltaT，但返回 double，精度更高，适合对时间精度要求高的场合。
 * 入参/返回：同上，只是返回类型为 double。
 */
double   DWT_GetDeltaT64(uint32_t *cnt_last);

/*
 * 作用：刷新全局时间 SysTime（拆成秒/毫秒/微秒三个字段）。
 * 入参/返回：无。
 * 说明：下面的 DWT_GetTimeline_* 内部会自动调用它，一般无需手动调用。
 */
void     DWT_SysTimeUpdate(void);

/*
 * 作用：读取当前时间线，单位【秒】。用来给事件打时间戳。
 * 入参：无。返回：float，单位秒。
 * 用法：float now = DWT_GetTimeline_s();
 */
float    DWT_GetTimeline_s(void);

/*
 * 作用：读取当前时间线，单位【毫秒】。
 * 入参：无。返回：float，单位毫秒。
 * 用法：float now_ms = DWT_GetTimeline_ms();
 */
float    DWT_GetTimeline_ms(void);

/*
 * 作用：读取当前时间线，单位【微秒】。测耗时最常用。
 * 入参：无。返回：uint64_t，单位微秒。
 * 用法：
 *           uint64_t t0 = DWT_GetTimeline_us();
 *           // ... 做某件事 ...
 *           uint64_t cost = DWT_GetTimeline_us() - t0;  // 微秒级耗时
 */
uint64_t DWT_GetTimeline_us(void);

/*
 * 作用：忙等延时，单位【秒】。精确阻塞当前代码一段时间。
 * 入参：seconds = 延时时长，单位秒（如 0.001f 表示 1ms）。
 * 返回：无。
 * 注意：这是死循环空转，会占死 CPU 并阻塞当前任务，只适合微秒/毫秒级短延时；
 *       长延时请用 FreeRTOS 的 osDelay()。
 * 用法：DWT_Delay(0.001f);   // 延时 1ms
 */
void     DWT_Delay(float seconds);

/*
 * 作用：忙等延时，单位【微秒】。
 * 入参：us = 延时时长，单位微秒。
 * 返回：无。
 * 用法：DWT_Delay_us(100);   // 延时 100us
 */
void     DWT_Delay_us(uint32_t us);

/*
 * 作用：忙等延时，单位【毫秒】。
 * 入参：ms = 延时时长，单位毫秒。
 * 返回：无。
 * 用法：DWT_Delay_ms(1);     // 延时 1ms
 */
void     DWT_Delay_ms(uint32_t ms);

#endif
