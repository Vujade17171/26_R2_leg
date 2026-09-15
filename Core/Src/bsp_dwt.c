#include "bsp_dwt.h"

/* ================================================================================
 * 文件：bsp_dwt.c
 * 作用：基于 Cortex-M7 DWT 周期计数器(CYCCNT)的高精度时钟。
 *
 * 原理：CYCCNT 是内核硬件计数器，每过一个 CPU 时钟周期自动 +1（纯硬件，不占 CPU）。
 *       只要记录 CPU 主频，就能把"周期数"换算成"真实时间"。
 *
 * 三个主要用途（力控常用）：
 *   1) DWT_GetDeltaT()      —— 测循环周期 dt，供积分/微分使用；
 *   2) DWT_GetTimeline_*()  —— 给事件打时间戳、测代码耗时；
 *   3) DWT_Delay_*()        —— 微秒/毫秒级忙等延时。
 *
 * 注意：CYCCNT 是 32 位，240MHz 下约 17.9 秒回绕一次；本文件通过 CYCCNT_RoundCount
 *       累加回绕次数，把计数扩展成 64 位，因此时间线可以连续运行远超 17.9 秒。
 * ================================================================================
 */

DWT_Time_t SysTime = {0};

/* ==================== 内部静态变量 ==================== */
static uint32_t CPU_FREQ_Hz;       /* CPU 频率(Hz) */
static uint32_t CPU_FREQ_Hz_ms;    /* 每毫秒的周期数 */
static uint32_t CPU_FREQ_Hz_us;    /* 每微秒的周期数 */
static uint32_t CYCCNT_RoundCount; /* CYCCNT 回绕次数 */
static uint32_t CYCCNT_Last;       /* 上次读到的 CYCCNT，用于检测回绕 */
static uint64_t CYCCNT64;          /* 64 位扩展周期计数 */

/* ==================== 内部函数 ==================== */

/*
 * 作用：检测 CYCCNT 是否发生回绕（本次值 < 上次值），若是则回绕次数 +1。
 *       这是内部辅助函数，外部不要调用。
 * 入参：cnt_now = 本次读到的 CYCCNT 值。
 * 返回：无。
 */
static void DWT_CNT_Update(uint32_t cnt_now)
{
    if (cnt_now < CYCCNT_Last) {
        CYCCNT_RoundCount++;
    }
    CYCCNT_Last = cnt_now;
}

/* ==================== 初始化 ==================== */

/*
 * 作用：使能 DWT 跟踪单元、清零并启动周期计数器 CYCCNT，并记录主频。
 * 入参：cpu_freq_mhz = CPU 主频(MHz)，本项目传 240。
 * 返回：无。
 * 用法：开循环前调用一次：DWT_Init(240);
 */
void DWT_Init(uint32_t cpu_freq_mhz)
{
    /* 使能 DWT 跟踪单元（DEMCR 的 TRCENA 位） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* 清零周期计数器 */
    DWT->CYCCNT = 0U;
    /* 使能 CYCCNT 计数（DWT_CTRL 的 CYCCNTENA 位） */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 记录主频，并预计算"每毫秒/每微秒的周期数"，避免重复除法 */
    CPU_FREQ_Hz    = cpu_freq_mhz * 1000000U;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000U;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000U;

    CYCCNT_RoundCount = 0U;
    CYCCNT_Last       = 0U;
    CYCCNT64          = 0ULL;
}

/* ==================== 两帧间隔测量 ==================== */

/*
 * 作用：测量两次调用之间的时间间隔（力控循环的 dt）。
 * 入参：cnt_last = 指向 uint32_t 计数值的指针，函数会把本次值写回。
 * 返回：时间间隔，单位秒(float)。
 * 用法：见 bsp_dwt.h 中的示例。
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    float dt = (float)(cnt_now - *cnt_last) / (float)CPU_FREQ_Hz;
    *cnt_last = cnt_now;
    DWT_CNT_Update(cnt_now);
    return dt;
}

/*
 * 作用：同 DWT_GetDeltaT，返回 double，精度更高。
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    double dt = (double)(cnt_now - *cnt_last) / (double)CPU_FREQ_Hz;
    *cnt_last = cnt_now;
    DWT_CNT_Update(cnt_now);
    return dt;
}

/* ==================== 全局时间线刷新 ==================== */

/*
 * 作用：把当前周期计数刷新成全局时间 SysTime（拆成秒/毫秒/微秒）。
 * 入参/返回：无。
 * 说明：DWT_GetTimeline_* 内部会调用它，一般无需手动调用。
 */
void DWT_SysTimeUpdate(void)
{
    uint32_t cnt_now = DWT->CYCCNT;
    DWT_CNT_Update(cnt_now);

    /* 64 位周期数 = 回绕次数 * 2^32 + 当前值 */
    CYCCNT64 = (uint64_t)CYCCNT_RoundCount * 4294967296ULL + (uint64_t)cnt_now;

    uint64_t sec = CYCCNT64 / (uint64_t)CPU_FREQ_Hz;
    uint64_t rem = CYCCNT64 - sec * (uint64_t)CPU_FREQ_Hz;

    SysTime.s  = sec;
    SysTime.ms = (uint32_t)(rem / CPU_FREQ_Hz_ms);
    SysTime.us = (uint32_t)((rem - (uint64_t)SysTime.ms * CPU_FREQ_Hz_ms) / CPU_FREQ_Hz_us);
}

/* ==================== 时间线读取 ==================== */

/*
 * 作用：返回当前时间线，单位秒。用于打时间戳。
 * 返回：float，单位秒。
 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();
    return (float)SysTime.s + (float)SysTime.ms * 0.001f + (float)SysTime.us * 0.000001f;
}

/*
 * 作用：返回当前时间线，单位毫秒。
 * 返回：float，单位毫秒。
 */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();
    return (float)SysTime.s * 1000.0f + (float)SysTime.ms + (float)SysTime.us * 0.001f;
}

/*
 * 作用：返回当前时间线，单位微秒。测代码耗时最常用。
 * 返回：uint64_t，单位微秒。
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s * 1000000ULL + (uint64_t)SysTime.ms * 1000ULL + (uint64_t)SysTime.us;
}

/* ==================== 忙等延时 ==================== */

/*
 * 作用：忙等延时，单位秒。
 * 入参：seconds = 延时时长（秒），如 0.001f 表示 1ms。
 * 返回：无。
 * 注意：死循环空转，占死 CPU，只适合微秒/毫秒级短延时；长延时用 osDelay()。
 */
void DWT_Delay(float seconds)
{
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait = (uint32_t)(seconds * (float)CPU_FREQ_Hz);
    while ((DWT->CYCCNT - tickstart) < wait) {
    }
}

/*
 * 作用：忙等延时，单位微秒。
 * 入参：us = 延时时长（微秒）。
 * 返回：无。
 * 用法：DWT_Delay_us(100);  // 延时 100us
 */
void DWT_Delay_us(uint32_t us)
{
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait = us * CPU_FREQ_Hz_us;
    while ((DWT->CYCCNT - tickstart) < wait) {
    }
}

/*
 * 作用：忙等延时，单位毫秒。
 * 入参：ms = 延时时长（毫秒）。
 * 返回：无。
 * 用法：DWT_Delay_ms(1);    // 延时 1ms
 */
void DWT_Delay_ms(uint32_t ms)
{
    DWT_Delay_us(ms * 1000U);
}
