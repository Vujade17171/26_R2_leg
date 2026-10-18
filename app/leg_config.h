/*****************************************************************************
 * leg_config.h —— 应用层参数集中配置
 *
 * 分层位置：app —— 只放"业务参数"，不放算法常量（算法常量在 ser 层）。
 * 说明：原 leg_task.c 顶部的宏（ARM_TRAJ_DURATION_S / ARM_TARGET_EPS /
 *   ARM_CTRL_DT_S ...）中属于"控制策略"的部分已移入 leg_motion 的
 *   LEG_MOTION_CFG_DEFAULT；本文件保留与硬件接线、任务节拍相关的部分。
 *****************************************************************************/
#ifndef __LEG_CONFIG_H
#define __LEG_CONFIG_H

/* ===================== 时间基准 ===================== */
#define APP_CPU_FREQ_MHZ        480u   /* CPU 主频 MHz，供 DWT 时间基准 */

/* ===================== 电机接线 ===================== */
#define APP_AK_NUM              2u     /* AK 电机数量：0=大臂(肩) 1=小臂(肘) */
#define APP_EL05_NUM            1u     /* EL05 电机数量：0=腕 */
#define APP_AK_ID_SHOULDER      1u     /* 大臂电机 CAN ID */
#define APP_AK_ID_ELBOW         2u     /* 小臂电机 CAN ID */
#define APP_EL05_ID_WRIST       3u     /* 腕 EL05 电机 CAN ID */
#define APP_EL05_MASTER_ID      0xFFu  /* EL05 主机 CAN ID */

/* ===================== 上电等待反馈 ===================== */
/* 等驱动板回传反馈的最长时间 (ms)：有真实反馈后再取初始姿态，
 * 否则 pos_rad 仍为 0，会算出虚假姿态当目标 → 上电大角度跳动 */
#define APP_RX_WAIT_TIMEOUT_MS  500u

/* ===================== 上电使能握手 =====================
 * AK / EL05 驱动板是【独立上电】的，比 MCU 慢（自检 / 响一声）。
 * MCU 十几毫秒就把"进入运控模式 FF×7+FC"/"使能(类型3)"发完了，此时驱动板 CAN
 * 还没起来 → 没人 ACK → FDCAN1 又是 AutoRetransmission=DISABLE → 使能帧直接丢；
 * 原来只发一次、丢了不补发，三个电机就永远停在未使能状态（底盘不需要使能，照跑）。
 * 带调试器时你停在 main 几秒才点运行，驱动板早起来了，所以那把看着是好的。
 *
 * 握手 = 先留出上电时间，再"发使能 → 间隔重发"，直到 AK×2 + EL05 都真的回过
 * 反馈帧（用各驱动自己的 rx_cnt 判定），或到上限就往下走（不卡死腿任务）。
 *
 * 重发期间没人 ACK 会把 FDCAN 打进 Bus-Off，所以握手循环里同时做了恢复
 * （BSP_CAN_RecoverIfBusOff），这也是这里敢用 3s 上限的前提。 */
#define APP_DRIVER_BOOT_DELAY_MS      300u  /* 发第一次握手使能前先等的上电时间(ms) */
#define APP_ENABLE_RETRY_INTERVAL_MS  50u   /* 重发间隔(ms)                        */
#define APP_ENABLE_RETRY_MAX          60u   /* 最多重发次数（60×50ms = 3s 上限）    */

/* ===================== 控制节拍 ===================== */
/* 主循环每轮执行两次 "tick += 1; delayUntil(tick);"：
 *   AK 帧 -> +1ms -> EL05 腕帧 -> +1ms -> 回到 AK 帧
 * 所以两拍 AK 之间隔的是 2ms，不是 1ms。下面只是 DWT 异常时的兜底值。 */
#define APP_CTRL_DT_FALLBACK_S  0.002f
#define APP_CTRL_DT_MAX_S       0.05f

#define PI                   3.141592f //圆周率

/* ===================== 调试 ===================== */
/* 1 = 把服务层状态发布到全局变量，供 MDK Watch 观测（变量名与旧版一致） */
#define APP_PUBLISH_DEBUG       1

#endif /* __LEG_CONFIG_H */