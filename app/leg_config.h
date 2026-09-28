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

/* ===================== 控制节拍 ===================== */
/* 主循环每轮执行两次 "tick += 1; delayUntil(tick);"：
 *   AK 帧 -> +1ms -> EL05 腕帧 -> +1ms -> 回到 AK 帧
 * 所以两拍 AK 之间隔的是 2ms，不是 1ms。下面只是 DWT 异常时的兜底值。 */
#define APP_CTRL_DT_FALLBACK_S  0.002f
#define APP_CTRL_DT_MAX_S       0.05f

/* dt 来源开关：1 = DWT 实测周期（推荐）；0 = 固定使用 APP_CTRL_DT_FALLBACK_S */
#define APP_DT_USE_DWT          1

/* ===================== 调试 ===================== */
/* 1 = 把服务层状态发布到全局变量，供 MDK Watch 观测（变量名与旧版一致） */
#define APP_PUBLISH_DEBUG       1

#endif /* __LEG_CONFIG_H */