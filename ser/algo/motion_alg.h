/*****************************************************************************
 * motion_alg.h —— 关节限速器 + 重力补偿（ser/algo）
 *
 * 分层位置：ser/algo —— 纯算法，不依赖 HAL / FreeRTOS / bsp，可 PC 单测。
 *   依赖：common（leg_types.h）
 *
 * 为什么从 kinematics.c 拆出来：
 *   两者与原运动学代码的变化原因完全不同（机械尺寸 vs 摆动安全 vs 负载配平），
 *   且各自都带"上一拍"动态状态。原实现把这些状态散成文件级 static 全局
 *   （g_q_cmd / last_tau_* / dwt_cnt_last），是隐式单例、第二条腿就会互相干扰。
 *   现在全部收进句柄，由调用方持有 —— 多腿/多实例天然可用。
 *
 * 关键改动：重力补偿不再自己去读 DWT 取周期，dt 由调用方传入。
 *   （时间只出现在 osal 与 app，算法层保持纯净）
 *****************************************************************************/
#ifndef __MOTION_ALG_H
#define __MOTION_ALG_H

#include <stdint.h>
#include "leg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 * 关节空间限速（摆动速率限制）
 * 运控(MIT)模式的协议里只有 p/v/kp/kd/t，没有"速度上限"参数（v 是前馈速度，
 * 配 kd 起阻尼，不是限速指令），所以限速在主控侧做：
 * 限制每拍关节指令角的增量 = 限速值(rad/s) × dt。
 * 限速值在 leg_types.h（JOINT_RATE_Q1/Q2/Q3_MAX）。
 * 只限制"期望轨迹"的推进速度：kp≠0 时电机实际摆速≈该上限；
 * 但 kp=0（纯力矩测试）或外力推动时实际速度不受此约束。
 * ===================================================================== */
typedef struct {
    LegJointAngles q_cmd;   /* 上一拍限速后的指令关节角 */
    uint8_t        inited;  /* 0 = 未初始化：首拍不限速（不跳变） */
} JointRateLimiter;

/* 复位限速器：以上电时的实际关节角作为指令起点，保证首拍不跳变 */
void JointRateLimit_Init(JointRateLimiter *rl, const LegJointAngles *q_now);

/* 期望关节角 -> 每拍增量受限的指令关节角
 *   q_des：IK/轨迹解出的期望关节角
 *   dt   ：距上次调用的实测周期 (s)，<=0 或未初始化时不做限速
 *   q_cmd：输出限速后的指令关节角（不能与 q_des 是同一变量） */
void JointRateLimit_Apply(JointRateLimiter *rl, const LegJointAngles *q_des,
                          float dt, LegJointAngles *q_cmd);

/* =====================================================================
 * 重力补偿（简单杠杆原理）
 * 每个关节的重力矩 = Σ (质量 × g × 该关节到该质量质心的水平距离)
 * 水平距离 = 各段长度 × cos(该段的绝对角度)，即力臂的水平投影。
 * 输出 = dir × 力矩 / gear，再经【限幅 + 变化率限制】平滑下发。
 * ===================================================================== */

/* 重力补偿配置：质量、质心力臂、方向、减速比、限幅与变化率限制 */
typedef struct {
    float m_arm;    /* 大臂连杆质量 (kg)                          */
    float lc_arm;   /* 大臂质心距肩关节 (m)                       */
    float m_elbow;  /* 肘关节等效质量 (kg)，位于 L1 末端           */
    float m_wrist;  /* 腕点等效质量 (kg)，位于 L2 末端             */
    float m_load;   /* 腕部负载质量 (kg)，空载=吸盘组件            */
    float lc_load;  /* 负载质心距腕关节 (m)                       */
    float g;        /* 重力加速度 (m/s²)                          */
    float dir_sh;   /* 肩：关节->电机 方向 (+1/-1)                */
    float dir_el;   /* 肘：方向 (+1/-1)                           */
    float dir_wr;   /* 腕：方向 (+1/-1)，电机反馈角=-关节角 → -1   */
    float gear_sh;  /* 肩：减速比/补偿强度，用当前工程数值          */
    float gear_el;  /* 肘：同上                                   */
    float gear_wr;  /* 腕：同上                                   */
    float max_torque_sh; /* 肩力矩限幅 (N·m，电机侧)              */
    float max_torque_el; /* 肘力矩限幅 (N·m，电机侧)              */
    float max_torque_wr; /* 腕力矩限幅 (N·m，电机侧)              */
    float rate_limit;    /* 力矩变化率限制 (N·m/s，电机侧)         */
} GravCfg;

/* 当前工程实测调好的默认参数（想改就改这个常量） */
extern const GravCfg GRAV_CFG_DEFAULT;

/* 重力补偿句柄：保存"上一拍力矩"与首拍标记（原实现是文件级 static） */
typedef struct {
    GravCfg cfg;
    float   last_tau_sh;
    float   last_tau_el;
    float   last_tau_wr;
    uint8_t first_run;   /* 首拍：不做变化率限制，直接取目标值作初值 */
} GravityComp;

/* 初始化：cfg 传 NULL 时使用 GRAV_CFG_DEFAULT */
void GravityComp_Init(GravityComp *gc, const GravCfg *cfg);

/* 计算并平滑补偿力矩
 *   link  ：连杆参数（由 Kinematics_Init 使用的同一份长度）
 *   q1/q2/q3：肩/肘/腕关节角 (rad)，腕关节角 = −EL05 电机反馈角
 *   dt    ：距上次调用的实测周期 (s)；首拍内部按 0 处理（只限幅不限速）
 *   tau_sh/tau_el/tau_wr：可传 NULL 表示不输出 */
void GravityComp_Update(GravityComp *gc, const LegLinkParam *link,
                        float q1, float q2, float q3, float dt,
                        float *tau_sh, float *tau_el, float *tau_wr);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_ALG_H */