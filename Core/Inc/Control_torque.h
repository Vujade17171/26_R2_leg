#ifndef __CONTROL_TORQUE_H
#define __CONTROL_TORQUE_H

#include "main.h"

/* ==================== 重力补偿配置结构体 ==================== */
/* 所有数值都是"机械参数"，需要你按自己机械臂实际测量/估算后填写。
 * 默认值已在 Control_torque.c 里给了参考值，可直接在调试器里改。 */
typedef struct {
    /* ---- 质量与几何参数 ---- */
    float m_upper;       /* 大臂连杆自身质量 (kg) */
    float lc_upper;      /* 大臂连杆质心到肩关节的距离 (m)，约等于 L1/2 */
    float m_elbow;       /* 小臂+肘部集中质量 (kg)，近似集中在小臂远端 */
    float lc_elbow;      /* 小臂质心到肘关节的距离 (m)，集中远端约等于 L2 */
    float m_wrist;       /* 腕部/末端集中质量 (kg)，近似集中在腕关节处 */
    float payload_mass;  /* 负载质量 (kg)，如吸盘/被抓物体 */
    float g;             /* 重力加速度 (m/s^2)，一般 9.81 */

    /* ---- 传动与方向 ---- */
    float shoulder_gear; /* 肩关节减速比：电机输出直接连关节填 1.0，有额外减速填 N */
    float elbow_gear;    /* 肘关节减速比：同上 */
    float shoulder_dir;  /* 肩补偿力矩方向：+1 或 -1。补偿方向反了就把这里取负 */
    float elbow_dir;     /* 肘补偿力矩方向：+1 或 -1 */

    /* ---- 力矩限幅（安全保护） ---- */
    float max_torque_shoulder; /* 肩前馈力矩上限 (Nm)，防止补偿过大 */
    float max_torque_elbow;    /* 肘前馈力矩上限 (Nm) */
    float torque_rate_limit;   /* 力矩变化率限制 (Nm/s)，防止力矩突变打手 */
} grav_comp_cfg_t;

/* 全局配置实例。定义在 Control_torque.c，可在外部（如调试器/Moto_Task.c）修改 */
extern grav_comp_cfg_t g_grav_cfg;

/*
 * 作用：计算重力补偿 + 限幅后的前馈力矩（合并版，一次调用完成）。
 *
 * 入参：
 *   tau_sh      —— 输出指针，肩（大臂 AK80）前馈力矩，单位 Nm（电机输出轴坐标）
 *   tau_el      —— 输出指针，肘（小臂 AK45）前馈力矩，单位 Nm
 *   has_payload —— 是否带负载：0=空载，1=带负载（会额外加 payload_mass 的补偿）
 *   dt          —— 控制周期（秒），用于力矩变化率限制。可传 CONTROL_DT(0.001)
 *
 * 返回：无（结果通过 tau_sh / tau_el 指针输出，指针传 NULL 表示不关心该值）
 *
 * 用法（配合你的 Moto_Task.c）：
 *   float tau_sh = 0.0f, tau_el = 0.0f;
 *   Control_Torque_FeedForward(&tau_sh, &tau_el, 0, CONTROL_DT);   // 空载
 *   AK_Motion_Control(&g_ak80, leg_motion.motor1_target_angle, 0, g_kp, g_kd, tau_sh);
 *   AK_Motion_Control(&g_ak45, leg_motion.motor2_target_angle, 0, g_kp, g_kd, tau_el);
 */
void Control_Torque_FeedForward(float* tau_sh, float* tau_el, int has_payload, float dt);

#endif
