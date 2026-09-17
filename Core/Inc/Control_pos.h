#ifndef __CONTROL_POS_H
#define __CONTROL_POS_H

#include "main.h"
#include "cmsis_os.h"
#include "math.h"          /* atan2f / fabsf / cosf / sinf 等数学函数 */
#include "bsp_fdcan.h"     /* g_ak80 / g_ak45 电机句柄 */
#include "bsp_structure.h" /* leg_pos_t / LegMotion_t 结构体 */
#include "Joint_EL05.h"    /* g_el05 腕部电机 */
#include "Joint_AK80.h"    /* AK80 电机驱动 */

/* ==================== 连杆几何参数（单位：米） ==================== */
/* 这三段就是机械臂的"骨头"，正/逆运动学都用它们算位置 */
#define L1 0.35f   /* 大臂长度：基座 J1 到 肘部 J2 的中心距 */
#define L2 0.25f   /* 小臂长度：肘部 J2 到 腕部 J3 的中心距 */
#define L3 0.10f   /* 腕部长度：腕部 J3 到 末端落点         */

/* ==================== 关节软限位（单位：rad） ==================== */
/* 防止机械臂撞到结构极限。数值是"保守值"，装好后可按实际机械边界微调。
 * 换算：π ≈ 3.14159，所以 90° = 1.5708，150° = 2.6179 */
#define Q1_MIN   (-1.25663704f)   /* 大臂最小角：-90° */
#define Q1_MAX   ( 3.1415926f)   /* 大臂最大角：+90° */
#define Q2_MIN   (-2.82743334f)   /* 小臂最小角：-150° */
#define Q2_MAX   ( 2.82743334f)   /* 小臂最大角：+150° */

/* ==================== IK 肘部构型选择 ==================== */
/* 同一个目标点，机械臂通常有两种摆法（肘部向上/向下）。
 * 这三个宏用来告诉 IK 用哪种摆法： */
#define ELBOW_AUTO   (-1)     /* 自动：两个摆法都试，选离当前位姿最近的 */
#define ELBOW_DOWN    0        /* 强制肘部向下（sin_q2 < 0） */
#define ELBOW_UP      1        /* 强制肘部向上（sin_q2 > 0） */

/* ==================== 全局对象声明 ==================== */
extern LegMotion_t leg_motion; /* 逆运动学结果缓存，见 Control_pos.c */
extern leg_pos_t   pos;        /* 正运动学结果缓存，见 Control_pos.c */

/* ==================== 工具函数 ==================== */
/* 下面这些函数加了 static inline，意思是"编译时直接展开进调用处"，
 * 省去函数调用的开销，适合这种被频繁调用的小函数。 */

/* 把角度归一化到 (-π, π] 范围内。
 * 例：3.5π 会被减成 -0.5π，避免角度越转越离谱 */
static inline float wrap_pi(float a)
{
    while (a >  3.14159265f) a -= 6.28318530f;  /* 大于 π 就减 2π */
    while (a < -3.14159265f) a += 6.28318530f;  /* 小于 -π 就加 2π */
    return a;
}

/* 限幅：把 v 限制在 [lo, hi] 之间。
 * 例：clampf(5, 0, 3) = 3；clampf(-1, 0, 3) = 0；clampf(2, 0, 3) = 2 */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 判断某个关节角是否在软限位内。
 * joint = 1 表示大臂，joint = 2 表示小臂。
 * 返回 1 表示合法，0 表示超限位 */
static inline int in_limit(float q, int joint)
{
    if (joint == 1) return (q >= Q1_MIN && q <= Q1_MAX);
    if (joint == 2) return (q >= Q2_MIN && q <= Q2_MAX);
    return 0;
}

/* ==================== 电机安装方向 ==================== */
/* 电机装反时，电机转子角与真实关节角之间差一个负号。
 * 方向系数 DIR：+1 = 正常，-1 = 装反。
 * 以后哪个电机装反/装正，只改下面对应的系数，其余代码不用动。 */
#define AK80_DIR  ( 1.0f)   /* 大臂电机 AK80：正常 */
#define AK45_DIR  ( 1.0f)   /* 小臂电机 AK45：正常（正面看逆时针角度增加） */

/* ==================== 电机零点偏移（机械零偏） ==================== */
/* 关节角为 0 时，电机实际要转到的角度。单独用宏管理，方便重新标定。 */
#define AK80_OFFSET   0.29   /* 大臂零偏 */
#define AK45_OFFSET   -1.9f  /* 小臂零偏：小臂水平时电机角度为 -1.9 rad */

/* 电机反馈角 → 关节角（正运动学读反馈时用，处理装反和零偏）。
 * 公式：q = (θ - OFFSET) * DIR
 * 由下发公式 θ = DIR * q + OFFSET 反解得出，保证读写一致 */
static inline float motor1_to_joint(float motor_angle) { return (motor_angle - AK80_OFFSET) * AK80_DIR; }
static inline float motor2_to_joint(float motor_angle) { return (motor_angle - AK45_OFFSET) * AK45_DIR; }

/* 关节角 → 电机下发角（逆运动学下发时用）。
 * 先按方向系数取反，再加零点偏移 */
static inline float joint_to_motor_1(float q1) { return AK80_DIR * q1 + AK80_OFFSET; }
static inline float joint_to_motor_2(float q2) { return AK45_DIR * q2 + AK45_OFFSET; }

/* ==================== 轨迹规划参数 ==================== */
/* 控制周期：单位秒。与电机控制任务的执行频率保持一致。
 * Moto_Task.c 里是 osDelay(1)，即 1ms 一个周期，所以这里是 0.001。 */
#define CONTROL_DT  0.001f

/* ==================== 全局对象声明 ==================== */
extern JTraj_t jtraj;  /* 轨迹规划句柄，见 Control_pos.c */

/* ==================== 公共函数声明 ==================== */

/* 正运动学：已知电机角度，算出末端位置。
 * 结果写入全局 pos，也会回写到 my_pos（如果非空） */
extern void Forward_Kinematics(leg_pos_t *my_pos);

/* 逆运动学（腕部目标版）：
 * 已知想要的腕部位置 target_x / target_z，反解电机角度。
 * elbow_up 填 ELBOW_AUTO / ELBOW_DOWN / ELBOW_UP。
 * 返回 0 成功，-1 失败（够不着等） */
extern int Inverse_Kinematics(leg_pos_t *my_pos, int elbow_up);

/* 逆运动学（末端目标版）：
 * 已知想要的末端落点 x_s / z_s + 姿态 yaw，自动反推腕部位置再解算。
 * yaw 填 0 表示沿用当前末端姿态 */
extern int Inverse_Kinematics_EE(leg_pos_t *my_pos, int elbow_up);

/* 逆运动学核心函数：上面两个封装最终都调用它 */
extern int leg_inverse(const leg_pos_t *leg_pos, int elbow_up);

/* ==================== 五次多项式轨迹规划 ==================== */

/* 自动计算轨迹时长：根据两个关节的转角差，按最大关节速度估算运动时间。
 * 入参：
 *   q1_start/q2_start —— 大臂/小臂起始关节角 (rad)
 *   q1_end  /q2_end   —— 大臂/小臂目标关节角 (rad)
 * 返回：建议轨迹时长 (s)，被钳制在 [0.65, 3.0] 秒 */
extern float calc_motion_time(float q1_start, float q2_start, float q1_end, float q2_end);

/* 启动轨迹规划（系数预计算版）：
 * 一次性把五次多项式 6 个系数 a0..a5 算好存进 jtraj.coeff，
 * 之后 jtraj_update 只需代入时间 t 求值，省去每周期重复计算。
 * 入参：
 *   q1_start/q2_start —— 起始关节角 (rad)，建议取当前电机角度保证无缝衔接
 *   q1_end  /q2_end   —— 目标关节角 (rad)
 *   T_sec             —— 轨迹总时长 (s)，可先用 calc_motion_time 自动算 */
extern void jtraj_start(float q1_start, float q2_start, float q1_end, float q2_end, float T_sec);

/* 每周期调用一次，推进轨迹并输出位置/速度/加速度到 motion（通常传 &leg_motion）。
 * 返回：0 = 轨迹运行中，1 = 本次轨迹已完成，-1 = 轨迹未激活 */
extern int jtraj_update(LegMotion_t *motion);

/* 二连杆（XZ 平面）雅可比矩阵：关节速度 → 末端笛卡尔速度。
 * 入参：q1/q2 当前关节角；输出 J11..J22 到指针。
 * 用途：把关节速度换算成末端速度，供 cartesian_velocity_limit 限速。 */
static inline void jacobian_rz(float q1, float q2, float *J11, float *J12, float *J21, float *J22)
{
    float s1  = sinf(q1);            /* sin(q1) */
    float c1  = cosf(q1);            /* cos(q1) */
    float s12 = sinf(q1 + q2);       /* sin(q1+q2) */
    float c12 = cosf(q1 + q2);       /* cos(q1+q2) */

    *J11 = -L1 * s1 - L2 * s12;      /* dx/dq1 */
    *J12 = -L2 * s12;                /* dx/dq2 */
    *J21 =  L1 * c1 + L2 * c12;      /* dz/dq1 */
    *J22 =  L2 * c12;                /* dz/dq2 */
}

/* 笛卡尔末端速度软限幅：对关节速度做平滑衰减，避免末端速度突变/超速。
 * 入参：
 *   q1/q2        —— 当前关节角 (rad)，用于算雅可比
 *   v1/v2        —— 输入/输出关节速度 (rad/s)，就地等比缩放
 *   max_cart_vel —— 末端最大允许速度 (m/s) */
extern void cartesian_velocity_limit(float q1, float q2, float *v1, float *v2, float max_cart_vel);

#endif
