/*****************************************************************************
 * kinematics.h —— 腿部运动学解算接口（正解 FK / 逆解 IK）
 *
 * 约定：
 *  - 单位一律 SI：角度 rad、长度 m；
 *  - 腿模型为 3 关节串联（髋侧摆 hip_abduction / 髋前摆 hip / 膝 knee），
 *    具体构型与连杆参数在 kinematics.c 中定义；
 *  - 本文件只提供接口与数据结构，具体算法在 kinematics.c 中实现。
 *****************************************************************************/
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define D_L1 0.35f  // 大臂长度
#define D_L2 0.25f  // 小臂长度
#define D_L3 0.00f  // 腕部长度(暂时忽略)

/* ================== 关节机械限位（rad，按实物机械限位调整） ==================
 * 定义放头文件是为了让轨迹规划等外部模块能在"规划阶段"就校验终点是否超限
 * （超限应拒绝规划，而不是先算完再钳位）。 */
#define JOINT1_MIN  (-1.25663704f)
#define JOINT1_MAX  ( 3.1415926f)
#define JOINT2_MIN  (-2.82743334f)
#define JOINT2_MAX  ( 2.82743334f)

/* 连杆长度（单位 m）：由机械设计确定，使用前在 Init 中赋值 */
typedef struct {
    float L1; 
    float L2;   
    float L3;  
} LegLinkParam;

/* 单腿 3 关节角度（单位 rad） */
typedef struct {
    float q1;  
    float q2;    
    float q3;   
} LegJointAngles;

/* 足端在腿坐标系中的位置（单位 m） */
typedef struct {
    float x;
    float z;
} FootPosition;

/* 初始化：写入连杆参数（长度等） */
void Kinematics_Init(const LegLinkParam *param);

/* 正解 FK：由关节角度求足端位置 */
void Kinematics_Forward(const LegJointAngles *q, FootPosition *foot);

/* 逆解 IK：由足端位置求关节角度（elbow_up：0=一个弯向，1=另一弯向，其它=自动择优；失败返回 -1，成功返回 0） */
int8_t Kinematics_Inverse(const FootPosition *foot, LegJointAngles *q, int elbow_up);

/* 雅可比矩阵：由关节角求 J = [∂x/∂q1 ∂x/∂q2; ∂z/∂q1 ∂z/∂q2] */
void jacobian_rz(float q1, float q2, float* J11, float* J12, float* J21, float* J22);

/* ================== 关节限位公开接口 ==================
 * 供轨迹规划等模块在规划阶段校验/钳位用（限位宏见文件开头）。 */

/* 单关节是否在机械限位内：joint = 1 或 2，返回 1=在限位内，0=超限 */
int Kinematics_JointInLimit(float q, int joint);

/* 把关节角组钳位到限位内；返回 1 = 发生过钳位（调用方可据此清零前馈），0 = 未钳位 */
int Kinematics_ClampJoint(LegJointAngles *q);

/* ================== 关节角 <-> 电机角 零点/方向换算 ================== */



/* 关节角 -> 电机角 */
float joint_to_motor_1(float q1);
float joint_to_motor_2(float q2);
float joint_to_motor_3(float q3);

/* 电机角 -> 关节角 */
float motor_to_joint_1(float m1);
float motor_to_joint_2(float m2);
float motor_to_joint_3(float m3);


/* ================== 关节空间限速（摆动速率限制） ==================
 * 运控(MIT)模式的协议里只有 p/v/kp/kd/t，没有"速度上限"参数（v 是前馈速度，
 * 配 kd 起阻尼，不是限速指令），所以限速在主控侧做：
 * 限制每拍关节指令角的增量 = 限速值(rad/s) × dt，从而限制电机摆动速率。
 * 限速值在下面这三个宏里，改这里即可。 */

/* 关节限速（rad/s）：肩/肘 5.0 ≈ 286°/s，腕 1.0 ≈ 57°/s
 * 注意 dt 现在由 DWT 实测（arm_control 实际周期 2ms），所以这组值就是真实的 rad/s 上限 */
#define JOINT_RATE_Q1_MAX   5.0f
#define JOINT_RATE_Q2_MAX   5.0f
#define JOINT_RATE_Q3_MAX   1.0f

/* 期望关节角 -> 每拍增量受限的指令关节角
 *   q_des：IK 解出的期望关节角
 *   dt   ：距上次调用的实测周期 (s)，<=0 时不做限速（首拍/异常周期）
 *   q_cmd：输出限速后的指令关节角（q_cmd 与 q_des 不能是同一个变量） */
void joint_rate_limit(const LegJointAngles *q_des, float dt, LegJointAngles *q_cmd);

/* 复位限速器状态：以上电时的实际关节角作为指令起点，保证首拍不跳变 */
void joint_rate_limit_reset(const LegJointAngles *q_now);

/* ================== 重力补偿（简单杠杆原理） ==================
 * 每个关节的重力矩 = Σ (质量 × g × 该关节到该质量质心的水平距离)
 * 水平距离 = 各段长度 × cos(该段的绝对角度)，即力臂的水平投影。 */

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
} grav_cfg_t;

/* 重力补偿（杠杆原理）：
 *   q1/q2/q3  肩/肘/腕关节角 (rad)，腕关节角 = −EL05 电机反馈角
 *   输出力矩 = dir × (Σ 质量×g×水平力臂) / gear，再经限幅 + 变化率限制平滑
 *            gear 用当前工程数值；实测偏差只需调 kinematics.c 里的 gear
 *   变化率限制的周期由 DWT 实测，需先调用 DWT_Init()
 *   tau_sh/tau_el/tau_wr 可传 NULL 表示不输出 */
void get_gravity_comp_torque(float q1, float q2, float q3,
                             float *tau_sh, float *tau_el, float *tau_wr);

#ifdef __cplusplus
}
#endif

#endif /* __KINEMATICS_H */
