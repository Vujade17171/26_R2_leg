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

/* ================== 关节角 <-> 电机角 零点/方向换算 ================== */



/* 关节角 -> 电机角 */
float joint_to_motor_1(float q1);
float joint_to_motor_2(float q2);

/* 电机角 -> 关节角 */
float motor_to_joint_1(float m1);
float motor_to_joint_2(float m2);

/* ================== 重力补偿（简单杠杆原理） ==================
 * 每个关节的重力矩 = Σ (质量 × g × 该关节到该质量质心的水平距离)
 * 水平距离 = 各段长度 × cos(该段的绝对角度)，即力臂的水平投影。 */

/* 重力补偿配置：质量、质心力臂、方向、减速比 */
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
    float gear_sh;  /* 肩补偿强度除数：1.0=输出侧口径；调大→补偿变小 */
    float gear_el;  /* 肘补偿强度除数：同上                          */
    float gear_wr;  /* 腕补偿强度除数：同上                          */
} grav_cfg_t;

/* 重力补偿（杠杆原理）：
 *   q1/q2/q3  肩/肘/腕关节角 (rad)，腕关节角 = −EL05 电机反馈角
 *   输出力矩 = dir × (Σ 质量×g×水平力臂) / gear
 *            gear=1.0 即输出侧口径；实测偏差只需调 kinematics.c 里的 gear
 *   tau_sh/tau_el/tau_wr 可传 NULL 表示不输出 */
void get_gravity_comp_torque(float q1, float q2, float q3,
                             float *tau_sh, float *tau_el, float *tau_wr);

#ifdef __cplusplus
}
#endif

#endif /* __KINEMATICS_H */
