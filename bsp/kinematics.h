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

/* ================== 重力补偿 ================== */

/* 重力补偿配置：两连杆质点质量、重力加速度、关节减速比 */
typedef struct {
    float m_elbow;       /* 肘关节处集中质量 (kg) */
    float m_wrist;       /* 腕/足端处集中质量 (kg) */
    float g;             /* 重力加速度 (m/s²，通常 9.81) */
    float shoulder_gear; /* 肩关节减速比 */
    float elbow_gear;    /* 肘关节减速比 */
} grav_comp_cfg_t;

/* 计算两连杆在重力下的肩/肘关节力矩，并折算成电机侧力矩。
 * 输入说明：
 *   cfg    重力补偿配置（质量、重力加速度、减速比）
 *   q1     肩关节角 (rad)
 *   q2     肘关节角 (rad)
 *   tau_sh 输出：肩电机侧重力补偿力矩 (N·m)，可传 NULL 表示不输出
 *   tau_el 输出：肘电机侧重力补偿力矩 (N·m)，可传 NULL 表示不输出 */
void get_gravity_torque_motor_cfg(const grav_comp_cfg_t *cfg,
                                  float q1, float q2,
                                  float *tau_sh, float *tau_el);

/* 动力学前馈配置（电机侧力矩前馈） */
typedef struct {
    float m_link1;       /* 大臂连杆质量 (kg)              */
    float lc_link1;      /* 大臂连杆质心距肩关节 (m)       */
    float kd_sh;         /* 肩关节粘性阻尼系数 (N·m·s/rad) */
    float kd_el;         /* 肘关节粘性阻尼系数 (N·m·s/rad) */
    float max_torque_sh; /* 肩电机力矩限幅 (N·m，电机侧)   */
    float max_torque_el; /* 肘电机力矩限幅 (N·m，电机侧)   */
    float rate_limit;    /* 力矩变化率限制 (N·m/s，电机侧) */
} dyn_ff_cfg_t;

/* 动力学前馈力矩计算：重力补偿 + 连杆自重 + 粘性阻尼，
 * 再经限幅与变化率平滑，输出电机侧前馈力矩。
 * （重力配置 grav 为本模块内的常量结构体，不作为入参；
 *   变化率限制用的周期由 DWT 实测，需先调用 DWT_Init()）
 * 输入说明：
 *   q1/q2    肩/肘关节角 (rad)
 *   dq1/dq2  肩/肘关节角速度 (rad/s)，用于粘性阻尼
 *   tau_sh_ff 输出：肩电机侧前馈力矩 (N·m)，可传 NULL
 *   tau_el_ff 输出：肘电机侧前馈力矩 (N·m)，可传 NULL */
void get_dynamic_feedforward_torque(float q1, float q2,
                                    float dq1, float dq2,
                                    float *tau_sh_ff, float *tau_el_ff);

#ifdef __cplusplus
}
#endif

#endif /* __KINEMATICS_H */
