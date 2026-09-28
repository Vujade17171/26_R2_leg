/*****************************************************************************
 * kinematics.h —— 腿部运动学解算接口（FK / IK / 雅可比 / 关节-电机换算）
 *
 * 分层位置：ser/algo —— 【服务层：算法】。纯计算，可在 PC 上单测。
 *   允许依赖：common（leg_types.h）、C 标准库
 *   禁止依赖：HAL、FreeRTOS、bsp、任何全局句柄
 *
 * 本次拆分：原 kinematics.c 同时承担了"运动学 + 关节限速 + 重力补偿"三件事，
 *   现已按变化原因拆开：
 *     - 运动学（机械尺寸相关）          -> 本文件
 *     - 关节限速（摆动安全相关）        -> motion_alg.h 的 JointRateLimiter
 *     - 重力补偿（负载配平相关）        -> motion_alg.h 的 GravityComp
 *
 * 约定：
 *  - 单位一律 SI：角度 rad、长度 m；
 *  - 腿模型为二连杆 XZ 平面（L3 暂忽略）；
 *  - 限位宏与限位判定（LegJointInLimit / LegJointsClamp）在 leg_types.h。
 *****************************************************************************/
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>
#include "leg_types.h"   /* LegLinkParam / LegJointAngles / FootPosition / 限位宏 */

/* 本头文件只做声明，不包含 <math.h>：cosf/sinf 等由 kinematics.c 自己直接包含。
 * 避免"靠别的头文件传递包含"这种隐患 —— GCC 常悄悄带进来，ARMCC 会直接
 * 报 #20: identifier "xxx" is undefined。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化：写入连杆参数（长度等） */
int32_t Kinematics_Init(const LegLinkParam *param);

/* 正解 FK：由关节角度求足端位置 */
void Kinematics_Forward(const LegJointAngles *q, FootPosition *foot);

/* 逆解 IK：由足端位置求关节角度
 *   elbow_up：0=一个弯向，1=另一弯向，其它=两个构型都试并按限位+连续性择优
 *   返回 SER_OK / SER_ERR_PARAM / SER_ERR_LIMIT（不可达或无满足限位的解） */
int32_t Kinematics_Inverse(const FootPosition *foot, LegJointAngles *q, int elbow_up);

/* 雅可比矩阵：由关节角求 J = [∂x/∂q1 ∂x/∂q2; ∂z/∂q1 ∂z/∂q2] */
void jacobian_rz(float q1, float q2, float *J11, float *J12, float *J21, float *J22);

/* ================== 关节角 <-> 电机角 零点/方向换算 ==================
 * 偏移量是"机械标定参数"（零位时电机读数），不是算法参数，
 * 实测变更时改 kinematics.c 顶部两个常量即可。 */

/* 关节角 -> 电机角 */
float joint_to_motor_1(float q1);
float joint_to_motor_2(float q2);
float joint_to_motor_3(float q3);

/* 电机角 -> 关节角 */
float motor_to_joint_1(float m1);
float motor_to_joint_2(float m2);
float motor_to_joint_3(float m3);

#ifdef __cplusplus
}
#endif

#endif /* __KINEMATICS_H */