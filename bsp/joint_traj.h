/*****************************************************************************
 * joint_traj.h —— 关节空间五次多项式轨迹（归一化 s 曲线：0 → 1）
 *
 * 数学：τ = t/T，s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
 *   s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0 → 起停平滑、端点零速零加速
 *   峰值：s'  峰值 = 1.875  → 关节峰值速度   = 1.875·Δ/T
 *         s'' 峰值 = 5.7735 → 关节峰值加速度 = 5.7735·Δ/T²
 *   两个峰值可用来反推时长：要不超过限速，需 T ≥ 1.875·Δ/限速值
 *   （JointTraj_SuggestDuration 就是干这个的）
 *
 * 约定：
 *  - 输入输出一律【关节侧】量：角度 rad、角速度 rad/s、角加速度 rad/s²；
 *    关节角 <-> 电机角的换算由调用方在"下发处"完成，本模块不碰电机量，也不发 CAN；
 *  - 起点必须由调用方传【实际反馈关节角】，保证首拍不跳变；
 *  - 终点超机械限位直接拒绝规划，不做钳位（钳位只改位置、不改期望增量，
 *    会让 v/a 前馈在终点不为 0，从而持续顶着限位出力）；
 *  - 本模块不依赖 CAN / 电机结构体 / 任何全局变量。
 *****************************************************************************/
#ifndef __JOINT_TRAJ_H
#define __JOINT_TRAJ_H

#include <stdint.h>
#include "kinematics.h"   /* LegJointAngles、JOINT*_MIN/MAX、JOINT_RATE_*_MAX */

#ifdef __cplusplus
extern "C" {
#endif

/* 状态/返回码（风格对齐 ak_motor.h 的 AK_OK 系列） */
#define JTRAJ_ERR_PARAM   0u   /* Plan：参数非法（空指针 / duration 过小）  */
#define JTRAJ_OK          1u   /* Plan：规划成功，开始运行                  */
#define JTRAJ_RUNNING     2u   /* Update：运行中                            */
#define JTRAJ_DONE        3u   /* Update：已到位并保持（幂等，每拍都返回）   */
#define JTRAJ_IDLE        4u   /* Update：无轨迹，出参不写                  */
#define JTRAJ_ERR_LIMIT   5u   /* Plan：终点超机械限位，拒绝规划            */

/* 三元素向量：v_des 用 rad/s，a_des 用 rad/s²（同形状，靠用法区分） */
typedef struct {
    float q1;
    float q2;
    float q3;
} JointTrajVec;

/* 轨迹句柄：一个实例 = 一条腿的一段运动 */
typedef struct {
    uint8_t state;       /* JTRAJ_RUNNING / JTRAJ_DONE / JTRAJ_IDLE */
    float   elapsed;     /* 已运行时间 (s)                          */
    float   duration;    /* 计划总时长 (s)                          */
    float   q_start[3];  /* 起点关节角 (rad)，0=q1, 1=q2, 2=q3      */
    float   q_end[3];    /* 终点关节角 (rad)                        */
} JointTraj;

/* 规划一条轨迹：以 q_now 为起点、q_goal 为终点，duration_s 秒内跑完。
 *   q_now 必须是【实际反馈关节角】（否则首拍跳变）
 *   q_goal.q3 建议直接传 q_now.q3（腕不进轨迹 → Δ3=0，恒保持规划时刻的角度）
 *   返回 JTRAJ_OK 或 JTRAJ_ERR_PARAM / JTRAJ_ERR_LIMIT（被拒时状态保持 IDLE） */
uint8_t JointTraj_Plan(JointTraj *t, const LegJointAngles *q_now,
                       const LegJointAngles *q_goal, float duration_s);

/* 每拍调用一次，推进轨迹并给出本拍的目标量。
 *   dt    ：实测控制周期 (s)，<=0 时不推进（防回退/除零），>50ms 按 50ms 处理
 *   q_des ：本拍目标关节角（腕通道 = q_start[2] + Δ3·s，当前 Δ3=0）
 *   v_des / a_des ：本拍前馈速度、前馈加速度
 *   返回 JTRAJ_RUNNING / JTRAJ_DONE / JTRAJ_IDLE（IDLE 时出参未被写入） */
uint8_t JointTraj_Update(JointTraj *t, float dt,
                         LegJointAngles *q_des,
                         JointTrajVec *v_des, JointTrajVec *a_des);

/* 放弃当前轨迹：状态回 IDLE（故障、急停、重新规划前调用） */
void JointTraj_Abort(JointTraj *t);

/* 纯计算辅助：给定位移与时长，算关节峰值速度 = 1.875·|delta|/T（不碰状态） */
float JointTraj_PeakVel(float delta, float duration);

/* 纯计算辅助：按关节限速反推一个"不会被限速器削"的最短时长（不碰状态）
 *   min_T 为下限（例如 0.3s）；返回值为建议时长，调用方可再乘余量系数 */
float JointTraj_SuggestDuration(float d1, float d2, float min_T);

#ifdef __cplusplus
}
#endif

#endif /* __JOINT_TRAJ_H */
