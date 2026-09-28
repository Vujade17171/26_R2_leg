/*****************************************************************************
 * leg_types.h —— 四层共享的基础类型、单位约定、限位判定与统一返回码
 *
 * 分层位置：common/inc —— 四层共享的"词汇表"，本身【不属于任何一层】。
 *   它只依赖 C 标准库，因此 bsp / osal / ser / app 都能安全包含它，
 *   而不会造成层间依赖（这是把它独立成 common 而不是塞进 ser 的原因）。
 *
 * 约定：
 *  - 单位一律 SI：角度 rad、长度 m、时间 s、力矩 N·m；
 *  - 返回码统一为 SER_OK(0) = 成功，负数 = 错误（SER_ERR_*）。
 *
 * 为什么把"限位判定"放这里：
 *   轨迹规划需要校验终点是否超限，运动学需要择优与钳位。若把限位判定放在
 *   kinematics 里，joint_traj 就必须反向依赖 kinematics（原工程正是如此）。
 *   放到公共头后，两个算法模块只共同依赖本文件，彼此解耦。
 *****************************************************************************/
#ifndef __LEG_TYPES_H
#define __LEG_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 统一返回码（全层通用） =====================
 * 约定：0 = 成功；负 = 错误。
 * 历史问题：旧 ak_motor.h 用 0=OK、1..3=错误，而旧 joint_traj.h 用 1=OK、0=错误，
 * 两套语义正好相反，跨层混用极易出错 —— 本工程统一为下面这一套。 */
#define SER_OK           0
#define SER_ERR_PARAM  (-1)   /* 空指针 / 参数越界                  */
#define SER_ERR_MODEL  (-2)   /* 电机量程模型不合法                 */
#define SER_ERR_BUS    (-3)   /* 总线发送失败（队列满 / 未初始化）   */
#define SER_ERR_LIMIT  (-4)   /* 超机械限位 / 目标不可达 / 参数非法  */
#define SER_ERR_STATE  (-5)   /* 当前状态不允许该操作               */
#define SER_ERR_NOENT  (-6)   /* 未找到匹配对象（ID 不匹配等）       */

/* ===================== 机械/连杆参数 ===================== */
#define D_L1 0.35f  /* 大臂长度 (m) */
#define D_L2 0.25f  /* 小臂长度 (m) */
#define D_L3 0.00f  /* 腕部长度 (m)，暂时忽略 */

/* 连杆长度（单位 m）：由机械设计确定，使用前在 Kinematics_Init 中赋值 */
typedef struct {
    float L1;
    float L2;
    float L3;
} LegLinkParam;

/* ===================== 关节量 ===================== */
/* 单腿 3 关节角：q1 肩(大臂) / q2 肘(小臂) / q3 腕，单位 rad */
typedef struct {
    float q1;
    float q2;
    float q3;
} LegJointAngles;

/* 关节速度/加速度向量：与 LegJointAngles 同形状，靠用法区分（rad/s、rad/s²） */
typedef struct {
    float q1;
    float q2;
    float q3;
} LegJointVec;

/* 足端在腿坐标系中的位置（单位 m） */
typedef struct {
    float x;
    float z;
} FootPosition;

/* ===================== 关节机械限位（rad，按实物机械限位调整） =====================
 * 放公共头是为了让轨迹规划等模块能在"规划阶段"就校验终点是否超限
 * （超限应拒绝规划，而不是先算完再钳位）。 */
#define JOINT1_MIN  (-1.25663704f)
#define JOINT1_MAX  ( 3.1415926f)
#define JOINT2_MIN  (-2.82743334f)
#define JOINT2_MAX  ( 2.82743334f)

/* ===================== 关节限速（rad/s，摆动速率上限） =====================
 * 运控(MIT)协议的 v 是前馈速度、不是限速指令，所以限速在主控侧做：
 * 限制每拍关节指令角的增量 = 限速值 × dt。 */
#define JOINT_RATE_Q1_MAX   5.0f   /* 肩 5.0 ≈ 286°/s */
#define JOINT_RATE_Q2_MAX   5.0f   /* 肘 5.0 ≈ 286°/s */
#define JOINT_RATE_Q3_MAX   1.0f   /* 腕 1.0 ≈  57°/s */

/* ===================== 限位判定（唯一实现） ===================== */

/* 单关节是否在机械限位内：joint = 1 或 2，返回 1=在限位内，0=超限
 * （其它 joint 值按 joint 2 处理，与原 Kinematics_JointInLimit 语义一致） */
static inline int LegJointInLimit(float q, int joint)
{
    if (joint == 1) { return ((q >= JOINT1_MIN) && (q <= JOINT1_MAX)) ? 1 : 0; }
    return ((q >= JOINT2_MIN) && (q <= JOINT2_MAX)) ? 1 : 0;
}

/* 关节角组 q1/q2 是否都在限位内：1=在限位内，0=超限 */
static inline int LegJointsInLimit(const LegJointAngles *q)
{
    if (q == 0) { return 0; }
    return (LegJointInLimit(q->q1, 1) && LegJointInLimit(q->q2, 2)) ? 1 : 0;
}

/* 把 q1/q2 钳位到限位内；返回 1 = 发生过钳位（调用方可据此清零前馈）
 * 注意：规划阶段应"拒绝超限目标"而不是依赖这里钳位 —— 钳位只改位置、不改期望增量，
 * 会让终点前馈不为 0，从而持续顶着限位出力。 */
static inline int LegJointsClamp(LegJointAngles *q)
{
    int clamped = 0;

    if (q == 0) { return 0; }

    if (q->q1 > JOINT1_MAX) { q->q1 = JOINT1_MAX; clamped = 1; }
    if (q->q1 < JOINT1_MIN) { q->q1 = JOINT1_MIN; clamped = 1; }
    if (q->q2 > JOINT2_MAX) { q->q2 = JOINT2_MAX; clamped = 1; }
    if (q->q2 < JOINT2_MIN) { q->q2 = JOINT2_MIN; clamped = 1; }

    return clamped;
}

#ifdef __cplusplus
}
#endif

#endif /* __LEG_TYPES_H */