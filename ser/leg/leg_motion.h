/*****************************************************************************
 * leg_motion.h —— 腿部运动服务（ser/leg）
 *
 * 分层位置：ser/leg —— 【服务层：运动编排】。
 *   它把散落在旧 leg_task.c 里的"关节层"逻辑收拢成可复用服务：
 *     反馈快照 -> 目标管理(规划/锁存) -> 轨迹推进 -> 关节限速
 *       -> 重力补偿 -> MIT 下发
 *   允许依赖：ser/algo、ser/dev、common
 *   禁止依赖：HAL、FreeRTOS、app、任何全局变量
 *
 * 关键改动（相对旧 leg_task.c）：
 *  1) 所有"上一拍"状态（轨迹、限速器、重力补偿、目标锁存、反馈快照）
 *     全部收进 LegMotion 句柄 —— 模块内零可变全局量，第二条腿可直接复用；
 *  2) 不再自己去读 DWT：dt 由调用方（app，经 osal）实测后传入；
 *  3) 设备句柄由 app 注入，本模块不定义 motors[] 之类的全局实例；
 *  4) 腕部（EL05）作为独立时隙的 WristStep()，保留原实现的节拍位置。
 *****************************************************************************/
#ifndef __LEG_MOTION_H
#define __LEG_MOTION_H

#include <stdint.h>
#include "leg_types.h"
#include "kinematics.h"
#include "joint_traj.h"
#include "motion_alg.h"
#include "ak_motor.h"
#include "el05_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 运动服务配置（数值与旧 leg_task.c 顶部宏一致） */
typedef struct {
    float traj_duration_s;    /* 默认规划时长 (s)                */
    float target_eps;         /* 目标点变化判定阈值 (m)          */
    float dt_fallback_s;      /* DWT 异常时的兜底周期 (s)        */
    float dt_max_s;           /* 实测周期上限 (s)                */
    float kp, kd;             /* 正常跟踪刚度/阻尼               */
    float fault_hold_kp;      /* 故障保持刚度（比正常低）        */
    float fault_hold_kd;      /* 故障保持阻尼                    */
    float wrist_kp;           /* EL05 腕部 Kp                    */
    float wrist_kd;           /* EL05 腕部 Kd                    */
} LegMotionCfg;

extern const LegMotionCfg LEG_MOTION_CFG_DEFAULT;

/* 腿部运动服务句柄：一个实例 = 一条腿 */
typedef struct {
    /* ---- 设备句柄（由 app 注入；本模块不定义实例） ---- */
    AK_Motor   *ak;
    uint8_t     ak_num;
    EL05_Motor *el05;
    uint8_t     el05_num;

    /* ---- 配置 ---- */
    LegMotionCfg cfg;
    LegLinkParam link;          /* 连杆参数（同时也交给 Kinematics_Init） */

    /* ---- 算法句柄 ---- */
    JointTraj        traj;
    JointRateLimiter ratelim;
    GravityComp      grav;

    /* ---- 目标管理 ---- */
    FootPosition target;                /* 已登记的目标点            */
    FootPosition target_planned;        /* 已规划的目标（用于去重）  */
    FootPosition target_pending;        /* 运行中锁存的新目标        */
    uint8_t      target_pending_valid;
    float        move_duration_s;       /* 本次运动的规划时长        */
    int32_t      plan_result;           /* 最近一次规划结果 SER_*    */

    /* ---- 本拍状态（同时作为 app 的 Watch 观测快照） ---- */
    LegJointAngles fb;         /* 反馈关节角快照（q3 = -EL05 电机角） */
    LegJointAngles q_cmd;      /* 限速后的指令关节角                  */
    FootPosition   foot_fb;    /* FK(反馈)                            */
    LegJointVec    v_cmd;      /* 本拍前馈速度 rad/s                  */
    LegJointVec    a_cmd;      /* 本拍前馈加速度 rad/s²               */
    float          tau_wr;     /* 腕重力补偿力矩（电机侧 N·m）         */
    float          traj_tau;   /* 轨迹进度 0~1                        */
    uint8_t        traj_state; /* JTRAJ_IDLE / RUNNING / DONE         */
    uint8_t        fault;      /* 非 0 = 本拍处于故障保持              */
} LegMotion;

/* 初始化：注入设备句柄、连杆参数与配置（cfg/grav_cfg 传 NULL 用默认值）。
 * 内部会调用 Kinematics_Init()，app 不必再单独初始化运动学。 */
int32_t LegMotion_Init(LegMotion *lm, AK_Motor *ak, uint8_t ak_num,
                       EL05_Motor *el05, uint8_t el05_num,
                       const LegLinkParam *link,
                       const LegMotionCfg *cfg, const GravCfg *grav_cfg);

/* 上电初始化姿态：以"当前实际末端位置"为目标并复位限速器（首拍不跳变）。
 * 必须在电机已使能、且已收到驱动板反馈之后调用。 */
int32_t LegMotion_InitPose(LegMotion *lm);

/* 登记笛卡尔目标点 x,z（只登记，不规划；实际规划在 Step 中按目标变化触发）。
 * duration_s > 0 时同时更新本次运动的规划时长；<=0 表示沿用当前时长。
 * 轨迹运行中重复调用只会锁存最新目标，等本段跑完再执行。 */
void LegMotion_SetTarget(LegMotion *lm, float x, float z, float duration_s);

/* 主循环每拍（AK 侧）：故障检查 -> 反馈 -> 目标管理 -> 轨迹推进
 *   -> 限速 -> 重力补偿 -> MIT 下发。
 * dt 为实测控制周期 (s)，由调用方经 osal 取得。 */
int32_t LegMotion_Step(LegMotion *lm, float dt);

/* 两次节拍之间（EL05 腕侧）：按当前反馈下发腕部位置与补偿力矩。
 * 保持与旧实现相同的时隙（AK 帧 -> 1ms -> 腕帧 -> 1ms）。 */
int32_t LegMotion_WristStep(LegMotion *lm);

/* 放弃当前轨迹（急停 / 重新规划前调用） */
void LegMotion_Abort(LegMotion *lm);

#ifdef __cplusplus
}
#endif

#endif /* __LEG_MOTION_H */