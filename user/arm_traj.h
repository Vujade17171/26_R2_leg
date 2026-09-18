/**
  ******************************************************************************
  * @file    arm_traj.h
  * @brief   五次多项式点到点关节轨迹
  ******************************************************************************
  * 说明：
  *   - 在两个三自由度关节位姿之间实现平滑起停；
  *   - 每个控制周期输出 3 个关节的目标位置和速度；
  *   - 不包含正/逆运动学、CAN 通信或重力前馈。
  ******************************************************************************
  */
#ifndef ARM_TRAJ_H
#define ARM_TRAJ_H

/* 启动五次多项式轨迹（q0=腕关节，q1=肩关节，q2=肘关节）。 */
void arm_traj_start(float q0_start, float q1_start, float q2_start,
                    float q0_end, float q1_end, float q2_end,
                    float duration_s);

/* 按 dt 秒推进轨迹。
 * 运行中返回 0，轨迹完成时返回 1。
 * 输出指针允许为 NULL；非 NULL 时写入对应的位置或速度。 */
int arm_traj_update(float dt,
                    float *q0, float *q1, float *q2,
                    float *v0, float *v1, float *v2);

/* 停止当前轨迹。 */
void arm_traj_stop(void);

/* 轨迹正在运行时返回非 0，否则返回 0。 */
int arm_traj_is_active(void);

#endif /* ARM_TRAJ_H */
