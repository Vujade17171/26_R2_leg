/**
  ******************************************************************************
  * @file    arm_traj.h
  * @brief   Quintic (5th-order) polynomial point-to-point joint trajectory
  ******************************************************************************
  * Description:
  *   - Soft-start / soft-stop between two 3-DOF joint poses.
  *   - Outputs target position / velocity / acceleration each cycle.
  ******************************************************************************
  */
#ifndef __ARM_TRAJ_H
#define __ARM_TRAJ_H

/* start a quintic trajectory for 3 joints (q0=wrist,q1=shoulder,q2=elbow) */
void arm_traj_start(float q0_s, float q1_s, float q2_s,
                    float q0_e, float q1_e, float q2_e,
                    float T_sec);

/* advance trajectory by dt seconds.
 * Return 1 when finished, 0 while running.
 * Outputs current target pos/vel/acc for the 3 joints. */
int arm_traj_update(float dt,
                    float *q0, float *q1, float *q2,
                    float *v0, float *v1, float *v2);

void arm_traj_stop(void);
int  arm_traj_is_active(void);

#endif /* __ARM_TRAJ_H */
