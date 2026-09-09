/**
  ******************************************************************************
  * @file    arm_kinematics.h
  * @brief   Planar 2R arm kinematics (inverse / reachability / joint-motor map)
  ******************************************************************************
  * Description:
  *   - XZ-plane 2R inverse kinematics (elbow-up branch).
  *   - Joint<->motor angle conversion using config offsets (reference project).
  *   - Joint limit table (reference project parameters).
  *   - q0 = wrist (yaw), q1 = shoulder(da bi), q2 = elbow(xiao bi).
  ******************************************************************************
  */
#ifndef __ARM_KINEMATICS_H
#define __ARM_KINEMATICS_H

#include <stdint.h>

#define ARM_L1  0.35f   /* upper arm length (m)  */
#define ARM_L2  0.25f   /* forearm length (m)    */
#define ARM_L3  0.10f   /* wrist/link end (m)    */

/* joint limit table (q0=wrist, q1=shoulder, q2=elbow) */
typedef struct
{
    float q_min[3];
    float q_max[3];
} arm_joint_limit_t;

extern arm_joint_limit_t g_arm_joint_limit;

/* joint angle -> motor angle (includes direction/offset) */
float arm_joint_to_motor_1(float q1);   /* shoulder */
float arm_joint_to_motor_2(float q2);   /* elbow   */

/* motor angle -> joint angle (for feedback display) */
float arm_motor_to_joint_1(float m1);
float arm_motor_to_joint_2(float m2);

/* reachability check: return 0 if (x,z) reachable, -1 otherwise */
int arm_reachable(float x, float z);

/* planar 2R IK (elbow-up). Input: x,z in meters, yaw in rad (wrist).
 * Output: q0=yaw, q1=shoulder, q2=elbow (rad).
 * Return 0 on success, -1 if unreachable. */
int arm_inverse(float x, float z, float yaw,
                float *q0, float *q1, float *q2);

/* clamp joint angles in place */
void arm_clamp_joints(float *q0, float *q1, float *q2);

/* is joint q within limit of index idx (0..2)? */
int arm_in_limit(float q, int idx);

#endif /* __ARM_KINEMATICS_H */
