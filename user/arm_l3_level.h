#ifndef __ARM_L3_LEVEL_H
#define __ARM_L3_LEVEL_H

#include <stdint.h>

/* L3 absolute-level calibration value. */
#define ARM_L3_LEVEL_C   2.9202114f

/* Maximum wrist compensation speed, rad/s. */
#define ARM_L3_MAX_SPEED 4.0f

/* Reset the L3 module and set the current wrist command reference. */
void arm_l3_level_init(float q0_current);

/* Start L3 following and latch the current wrist command reference. */
void arm_l3_level_start(float q0_current);

/* Stop L3 following. */
void arm_l3_level_stop(void);

/* Return 1 when L3 following is active. */
uint8_t arm_l3_level_is_active(void);

/* Update the stored wrist command reference without changing active state. */
void arm_l3_level_set_q0_cmd(float q0);

/* Compute the wrist target that keeps L3 level.
 * q0_ref is used to select the nearest equivalent angle. */
float arm_l3_level_target(float q1, float q2, float q0_ref);

/* Update the L3 wrist command.
 * q1, q2: current/commanded shoulder and elbow angles, rad
 * v1, v2: shoulder and elbow velocities, rad/s
 * max_step: maximum change of q0 in this control cycle, rad
 * v0: output wrist velocity feed-forward, rad/s
 * return: updated q0 command, rad
 */
float arm_l3_level_update(float q1, float q2,
                          float v1, float v2,
                          float max_step,
                          float *v0);

#endif /* __ARM_L3_LEVEL_H */
