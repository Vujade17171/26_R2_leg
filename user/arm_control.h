/**
  ******************************************************************************
  * @file    arm_control.h
  * @brief   Robot-arm end-position control (3 motors, shared FDCAN bus)
  ******************************************************************************
  * Description:
  *   - Target inputs are modified in Keil debug via volatile variables:
  *       arm_target[0..2] = (x,z,yaw), unit: m, m, rad
  *       arm_cmd_new  = 1 -> execute the new cartesian target.
  *   - arm_dbg gives live feedback for the Watch window.
  ******************************************************************************
  */
#ifndef __ARM_CONTROL_H
#define __ARM_CONTROL_H

#include "stm32h7xx_hal.h"

#define ARM_MOTOR_NUM   3

/* per-joint debug info (Watch arm_dbg in Keil) */
typedef struct
{
    float   angle;    /* current joint angle rad      */
    float   target;   /* current commanded target rad */
} arm_joint_dbg_t;

typedef struct
{
    arm_joint_dbg_t joint[ARM_MOTOR_NUM];  /* [0]=shoulder, [1]=elbow, [2]=wrist */
    float x, z;                           /* current cartesian target            */
    float x_actual;                       /* wrist-centre x from forward kinematics */
    float z_actual;                       /* wrist-centre z from forward kinematics */
    uint8_t reached;                      /* 1 = end-effector at target          */
    int16_t last_err;                     /* 0=ok, -1=unreachable, else code     */
} arm_dbg_t;

extern arm_dbg_t arm_dbg;


/* Keil-debug input variables (modify these in Watch window) */
extern volatile float    arm_target[3];   /* cart:(x,z,yaw) */
extern volatile uint8_t  arm_cmd_new;     /* set 1 to run new target */

void arm_init(FDCAN_HandleTypeDef *hfdcan);
void arm_run(void);                 /* call every ~10 ms */
int  arm_goto(float x, float z, float yaw);

#endif /* __ARM_CONTROL_H */
