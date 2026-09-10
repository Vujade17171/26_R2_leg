/**
  ******************************************************************************
  * @file    leg_task.c
  * @brief   Leg_Task FreeRTOS 任务实现（空任务骨架）
  *
  * 说明：
  *  - 本文件提供 leg_task() 的强定义，覆盖 CubeMX 在 freertos.c 中生成的
  *    __weak void leg_task(void *argument)；
  *  - 任务配置（CubeMX .ioc）：名称 Leg_Task，优先级 osPriorityLow(8)，
  *    堆栈 256*4 字节，CMSIS-RTOS v2；
  *  - 在下方两个 USER CODE 区域中填写实际控制逻辑即可。
  ******************************************************************************
  */
#include "leg_task.h"
#include "ak_motor.h"
#include "fdcan.h"    
#include "kinematics.h"  /* 腿部运动学接口：FK/IK、关节角<->电机角换算、重力补偿等 */

AK_Motor motors[2];  /* 电机句柄数组，按实际机械布局初始化 */

LegLinkParam leg_link_param = { D_L1, D_L2, D_L3 };  /* 连杆参数：大臂/小臂/腕部长度，单位 m */

LegJointAngles motor_angles = { 0.0f, 0.0f, 0.0f }; //电机角

FootPosition target_pos = { 0.0f, 0.0f }; //末端端位置x,z

FootPosition FK_pos = { 0.0f, 0.0f }; //正解位置x,z



/* 电机数量：供 Mycan 接收回调按 sizeof 自动计算，新增电机无需改这里 */
const uint8_t g_ak_motor_num = (uint8_t)(sizeof(motors) / sizeof(motors[0]));


void leg_task(void *argument)
{


  AK_Motor_Init(&motors[0], &hfdcan1, 1, &AK_MODEL_AK80_9);
  AK_Motor_Init(&motors[1], &hfdcan1, 2, &AK_MODEL_AK45_10);
  AK_Motor_Enable(&motors[0]); 
  AK_Motor_Enable(&motors[1]); 
  Kinematics_Init(&leg_link_param);
  osDelay(10);                  

  for (;;)
  {

    

    osDelay(1);
  }
}
