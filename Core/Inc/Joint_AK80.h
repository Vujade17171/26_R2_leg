#ifndef __JOINT_AK80_H
#define __JOINT_AK80_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_structure.h"
#include "bsp_fdcan.h"
#include "string.h"
//宏定义声明：
//需要确认
#define CMD_ENTER_MODE   0xFC
#define CMD_EXIT_MODE    0xFD   
#define CMD_SET_ORIGIN   0xFE  
#define CMD_READ_STATUS  0xFC
#define CLAMP(value, min, max)  (((value) < (min)) ? (min) : (((value) > (max)) ? (max) : (value)))

#define AK80_ID 0x01


/* 运控模式参数限制 (AK60-6规格) */
//需要确认
static const float P_MIN = -12.5f;
static const float P_MAX = 12.5f;
static const float V_MIN = -30.0f;
static const float V_MAX = 30.0f;
static const float T_MIN = -18.0f;
static const float T_MAX = 18.0f;
static const float KP_MIN = 0.0f;
static const float KP_MAX = 500.0f;
static const float KD_MIN = 0.0f;
static const float KD_MAX = 5.0f;

//函数外部调用声明：
extern void parse_motion_feedback(AK_Handle_t*ak,uint8_t *data);
extern HAL_StatusTypeDef AK_Motion_Init(AK_Handle_t*ak,FDCAN_HandleTypeDef *pcan, uint8_t motor_id);
extern HAL_StatusTypeDef AK_Motion_Enter(AK_Handle_t*ak);
extern HAL_StatusTypeDef AK_Motion_Exit(AK_Handle_t*ak);
extern HAL_StatusTypeDef AK_Motion_Control(AK_Handle_t*ak,float position, float speed, float kp, float kd, float torque);

#endif