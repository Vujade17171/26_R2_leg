#ifndef __JOINT_EL05_H
#define __JOINT_EL05_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_structure.h"

/* ==================== EL05 通信类型（手册 4.1 节） ==================== */
#define EL05_COMM_OPERATION_CONTROL  0x01  /* 运控模式控制指令 */
#define EL05_COMM_STATUS             0x02  /* 电机反馈数据 */
#define EL05_COMM_ENABLE             0x03  /* 电机使能运行 */
#define EL05_COMM_DISABLE            0x04  /* 电机停止运行 */
#define EL05_COMM_SET_ZERO           0x06  /* 设置电机机械零位 */

/* 主机 CAN ID*/
#define EL05_HOST_ID                 0xFF

/* ==================== EL05 MIT 运控模式量程（手册） ==================== */
#define EL05_P_MAX   12.57f   /* 位置 ±12.57 rad (4π) */
#define EL05_P_MIN   (-EL05_P_MAX)
#define EL05_V_MAX   50.0f    /* 速度 ±50 rad/s */
#define EL05_V_MIN   (-EL05_V_MAX)
#define EL05_T_MAX   6.0f     /* 力矩 ±6 Nm */
#define EL05_T_MIN   (-EL05_T_MAX)
#define EL05_KP_MAX  500.0f   /* Kp 0 ~ 500 */
#define EL05_KP_MIN  0.0f
#define EL05_KD_MAX  5.0f     /* Kd 0 ~ 5 */
#define EL05_KD_MIN  0.0f

#define EL05_ID      0x03

/* ==================== 函数声明 ==================== */
extern EL05_Handle_t g_el05;

extern void EL05_Parse_Feedback(EL05_Handle_t *el05, uint32_t ext_id, uint8_t *data);
extern HAL_StatusTypeDef EL05_Motion_Init(EL05_Handle_t *el05, FDCAN_HandleTypeDef *pcan, uint8_t motor_id);
extern HAL_StatusTypeDef EL05_Motion_Enter(EL05_Handle_t *el05);
extern HAL_StatusTypeDef EL05_Motion_Exit(EL05_Handle_t *el05);
extern HAL_StatusTypeDef EL05_Motion_Control(EL05_Handle_t *el05, float position, float speed, float kp, float kd, float torque);

#endif
