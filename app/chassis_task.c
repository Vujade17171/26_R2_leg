#include "chassis_task.h"
#include "vesc_motor.h"
#include "chassis_config.h"
#include "can_router.h"
#include "osal_time.h"

VESC_Motor vesc_motor[APP_VESC_NUM];

#define VESC_MOTOR_NUM    ((uint8_t)(sizeof(vesc_motor) / sizeof(vesc_motor[0])))

void chassis_task(void *argument)
{
    (void)argument;
    
    VESC_Motor_Struct_Init(&vesc_motor[0], BSP_CAN_BUS_2, 91);
    VESC_Motor_Struct_Init(&vesc_motor[1], BSP_CAN_BUS_2, 103);
    VESC_Motor_Struct_Init(&vesc_motor[2], BSP_CAN_BUS_2, 121);
    VESC_Motor_Struct_Init(&vesc_motor[3], BSP_CAN_BUS_2, 80);
    (void)VESC_CAN_Router_Init(BSP_CAN_BUS_2);
    CAN_Router_VESC(vesc_motor, VESC_MOTOR_NUM);
    for (;;)
    {
        
        OSAL_Time_DelayMs(1u);
    }
}