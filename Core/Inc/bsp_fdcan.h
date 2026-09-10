#ifndef __BSP_FDCAN_H
#define __BSP_FDCAN_H

#include "main.h"
#include "cmsis_os.h"
#include "bsp_structure.h"
#include "Joint_AK80.h"
//参数调用声明：
extern FDCAN_HandleTypeDef hfdcan1;



//函数调用声明：
extern void can_filter_init(void);
extern HAL_StatusTypeDef can_send_data(AK_Handle_t*ak,uint32_t cob_id, uint8_t *data, uint8_t len);
extern HAL_StatusTypeDef can_send_ext_data(FDCAN_HandleTypeDef *pcan, uint32_t ext_id, uint8_t *data, uint8_t len);


//结构体调用声明：
extern AK_Handle_t g_ak80;
extern AK_Handle_t g_ak45;
#endif