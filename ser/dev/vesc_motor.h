#ifndef __VESC_MOTOR_H
#define __VESC_MOTOR_H

#include "bsp_can.h"   /* bsp_can_bus_t */

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_can.h"

typedef enum 
{
	CAN_PACKET_SET_DUTY = 0,							//设置电机占空比
	CAN_PACKET_SET_CURRENT = 1,						//设置电机目标电流值
	CAN_PACKET_SET_CURRENT_BRAKE = 2,			//设置制动电流
	CAN_PACKET_SET_RPM = 3,								//目标转速
	CAN_PACKET_SET_POS = 4,								//目标位置
	
	
	CAN_PACKET_FILL_RX_BUFFER = 5,				//填充接收缓冲区
	CAN_PACKET_FILL_RX_BUFFER_LONG = 6,		//更长的缓冲区
	CAN_PACKET_PROCESS_RX_BUFFER = 7,			//处理接收缓冲区的数据，用于多帧接收
	CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
	CAN_PACKET_STATUS = 9,								//获取电机状态值
	
	CAN_PACKET_SET_CURRENT_REL = 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
	CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
	CAN_PACKET_STATUS_2 = 14,
	CAN_PACKET_STATUS_3 = 15,
	CAN_PACKET_STATUS_4 = 16,
	CAN_PACKET_PING = 17,
	CAN_PACKET_PONG = 18,
	CAN_PACKET_DETECT_APPLY_ALL_FOC = 19,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES = 20,
	CAN_PACKET_CONF_CURRENT_LIMITS = 21,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS = 22,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN = 23,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN = 24,
	CAN_PACKET_CONF_FOC_ERPMS = 25,
	CAN_PACKET_CONF_STORE_FOC_ERPMS = 26,
	CAN_PACKET_STATUS_5 = 27
}CAN_PACKET_ID;

typedef struct
{	
	//这些有点用吧，应该是的吧
	float ERpm;								// 电速度
	float Rpm;								// 机械速度
	float PID_Position_Now;		// 当前PID位置
	float PID_Position_Last;	// 上一次PID位置（用于计算变化量）
	int Cnt;									// 数据接收计数（用于判断通信是否正常）
	float Total_Position;			// 累计位置（总转动角度）
	float Total_Current;			// 总电流（A）
	float Duty;
}VESC_RxData;

typedef struct
{
	bsp_can_bus_t bus;
	uint16_t ID;              // 电机的CAN通信ID（唯一标识）
}VESC_Basic_Parameters;

typedef struct VESC_Motor
{
    VESC_RxData RxData;               				// 接收数据（运行状态）
    VESC_Basic_Parameters Basic_Parameters; 	// 基础参数（CAN配置）

    // 函数指针：电机控制接口（封装不同指令的发送逻辑）
    void (*SetCurrent)(struct VESC_Motor* motor, int32_t current); 													// 设置电流（mA）
    void (*SetRpm)(struct VESC_Motor* motor, int32_t rpm);         													// 设置转速（RPM）      													// 设置占空比（-1~1）
    void (*SetPos)(struct VESC_Motor* motor, int32_t pos);         													// 设置位置
} VESC_Motor;

void VESC_Motor_Struct_Init(VESC_Motor* vesc_motor,bsp_can_bus_t bus,uint16_t id);

void VESC_Set_Rpm_Struct(VESC_Motor* vesc_motor,int32_t RPM);
void VESC_Set_Current_Struct(VESC_Motor* vesc_motor,int32_t current);
void VESC_Set_Pos_Struct(VESC_Motor* vesc_motor,int32_t pos);

uint32_t VESC_Motor_RxDispatch(VESC_Motor *motors, uint8_t count,
                        uint32_t ext_id, const uint8_t *data, uint8_t len);

/* 接收解析：把一帧 VESC 反馈写入指定电机句柄 */
int32_t VESC_Motor_OnCanRx(VESC_Motor *m, uint32_t ext_id,
                           const uint8_t *data, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* __VESC_MOTOR_H */