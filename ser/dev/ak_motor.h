/*****************************************************************************
 * ak_motor.h —— CubeMars AK 系列电机模组 运控(MIT)模式 CAN 驱动接口
 *
 * 分层位置：ser/dev —— 【设备服务层：协议】。
 *   允许依赖：bsp（只拿总线编号）、common
 *   禁止依赖：HAL 具体类型、FreeRTOS、app 目录
 *   依据：工程内《AK 系列电机模组产品使用说明2.pdf》第 5.3 节
 *        标准帧，ID=电机ID(默认1)，DLC=8；控制前必须先发"进入电机控制模式"特殊码。
 *
 * 与旧版的区别：句柄里的 bus 由 void*（强转 FDCAN_HandleTypeDef*）改为 bsp 的
 *   总线编号，本模块因此不再包含 stm32h7xx_hal.h；返回码统一为 SER_* 体系。
 *****************************************************************************/
#ifndef __AK_MOTOR_H
#define __AK_MOTOR_H

#include <stdint.h>
#include "leg_types.h"
#include "bsp_can.h"   /* bsp_can_bus_t：驱动只说总线编号，不碰 HAL 类型 */

#ifdef __cplusplus
extern "C" {
#endif

/* 电机模型：位置/速度/扭矩限幅（手册 5.3 参数范围表） */
typedef struct {
    const char *name;      /* 型号名                    */
    float p_min, p_max;    /* 位置限幅 rad              */
    float v_min, v_max;    /* 速度限幅 rad/s            */
    float t_min, t_max;    /* 扭矩限幅 N·m              */
    float kp_max, kd_max;  /* Kp/Kd 上限（0~500 / 0~5） */
} AK_MotorModel;

/* 手册 5.3 内置参数 */
extern const AK_MotorModel AK_MODEL_AK80_9;   /* ±12.5rad, ±50rad/s, ±18N·m */
extern const AK_MotorModel AK_MODEL_AK45_10;  /* ±12.5rad, ±20rad/s, ±8N·m  */

/* 电机句柄（状态量由运控反馈帧自动更新） */
typedef struct AK_Motor {
    bsp_can_bus_t bus;          /* bsp 总线编号（不是 HAL 句柄）  */
    uint8_t id;                 /* 电机 ID（默认 1）             */
    const AK_MotorModel *model; /* 电机模型                      */
    volatile float pos_rad;     /* 位置 rad                      */
    volatile float vel_rads;    /* 速度 rad/s                    */
    volatile float torque_nm;   /* 扭矩 N·m                      */
    volatile int16_t temp_c;    /* 电机温度 ℃（手册 -40~215）    */
    volatile uint8_t err_code;  /* 错误标志                      */
} AK_Motor;

/* 生成自定义电机模型（按手册 5.3 参数表填写限幅） */
AK_MotorModel AK_Motor_MakeModel(const char *name,
                                 float p_min, float p_max,
                                 float v_min, float v_max,
                                 float t_min, float t_max,
                                 float kp_max, float kd_max);

/* 初始化：把电机句柄绑定到总线编号与电机 ID */
int32_t AK_Motor_Init(AK_Motor *m, bsp_can_bus_t bus, uint8_t motor_id,
                      const AK_MotorModel *model);

/* 进入电机控制模式（特殊码 0xFF×7+0xFC）：控制前必须先发一次 */
int32_t AK_Motor_Enable(AK_Motor *m);

/* 退出电机控制模式（特殊码 0xFF×7+0xFD） */
int32_t AK_Motor_Exit(AK_Motor *m);

/* 设置电机当前位置为 0 点（特殊码 0xFF×7+0xFE） */
int32_t AK_Motor_SetZero(AK_Motor *m);

/* 运控MIT：按模型限幅打包发送位置/速度/扭矩指令（标准帧，ID=电机ID） */
int32_t AK_Motor_MIT(AK_Motor *m, float pos_rad, float vel_rad_s,
                     float kp, float kd, float tor_nm);

/* 接收解析：把一帧运控反馈写入指定电机句柄 */
int32_t AK_Motor_OnCanRx(AK_Motor *m, uint32_t std_id,
                         const uint8_t *data, uint8_t len);

/* 接收分发：按标准帧 ID（=电机ID）自动匹配 motors[] 并解析 */
int32_t AK_Motor_RxDispatch(AK_Motor *motors, uint8_t count, uint32_t std_id,
                            const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __AK_MOTOR_H */