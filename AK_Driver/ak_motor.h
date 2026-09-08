/*****************************************************************************
 * ak_motor.h —— CubeMars AK 系列电机模组 运控(MIT)模式 CAN 驱动接口
 *
 * 依据工程内《AK 系列电机模组产品使用说明2.pdf》第 5.3 节（运控模式通讯协议）：
 *   标准帧，ID=电机ID(默认1)，DLC=8；控制前必须先发"进入电机控制模式"特殊码。
 *****************************************************************************/
#ifndef __AK_MOTOR_H
#define __AK_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 返回状态 */
#define AK_OK           0u   /* 帧已成功送入发送队列      */
#define AK_ERR_PARAM    1u   /* 参数非法                  */
#define AK_ERR_MODEL    2u   /* 电机模型限幅区间未配置好  */
#define AK_ERR_BUS      3u   /* CAN 发送失败(队列满等)    */

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
    void  *bus;                 /* 指向 FDCAN_HandleTypeDef    */
    uint8_t id;                 /* 电机 ID（默认 1）           */
    const AK_MotorModel *model; /* 电机模型                    */
    volatile float pos_rad;     /* 位置 rad                    */
    volatile float vel_rads;    /* 速度 rad/s                  */
    volatile float torque_nm;   /* 扭矩 N·m                    */
    volatile int16_t temp_c;    /* 电机温度 ℃（手册 -40~215）  */
    volatile uint8_t err_code;  /* 错误标志                    */
} AK_Motor;

/* 生成自定义电机模型（按手册 5.3 参数表填写限幅） */
AK_MotorModel AK_Motor_MakeModel(const char *name,
                                 float p_min, float p_max,
                                 float v_min, float v_max,
                                 float t_min, float t_max,
                                 float kp_max, float kd_max);

/* 初始化：把电机句柄绑定到 FDCAN 总线与电机 ID */
void AK_Motor_Init(AK_Motor *m, void *fdcan_bus, uint8_t motor_id,
                   const AK_MotorModel *model);

/* 进入电机控制模式（特殊码 0xFF×7+0xFC）：控制前必须先发一次 */
uint8_t AK_Motor_Enable(AK_Motor *m);

/* 退出电机控制模式（特殊码 0xFF×7+0xFD） */
uint8_t AK_Motor_Exit(AK_Motor *m);

/* 设置电机当前位置为 0 点（特殊码 0xFF×7+0xFE） */
uint8_t AK_Motor_SetZero(AK_Motor *m);

/* 运控MIT：按模型限幅打包发送位置/速度/扭矩指令（标准帧，ID=电机ID） */
uint8_t AK_Motor_MIT(AK_Motor *m, float pos_rad, float vel_rad_s,
                     float kp, float kd, float tor_nm);

/* 接收解析：把一帧运控反馈写入指定电机句柄 */
uint8_t AK_Motor_OnCanRx(AK_Motor *m, uint32_t std_id,
                         const uint8_t *data, uint8_t len);

/* 接收分发：按标准帧 ID（=电机ID）自动匹配 motors[] 并解析 */
uint8_t AK_Motor_RxDispatch(AK_Motor *motors, uint8_t count,
                            uint32_t std_id,
                            const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __AK_MOTOR_H */
