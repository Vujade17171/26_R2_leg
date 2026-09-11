/*****************************************************************************
 * EL05_motor.h —— RobStride EL05 电机驱动接口
 *
 * 依据工程内《EL05使用说明书2600428.pdf》第 4 章（私有协议，扩展帧，1Mbps）：
 *   29 位 ID：bit28~24 通信类型 | bit23~8 数据区2 | bit7~0 目标电机 CAN_ID
 *   通信类型 1：运控模式电机控制指令
 *   通信类型 2：电机反馈数据
 *   通信类型 3：电机使能运行
 *   通信类型 4：电机停止运行（Byte[0]=1 时清故障）
 *   通信类型 6：设置电机机械零位
 * 仅实现说明书上述功能；控制模式仅实现运控模式。
 *****************************************************************************/
#ifndef __EL05_MOTOR_H
#define __EL05_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 返回状态 */
#define EL05_OK           0u   /* 帧已送入发送队列        */
#define EL05_ERR_PARAM    1u   /* 参数非法                */
#define EL05_ERR_MODEL    2u   /* 模型限幅区间异常        */
#define EL05_ERR_BUS      3u   /* CAN 发送失败            */

/* 通信类型（手册 4.1） */
#define EL05_TYPE_GET_ID      0u
#define EL05_TYPE_MOTION      1u   /* 运控模式电机控制指令   */
#define EL05_TYPE_FEEDBACK    2u   /* 电机反馈数据           */
#define EL05_TYPE_ENABLE      3u   /* 电机使能运行           */
#define EL05_TYPE_STOP        4u   /* 电机停止运行           */
#define EL05_TYPE_SET_ZERO    6u   /* 设置电机机械零位       */

/* 电机模型：运控指令/反馈的物理量程（手册 4.4 程序样例宏定义） */
typedef struct {
    const char *name;      /* 型号名                          */
    float p_min, p_max;    /* 位置量程 rad   (-12.57~12.57)   */
    float v_min, v_max;    /* 速度量程 rad/s (-50~50)         */
    float t_min, t_max;    /* 力矩量程 N·m   (-6~6)           */
    float kp_max;          /* Kp 上限 (0~500)                 */
    float kd_max;          /* Kd 上限 (0~5)                   */
} EL05_Model;

/* EL05 内置量程 */
extern const EL05_Model EL05_MODEL;

/* 电机句柄（状态量由通信类型 2 反馈更新） */
typedef struct EL05_Motor {
    void   *bus;                /* 指向 FDCAN_HandleTypeDef   */
    uint8_t id;                 /* 目标电机 CAN_ID            */
    uint8_t master_id;          /* 主机 CAN_ID（反馈帧标识）  */
    const EL05_Model *model;    /* 电机量程模型               */
    volatile float pos_rad;     /* 当前角度 rad               */
    volatile float vel_rads;    /* 当前角速度 rad/s           */
    volatile float torque_nm;   /* 当前力矩 N·m               */
    volatile float temp_c;      /* 当前温度 ℃                */
    volatile uint8_t fault;     /* 故障位 bit0~5（手册 4.1）   */
    volatile uint8_t mode_state;/* 模式状态 0复位/1标定/2运行  */
} EL05_Motor;

/* 初始化：绑定总线、目标电机 CAN_ID、主机 CAN_ID 与量程模型 */
void EL05_Init(EL05_Motor *m, void *fdcan_bus, uint8_t motor_id,
               uint8_t master_id, const EL05_Model *model);

/* 电机使能运行（通信类型 3） */
uint8_t EL05_Enable(EL05_Motor *m);

/* 电机停止运行（通信类型 4） */
uint8_t EL05_Stop(EL05_Motor *m);

/* 电机停止运行并清故障（通信类型 4，Byte[0]=1） */
uint8_t EL05_ClearFault(EL05_Motor *m);

/* 设置电机机械零位（通信类型 6） */
uint8_t EL05_SetZero(EL05_Motor *m);

/* 运控模式控制指令（通信类型 1）：位置/速度/Kp/Kd/前馈力矩 */
uint8_t EL05_MotionControl(EL05_Motor *m, float pos_rad, float vel_rad_s,
                           float kp, float kd, float tor_nm);

/* 接收解析：把一帧通信类型 2 反馈写入指定电机句柄 */
uint8_t EL05_OnCanRx(EL05_Motor *m, uint32_t ext_id,
                     const uint8_t *data, uint8_t len);

/* 接收分发：按帧内"当前电机 CAN_ID"自动匹配 motors[] 并解析 */
uint8_t EL05_RxDispatch(EL05_Motor *motors, uint8_t count,
                        uint32_t ext_id, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __EL05_MOTOR_H */
