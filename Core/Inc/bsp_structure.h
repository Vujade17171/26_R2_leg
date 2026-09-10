#ifndef __BSP_STRUCTURE_H
#define __BSP_STRUCTURE_H

#include "main.h"
#include "cmsis_os.h"

/* ====================AK系列结构体 ==================== */
/**
  * @brief AK 电机参数结构体
  * @note  定义电机的软限位和保护参数范围
*/
typedef struct {
    float pos_min;          /**< 位置下限 (rad)，软限位保护 */
    float pos_max;          /**< 位置上限 (rad)，软限位保护 */
    float vel_min;          /**< 速度下限 (rad/s)，负值表示反转最大速度 */
    float vel_max;          /**< 速度上限 (rad/s)，正值表示正转最大速度 */
    float tor_min;          /**< 转矩下限 (Nm) */
    float tor_max;          /**< 转矩上限 (Nm) */
    float kp_min;           /**< 位置环KP最小值，用于动态调参范围限制 */
    float kp_max;           /**< 位置环KP最大值 */
    float kd_min;           /**< 位置环KD最小值 */
    float kd_max;           /**< 位置环KD最大值 */
} AK_MotorParams_t;

/**
  * @brief AK 电机实时状态结构体
  * @note  存储从电机反馈读取的实时运行数据
  */
typedef struct {
    float position;          /**< 当前转子位置 (rad)，范围一般为 0~2π */
    float speed;             /**< 当前转子速度 (rad/s)，正值为正转 */
    float current;           /**< 当前相电流有效值 (A) */
    int8_t temperature;      /**< 当前电机温度 (℃)，有符号数支持负温显示 */
    uint8_t error_code;      /**< 错误代码，详见产品手册错误码定义 */
    uint8_t id;              /**< 电机ID，CAN总线从站地址 (1~32) */
} AK_MotorStatus_t;
/**
  * @brief AK 电机控制句柄结构体
  * @note  集成所有电机相关资源，用于管理单个AK80电机实例
  */
typedef struct {
    FDCAN_HandleTypeDef *pcan_handle;  /**< FDCAN外设句柄指针，指向对应CAN接口 */
    uint8_t motor_id;                  /**< 电机ID号 (1~32)，必须与硬件拨码一致 */
    uint8_t motor_type;               /**< 电机型号标识，0x01为AK80-9标准版 */
    uint8_t is_initialized;           /**< 初始化标志，0=未初始化，1=已初始化 */
    uint8_t is_entered;               /**< 电机使能标志，0=未进入运行态，1=已使能 */
    AK_MotorParams_t params;        /**< 电机参数配置，包含软限位和保护阈值 */
    AK_MotorStatus_t status;        /**< 电机实时状态，周期性更新 */
} AK_Handle_t;


/* ====================位置坐标结构体 ==================== */
typedef struct {
    float x;              // 腕x坐标 
    float y;              // 末端y坐标
    float z;              // 腕z坐标 
	  float x_s;            // 末端x坐标
	  float z_s;            // 末端z坐标
    float x_speed;        // 末端x速度
    float y_speed;        // 末端y速度
    float z_speed;        // 末端z速度
    float target_x;       // 目标x坐标
    float target_y;       // 目标y坐标
    float target_z;       // 目标z坐标
    float yaw;            // 云台角度
} leg_pos_t;

/* ==================== EL05 结构体 ==================== */
typedef struct {
    float pos_min; float pos_max;
    float vel_min; float vel_max;
    float tor_min; float tor_max;
    float kp_min;  float kp_max;
    float kd_min;  float kd_max;
} EL05_MotorParams_t;

typedef struct {
    float position;      /* 当前位置 rad */
    float speed;         /* 当前速度 rad/s */
    float torque;        /* 当前力矩 Nm */
    float temperature;   /* 温度 ℃ */
    uint8_t mode_state;  /* 模式状态：0=Reset 1=Cali 2=Motor */
    uint8_t fault;       /* 故障位：bit0欠压 bit1过流 bit2过温 bit3磁编码 bit4堵转 bit5未标定 */
    uint8_t id;
} EL05_MotorStatus_t;

typedef struct {
    FDCAN_HandleTypeDef *pcan_handle;
    uint8_t motor_id;
    uint8_t host_id;
    uint8_t is_initialized;
    uint8_t is_entered;
    EL05_MotorParams_t params;
    EL05_MotorStatus_t status;
} EL05_Handle_t;


#endif