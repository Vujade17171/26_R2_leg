/*****************************************************************************
 * ak_motor.h
 * CubeMars AK 系列执行器 CAN 驱动（依据《AK系列产品说明书 v3.2.0》编写）
 *
 * 特点：
 *  1. 纯驱动模块：不改动主工程任何现有文件；
 *  2. 接口全部使用 SI 单位：rad / rad·s⁻¹ / N·m / A；
 *  3. 支持 伺服模式(占空比/电流/刹车/速度/位置/位置-速度) 与 力控 MIT 模式；
 *  4. 接收端解析 0x29(实时状态)/0x2A(位置)/0x2C(启动应答) 反馈帧。
 *
 * 使用前提（工程侧需要自行满足，本模块不代劳）：
 *  - FDCAN/CAN 已初始化且波特率 = 1 Mbps（AK 驱动板默认），
 *    并经 HAL_FDCAN_Start() 启动；
 *  - 接收通路：CubeMX/代码中给 FDCAN 配置 >=1 个扩展帧滤波器
 *    （掩码 0x1FFFFFFF 或按 ID 过滤），RxFIFO0 元素数 >=1；
 *  - 在 FDCAN 接收回调里调用 AK_Motor_OnCanRx()/AK_Motor_RxDispatch()；
 *  - AK45-10 等手册未收录型号的参数(限幅/Kt/极对数/减速比)需自行填实。
 *
 * 作者：项目组
 *****************************************************************************/
#ifndef __AK_MOTOR_H
#define __AK_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------- 状态 / 返回值 ------------------------- */
#define AK_OK           0u  /* 发送入队成功 / 正常            */
#define AK_ERR_PARAM    1u  /* 参数非法                       */
#define AK_ERR_MODEL    2u  /* 电机模型未配置完整(限幅为0等)  */
#define AK_ERR_BUS      3u  /* CAN 发送失败(邮箱满等)         */

/* ------------------- CAN 控制模式 ID（手册 4.1） ------------------ */
#define AK_MODE_DUTY          0u   /* 占空比     */
#define AK_MODE_CURRENT       1u   /* 电流环     */
#define AK_MODE_CURRENT_BRAKE 2u   /* 电流刹车   */
#define AK_MODE_SPEED         3u   /* 速度环     */
#define AK_MODE_POS           4u   /* 位置环     */
#define AK_MODE_ORIGIN        5u   /* 设置原点   */
#define AK_MODE_POS_SPD       6u   /* 位置速度环 */
#define AK_MODE_MIT           8u   /* 力控/MIT   */
#define AK_MODE_DISABLE      15u   /* 电机失能   */
#define AK_MODE_FRAME_CFG    16u   /* 反馈报文设置 */

/* ------------------- 反馈功能 ID（手册 4.3.1） --------------------- */
#define AK_FB_STATE  0x29u   /* 实时状态帧(1~2000Hz 周期上报) */
#define AK_FB_POS32  0x2Au   /* 4 字节位置帧(需 bit15 打开)   */
#define AK_FB_ENTER  0x2Cu   /* 进入伺服模式应答(FA FB FC FD) */

/* 电机失能成功后 0x29 帧 Data[7] 回传的应答值 */
#define AK_DISABLE_ACK 0x77u

/* 0x2A 位置帧 LSB 权重（单位：度；命令侧位置缩放为 x10000，
 * 若手册后续版本改为 x100，修改此宏即可） */
#define AK_POS32_LSB_DEG 0.0001f

/* ------------------------------------------------------------------ */
/* 电机模型：用于限幅 / Kt / 减速比 / 极对数                           */
/* 速度/扭矩限幅与 Kt 来源：手册 4.2 参数范围表                        */
/* ------------------------------------------------------------------ */
typedef struct {
    const char *name;      /* 型号名，如 "AK80-9"          */
    float kv;              /* 电机 KV（记录用）            */
    float kt;              /* 扭矩系数 N·m/A，T=Iq*Kt      */
    float gear;            /* 减速比（输出侧/电机侧）      */
    float pole_pairs;      /* 极对数                        */
    float p_min, p_max;    /* MIT 位置限幅 rad              */
    float v_min, v_max;    /* MIT 速度限幅 rad/s(输出端)   */
    float t_min, t_max;    /* MIT 扭矩限幅 N·m(输出端)     */
    float kp_max, kd_max;  /* MIT 增益上限(0~500 / 0~5)    */
} AK_MotorModel;

/* 内置模型：
 *  - AK80-9：手册 v3.2.0 已收录（Kt=0.5701，速度 ±65 rad/s，
 *    扭矩 ±18 N·m）。极对数/减速比如与实物不符请自行修正。
 *  - AK45-10(KV75)：手册未收录，kt/限幅等关键量未知，默认置 0；
 *    使用前必须用 AK_Motor_MakeModel() 填实，否则 MIT/速度换算
 *    返回 AK_ERR_MODEL。 */
extern const AK_MotorModel AK_MODEL_AK80_9;
extern const AK_MotorModel AK_MODEL_AK45_10;

/* 生成自定义模型（未提供的量填 0；返回 struct 可直接传入 Init） */
AK_MotorModel AK_Motor_MakeModel(const char *name,
                                 float kv, float kt,
                                 float gear, float pole_pairs,
                                 float p_min, float p_max,
                                 float v_min, float v_max,
                                 float t_min, float t_max,
                                 float kp_max, float kd_max);

/* ------------------------------ 电机句柄 ------------------------- */
typedef struct AK_Motor {
    void  *bus;               /* 指向 FDCAN_HandleTypeDef（类型擦除） */
    uint8_t id;               /* 驱动器 ID（CAN ID 低 8 位）          */
    const AK_MotorModel *model;/* 电机模型                            */

    /* ---- 由 0x29 反馈帧更新的状态（SI） ---- */
    volatile float pos_rad;    /* 位置 rad（0x29 帧为 ±3200° 粗量程）  */
    volatile float vel_rads;   /* 速度 rad/s（输出端）                */
    volatile float iq_a;       /* Iq 电流 A                           */
    volatile int8_t  temp_c;   /* 驱动板温度 ℃                       */
    volatile uint8_t err_code; /* 报错码（0 无/1 过温/2 过流/3 过压/
                                  4 欠压/5 编码器/6 MOS过温/7 堵转）  */
    volatile uint8_t flags;    /* 位0: 已收到失能应答(0x77)           */
    volatile int32_t raw_pos32;/* 0x2A 帧原始 int32（x0.0001°）       */
} AK_Motor;

#define AK_FLAG_DISABLE_ACKED 0x01u

/* ------------------------------ API ------------------------------- */
/* 初始化：bus 传 &hfdcan1；model 可传内置模型或自定义模型 */
void    AK_Motor_Init(AK_Motor *m, void *fdcan_bus,
                      uint8_t drive_id, const AK_MotorModel *model);

/* 更换电机模型（运行时换型/改限幅用） */
uint8_t AK_Motor_SetModel(AK_Motor *m, const AK_MotorModel *model);

/* ---- 伺服模式（全部 SI） ---- */
uint8_t AK_Motor_SetDuty(AK_Motor *m, float duty);        /* 占空比 0~±0.95    */
uint8_t AK_Motor_SetCurrent(AK_Motor *m, float amp);      /* 电流环 A          */
uint8_t AK_Motor_SetBrake(AK_Motor *m, float amp);        /* 电流刹车 A        */
uint8_t AK_Motor_SetSpeed(AK_Motor *m, float rad_s);      /* 速度环 rad/s(输出)*/
uint8_t AK_Motor_SetSpeedERPM(AK_Motor *m, int32_t erpm); /* 速度环 ERPM(电气) */
uint8_t AK_Motor_SetPosition(AK_Motor *m, float rad);     /* 位置环 rad(输出)  */
uint8_t AK_Motor_SetPositionVelocity(AK_Motor *m,
                                     float pos_rad,
                                     float vel_rad_s,
                                     float acc_rad_s2);    /* 位置速度环        */
uint8_t AK_Motor_SetOrigin(AK_Motor *m, uint8_t permanent); /* 0 临时零点/1 永久 */

/* ---- 失能与反馈配置 ---- */
uint8_t AK_Motor_Disable(AK_Motor *m);                    /* 电机失能          */
uint8_t AK_Motor_ConfigFeedback(AK_Motor *m, uint16_t cfg); /* bit15=加发0x2A  */
                                                            /* bit14=单圈0~360° */

/* ---- 力控 / MIT 模式 ---- */
/* 三种用法均由发送内容决定：
 *   位置控制: AK_Motor_MIT(m, pos, 0, kp, kd, 0);
 *   速度控制: AK_Motor_MIT(m, 0, vel, 0, kd, 0);
 *   力矩控制: AK_Motor_MIT(m, 0, 0, 0, 0, torque);
 */
uint8_t AK_Motor_MIT(AK_Motor *m, float pos_rad, float vel_rad_s,
                     float kp, float kd, float tor_nm);

/* ---- 接收解析（在 FDCAN RxFifo0 回调中调用） ---- */
/* 单电机：ext_id 为收到的 29 位扩展 ID，data/dlc 为数据与长度 */
uint8_t AK_Motor_OnCanRx(AK_Motor *m, uint32_t ext_id,
                         const uint8_t *data, uint8_t dlc);
/* 多电机广播：自动按驱动器 ID 匹配到对应句柄 */
uint8_t AK_Motor_RxDispatch(AK_Motor *motors, uint8_t count,
                            uint32_t ext_id,
                            const uint8_t *data, uint8_t dlc);

/* ---- 工具换算 ---- */
float AK_ERPM_To_RadS(float erpm, float pole_pairs, float gear); /* 输出端 rad/s */
float AK_RadS_To_ERPM(float rad_s, float pole_pairs, float gear);

#ifdef __cplusplus
}
#endif

#endif /* __AK_MOTOR_H */
