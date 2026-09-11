/*****************************************************************************
 * EL05_motor.c —— RobStride EL05 电机驱动实现（私有协议 / 扩展帧 / 运控模式）
 *
 * 依据工程内《EL05使用说明书2600428.pdf》第 4 章：
 *   29 位 ID：bit28~24 通信类型 | bit23~8 数据区2 | bit7~0 目标电机 CAN_ID
 *   通信类型 1 运控：数据区2=力矩(16bit)，数据区＝位置/速度/Kp/Kd（各16bit，高字节在前）
 *   通信类型 2 反馈：数据区2 低字节=电机CAN_ID、高字节=故障(bit0~5)+模式(bit6~7)
 *   通信类型 3 使能 / 4 停止 / 6 设机械零位
 * 量程（手册 4.4）：位置 ±12.57rad、速度 ±50rad/s、力矩 ±6N·m、
 *                   Kp 0~500、Kd 0~5；转换用 float_to_uint/x_max 归一化。
 *****************************************************************************/
#include "EL05_motor.h"

#include "stm32h7xx_hal.h"

/* EL05 内置量程（手册 4.4 程序样例宏定义） */
const EL05_Model EL05_MODEL = {
    "EL05", -12.57f, 12.57f, -50.0f, 50.0f, -6.0f, 6.0f, 500.0f, 5.0f
};

/* 组装 29 位扩展 ID：通信类型(5bit) | 数据区2(16bit) | 目标电机 CAN_ID(8bit) */
#define EL05_MAKE_ID(type, data16, id)                                 \
    ((((uint32_t)(type)   & 0x1Fu)   << 24) |                          \
     (((uint32_t)(data16) & 0xFFFFu) << 8)  |                          \
     ( (uint32_t)(id)     & 0xFFu))

/* 区间钳位 */
static float EL05_FClamp(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

/* float -> 定点（手册 float_to_uint：归一化到 (1<<bits)-1） */
static uint16_t EL05_Float_To_Uint(float x, float x_min, float x_max, uint32_t bits)
{
    float span = x_max - x_min;
    uint32_t max_u = (1u << bits) - 1u;
    if (span <= 0.0f) { return 0u; }
    x = EL05_FClamp(x, x_min, x_max);
    return (uint16_t)((x - x_min) * ((float)max_u / span));
}

/* 定点 -> float（与上面互逆） */
static float EL05_Uint_To_Float(uint16_t x_int, float x_min, float x_max, uint32_t bits)
{
    float span = x_max - x_min;
    uint32_t max_u = (1u << bits) - 1u;
    if (max_u == 0u) { return x_min; }
    return ((float)x_int) * span / ((float)max_u) + x_min;
}

static uint16_t EL05_Read_U16(const uint8_t *d)  /* 高字节在前 */
{
    return (uint16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);
}

static void EL05_Write_U16(uint8_t *d, uint16_t v)
{
    d[0] = (uint8_t)(v >> 8);
    d[1] = (uint8_t)(v & 0xFFu);
}

/* 底层发送：扩展帧、经典 CAN、DLC=8 */
static uint8_t EL05_Send(void *bus, uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)bus;
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t buf[8], i;

    if (hfdcan == 0) { return EL05_ERR_PARAM; }
    if (len > 8u) { len = 8u; }
    for (i = 0u; i < 8u; i++) { buf[i] = (i < len) ? data[i] : 0u; }

    tx.Identifier          = ext_id & 0x1FFFFFFFu;
    tx.IdType              = FDCAN_EXTENDED_ID;   /* 手册 4：扩展帧 */
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = (uint32_t)len;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, buf) != HAL_OK) {
        return EL05_ERR_BUS;
    }
    return EL05_OK;
}

/* 初始化：绑定总线、目标电机 CAN_ID、主机 CAN_ID 与量程模型 */
void EL05_Init(EL05_Motor *m, void *fdcan_bus, uint8_t motor_id,
               uint8_t master_id, const EL05_Model *model)
{
    if (m == 0) { return; }
    m->bus = fdcan_bus;
    m->id = motor_id;
    m->master_id = master_id;
    m->model = (model != 0) ? model : &EL05_MODEL;
    m->pos_rad = 0.0f;
    m->vel_rads = 0.0f;
    m->torque_nm = 0.0f;
    m->temp_c = 0.0f;
    m->fault = 0u;
    m->mode_state = 0u;
}

/* 电机使能运行（通信类型 3）：数据区清 0 */
uint8_t EL05_Enable(EL05_Motor *m)
{
    uint8_t d[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (m == 0) { return EL05_ERR_PARAM; }
    return EL05_Send(m->bus, EL05_MAKE_ID(EL05_TYPE_ENABLE, m->master_id, m->id), d, 8u);
}

/* 电机停止运行（通信类型 4）：数据区清 0 */
uint8_t EL05_Stop(EL05_Motor *m)
{
    uint8_t d[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (m == 0) { return EL05_ERR_PARAM; }
    return EL05_Send(m->bus, EL05_MAKE_ID(EL05_TYPE_STOP, m->master_id, m->id), d, 8u);
}

/* 电机停止运行并清故障（通信类型 4，Byte[0]=1） */
uint8_t EL05_ClearFault(EL05_Motor *m)
{
    uint8_t d[8] = {1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (m == 0) { return EL05_ERR_PARAM; }
    return EL05_Send(m->bus, EL05_MAKE_ID(EL05_TYPE_STOP, m->master_id, m->id), d, 8u);
}

/* 设置电机机械零位（通信类型 6，Byte[0]=1） */
uint8_t EL05_SetZero(EL05_Motor *m)
{
    uint8_t d[8] = {1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    if (m == 0) { return EL05_ERR_PARAM; }
    return EL05_Send(m->bus, EL05_MAKE_ID(EL05_TYPE_SET_ZERO, m->master_id, m->id), d, 8u);
}

/* 运控模式控制指令（通信类型 1）：
 * 数据区2 = 力矩；数据区 = 位置/速度/Kp/Kd（各 16bit，高字节在前） */
uint8_t EL05_MotionControl(EL05_Motor *m, float pos_rad, float vel_rad_s,
                           float kp, float kd, float tor_nm)
{
    const EL05_Model *mdl;
    uint8_t d[8];
    uint16_t t_u, p_u, v_u, kp_u, kd_u;

    if (m == 0) { return EL05_ERR_PARAM; }
    mdl = m->model;
    if ((mdl->p_max <= mdl->p_min) || (mdl->v_max <= mdl->v_min) ||
        (mdl->t_max <= mdl->t_min) || (mdl->kp_max <= 0.0f) || (mdl->kd_max <= 0.0f)) {
        return EL05_ERR_MODEL;
    }

    t_u  = EL05_Float_To_Uint(tor_nm,    mdl->t_min, mdl->t_max, 16u);
    p_u  = EL05_Float_To_Uint(pos_rad,   mdl->p_min, mdl->p_max, 16u);
    v_u  = EL05_Float_To_Uint(vel_rad_s, mdl->v_min, mdl->v_max, 16u);
    kp_u = EL05_Float_To_Uint(kp, 0.0f, mdl->kp_max, 16u);
    kd_u = EL05_Float_To_Uint(kd, 0.0f, mdl->kd_max, 16u);

    EL05_Write_U16(&d[0], p_u);    /* Byte0~1 目标角度   */
    EL05_Write_U16(&d[2], v_u);    /* Byte2~3 目标角速度 */
    EL05_Write_U16(&d[4], kp_u);   /* Byte4~5 Kp         */
    EL05_Write_U16(&d[6], kd_u);   /* Byte6~7 Kd         */

    /* 力矩放在 29 位 ID 的数据区2 */
    return EL05_Send(m->bus, EL05_MAKE_ID(EL05_TYPE_MOTION, t_u, m->id), d, 8u);
}

/* 接收解析（通信类型 2）：
 * ID 数据区2 低字节=当前电机CAN_ID、高字节=故障(bit0~5)+模式状态(bit6~7)
 * 数据：角度/角速度/力矩/温度×10，均 16bit 高字节在前 */
uint8_t EL05_OnCanRx(EL05_Motor *m, uint32_t ext_id,
                     const uint8_t *data, uint8_t len)
{
    const EL05_Model *mdl;
    uint32_t type;
    uint8_t  motor_id, hi;
    uint16_t p_u, v_u, t_u, temp_u;

    if ((m == 0) || (data == 0) || (len < 8u)) { return EL05_ERR_PARAM; }

    type = (ext_id >> 24) & 0x1Fu;
    if (type != EL05_TYPE_FEEDBACK) { return EL05_ERR_PARAM; }

    motor_id = (uint8_t)((ext_id >> 8) & 0xFFu);   /* 数据区2 低字节 */
    if (motor_id != m->id) { return EL05_ERR_PARAM; }

    hi = (uint8_t)((ext_id >> 16) & 0xFFu);        /* 数据区2 高字节 */
    m->fault      = (uint8_t)(hi & 0x3Fu);         /* bit0~5 故障位  */
    m->mode_state = (uint8_t)((hi >> 6) & 0x03u);  /* bit6~7 模式状态*/

    mdl = m->model;
    p_u    = EL05_Read_U16(&data[0]);
    v_u    = EL05_Read_U16(&data[2]);
    t_u    = EL05_Read_U16(&data[4]);
    temp_u = EL05_Read_U16(&data[6]);

    m->pos_rad   = EL05_Uint_To_Float(p_u, mdl->p_min, mdl->p_max, 16u);
    m->vel_rads  = EL05_Uint_To_Float(v_u, mdl->v_min, mdl->v_max, 16u);
    m->torque_nm = EL05_Uint_To_Float(t_u, mdl->t_min, mdl->t_max, 16u);
    m->temp_c    = (float)((int16_t)temp_u) / 10.0f;   /* 温度×10 */
    return EL05_OK;
}

/* 接收分发：按反馈帧里的"当前电机 CAN_ID"匹配句柄 */
uint8_t EL05_RxDispatch(EL05_Motor *motors, uint8_t count,
                        uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t motor_id;

    if (motors == 0) { return EL05_ERR_PARAM; }
    motor_id = (uint8_t)((ext_id >> 8) & 0xFFu);
    for (i = 0u; i < count; i++) {
        if (motors[i].id == motor_id) {
            return EL05_OnCanRx(&motors[i], ext_id, data, len);
        }
    }
    return EL05_ERR_PARAM;
}
