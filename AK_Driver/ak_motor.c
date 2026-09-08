/*****************************************************************************
 * ak_motor.c —— CubeMars AK 系列电机模组 运控(MIT)模式 CAN 驱动实现
 *
 * 依据工程内《AK 系列电机模组产品使用说明2.pdf》第 5.3 节：
 *   标准帧，ID=电机ID(默认1)，DLC=8，CAN 1Mbps；
 *   特殊码：进入控制模式 {FF×7,FC}、退出 {FF×7,FD}、设零 {FF×7,FE}；
 *   控制帧布局：p16[0:1] v12[2,3高4] kp12[3低4,4] kd12[5,6高4] t12[6低4,7]；
 *   反馈帧布局：id[0] p16[1:2] v12[3,4高4] t12[4低4,5] 温度[6] 错误[7]。
 *****************************************************************************/
#include "ak_motor.h"

#include "stm32h7xx_hal.h"

/* 内置模型（手册 5.3 参数范围表） */
const AK_MotorModel AK_MODEL_AK80_9  = { "AK80-9",  -12.5f, 12.5f, -50.0f, 50.0f, -18.0f, 18.0f, 500.0f, 5.0f };
const AK_MotorModel AK_MODEL_AK45_10 = { "AK45-10", -12.5f, 12.5f, -20.0f, 20.0f, -8.0f,  8.0f,  500.0f, 5.0f };

/* 区间钳位 */
static float AK_FClamp(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

/* float 定点化（手册 float_to_uint），并钳位到 bits 位可表示上限 */
static uint32_t AK_Float_To_Uint(float x, float x_min, float x_max,
                                 uint32_t bits, uint8_t *ok)
{
    uint32_t u, max_u;
    if (ok != 0) { *ok = AK_OK; }
    if (x_max - x_min <= 0.0f) { if (ok != 0) { *ok = AK_ERR_MODEL; } return 0u; }
    x = AK_FClamp(x, x_min, x_max);
    u = (uint32_t)((x - x_min) * ((float)(1u << bits) / (x_max - x_min)));
    max_u = (1u << bits) - 1u;
    if (u > max_u) { u = max_u; }
    return u;
}

/* 定点 -> float（手册 uint_to_float） */
static float AK_Uint_To_Float(uint32_t x_int, float x_min, float x_max, uint32_t bits)
{
    return ((float)x_int) * (x_max - x_min) / ((float)((1u << bits) - 1u)) + x_min;
}

/* 底层发送：标准帧、经典 CAN（非 FD）、DLC<=8 */
static uint8_t AK_Bus_Send(void *bus, uint32_t std_id,
                           const uint8_t *data, uint8_t len)
{
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)bus;
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t buf[8], i;

    if (hfdcan == 0) { return AK_ERR_PARAM; }
    if (len > 8u) { len = 8u; }
    for (i = 0u; i < 8u; i++) { buf[i] = (i < len) ? data[i] : 0u; }

    tx.Identifier          = std_id;
    tx.IdType              = FDCAN_STANDARD_ID;   /* 标准帧 */
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = (uint32_t)len;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;   /* 经典帧 */

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, buf) != HAL_OK) {
        return AK_ERR_BUS;
    }
    return AK_OK;
}

/* 生成自定义电机模型 */
AK_MotorModel AK_Motor_MakeModel(const char *name,
                                 float p_min, float p_max,
                                 float v_min, float v_max,
                                 float t_min, float t_max,
                                 float kp_max, float kd_max)
{
    AK_MotorModel mdl;
    mdl.name = name;
    mdl.p_min = p_min; mdl.p_max = p_max;
    mdl.v_min = v_min; mdl.v_max = v_max;
    mdl.t_min = t_min; mdl.t_max = t_max;
    mdl.kp_max = kp_max; mdl.kd_max = kd_max;
    return mdl;
}

/* 初始化句柄 */
void AK_Motor_Init(AK_Motor *m, void *fdcan_bus, uint8_t motor_id,
                   const AK_MotorModel *model)
{
    if (m == 0) { return; }
    m->bus = fdcan_bus;
    m->id = motor_id;
    m->model = (model != 0) ? model : &AK_MODEL_AK80_9;
    m->pos_rad = 0.0f; m->vel_rads = 0.0f; m->torque_nm = 0.0f;
    m->temp_c = 0; m->err_code = 0u;
}

/* 进入电机控制模式（特殊码 FF×7+FC） */
uint8_t AK_Motor_Enable(AK_Motor *m)
{
    static const uint8_t en[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFCu};
    if (m == 0) { return AK_ERR_PARAM; }
    return AK_Bus_Send(m->bus, m->id, en, 8u);
}

/* 退出电机控制模式（特殊码 FF×7+FD） */
uint8_t AK_Motor_Exit(AK_Motor *m)
{
    static const uint8_t ex[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFDu};
    if (m == 0) { return AK_ERR_PARAM; }
    return AK_Bus_Send(m->bus, m->id, ex, 8u);
}

/* 设置电机当前位置为 0 点（特殊码 FF×7+FE） */
uint8_t AK_Motor_SetZero(AK_Motor *m)
{
    static const uint8_t z[8] = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFEu};
    if (m == 0) { return AK_ERR_PARAM; }
    return AK_Bus_Send(m->bus, m->id, z, 8u);
}

/* 运控MIT：打包并发送位置/速度/扭矩指令 */
uint8_t AK_Motor_MIT(AK_Motor *m, float pos_rad, float vel_rad_s,
                     float kp, float kd, float tor_nm)
{
    const AK_MotorModel *mdl;
    uint8_t b[8], ok = AK_OK;
    uint32_t p_i, v_i, kp_i, kd_i, t_i;

    if (m == 0) { return AK_ERR_PARAM; }
    mdl = m->model;
    p_i  = AK_Float_To_Uint(pos_rad,  mdl->p_min, mdl->p_max, 16u, &ok); if (ok != AK_OK) { return ok; }
    v_i  = AK_Float_To_Uint(vel_rad_s, mdl->v_min, mdl->v_max, 12u, &ok); if (ok != AK_OK) { return ok; }
    kp_i = AK_Float_To_Uint(kp, 0.0f, mdl->kp_max, 12u, &ok);             if (ok != AK_OK) { return ok; }
    kd_i = AK_Float_To_Uint(kd, 0.0f, mdl->kd_max, 12u, &ok);             if (ok != AK_OK) { return ok; }
    t_i  = AK_Float_To_Uint(tor_nm, mdl->t_min, mdl->t_max, 12u, &ok);    if (ok != AK_OK) { return ok; }

    b[0] = (uint8_t)(p_i >> 8);                       /* 位置高8 */
    b[1] = (uint8_t)(p_i & 0xFFu);                    /* 位置低8 */
    b[2] = (uint8_t)(v_i >> 4);                       /* 速度高8 */
    b[3] = (uint8_t)(((v_i & 0x0Fu) << 4) | (kp_i >> 8)); /* 速度低4 | Kp高4 */
    b[4] = (uint8_t)(kp_i & 0xFFu);                   /* Kp低8 */
    b[5] = (uint8_t)(kd_i >> 4);                      /* Kd高8 */
    b[6] = (uint8_t)(((kd_i & 0x0Fu) << 4) | (t_i >> 8)); /* Kd低4 | 扭矩高4 */
    b[7] = (uint8_t)(t_i & 0xFFu);                    /* 扭矩低8 */
    return AK_Bus_Send(m->bus, m->id, b, 8u);
}

/* 解析运控反馈：id[0] 位置16[1:2] 速度12 扭矩12 温度 错误 */
uint8_t AK_Motor_OnCanRx(AK_Motor *m, uint32_t std_id,
                         const uint8_t *data, uint8_t len)
{
    const AK_MotorModel *mdl;
    uint32_t p_raw, v_raw, t_raw;

    if ((m == 0) || (data == 0) || (len < 8u)) { return AK_ERR_PARAM; }
    if ((uint32_t)m->id != std_id) { return AK_ERR_PARAM; }
    mdl = m->model;

    p_raw = ((uint32_t)data[1] << 8) | data[2];              /* 位置16位 */
    v_raw = ((uint32_t)data[3] << 4) | (data[4] >> 4);       /* 速度12位 */
    t_raw = (((uint32_t)(data[4] & 0x0Fu)) << 8) | data[5];  /* 扭矩12位 */

    m->pos_rad   = AK_Uint_To_Float(p_raw, mdl->p_min, mdl->p_max, 16u);
    m->vel_rads  = AK_Uint_To_Float(v_raw, mdl->v_min, mdl->v_max, 12u);
    /* 手册接收例程：torque = uint_to_float(i_int, -T_MAX, T_MAX, 12) */
    m->torque_nm = AK_Uint_To_Float(t_raw, -mdl->t_max, mdl->t_max, 12u);
    /* 手册：Temperature = T_int - 40（范围 -40~215） */
    m->temp_c    = (int16_t)data[6] - 40;
    m->err_code  = data[7];
    return AK_OK;
}

/* 按标准帧 ID（=电机ID）分发到电机句柄 */
uint8_t AK_Motor_RxDispatch(AK_Motor *motors, uint8_t count,
                            uint32_t std_id,
                            const uint8_t *data, uint8_t len)
{
    uint8_t i;
    if (motors == 0) { return AK_ERR_PARAM; }
    for (i = 0u; i < count; i++) {
        if ((uint32_t)motors[i].id == std_id) {
            return AK_Motor_OnCanRx(&motors[i], std_id, data, len);
        }
    }
    return AK_ERR_PARAM;
}
