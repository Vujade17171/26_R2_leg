/*****************************************************************************
 * ak_motor.c
 * CubeMars AK 系列执行器 CAN 驱动实现（依据《AK系列产品说明书 v3.2.0》）
 *
 * 协议速查（详见说明书第 4 章）：
 *  - 帧格式：CAN 扩展帧，DLC<=8
 *  - 控制帧 ID = (控制模式ID << 8) | 驱动器ID
 *    模式: 0 占空比 | 1 电流 | 2 电流刹车 | 3 速度 | 4 位置 | 5 原点
 *          | 6 位置速度环 | 8 力控MIT | 15 失能 | 16 反馈报文设置
 *  - 反馈帧 ID = (功能ID << 8) | 驱动器ID
 *    功能: 0x29 实时状态 | 0x2A 4字节位置 | 0x2C 伺服起始应答
 *  - 力控MIT 打包: p:16bit, v:12bit, kp:12bit, kd:12bit, t:12bit
 *    （float_to_uint 定点化，与 MIT mini-cheetah 风格一致）
 *
 * 发送底层仅依赖 HAL FDCAN（stm32h7xx_hal.h）；若换平台，
 * 只需改写 AK_Bus_Send() 一个函数。
 *****************************************************************************/
#include "ak_motor.h"

/* 换用非 FDCAN 总线或其它系列时，仅需调整本头文件 */
#include "stm32h7xx_hal.h"

/* ------------------------------------------------------------------ */
/* 内置电机模型                                                       */
/* ------------------------------------------------------------------ */
/* AK80-9（手册 v3.2.0 第 4.2 节）：
 *   Kt=0.5701 N·m/A，速度限幅 ±65 rad/s，扭矩限幅 ±18 N·m，
 *   Kp 0~500，Kd 0~5；KV=100。
 * 减速比与极对数手册未直接给出：AKxx-9 按 9:1、21 对极填写，
 * 请对照实物/参数文件确认后修改。                                   */
const AK_MotorModel AK_MODEL_AK80_9 = {
    "AK80-9",      /* name         */
    100.0f,        /* kv           */
    0.5701f,       /* kt  N·m/A    */
    9.0f,          /* gear         */
    21.0f,         /* pole_pairs   */
    -1.256636f, 3.14159f, /* p_min/p_max rad：按实际机械限位调整 */
    -65.0f, 65.0f,   /* v_min/v_max rad/s */
    -18.0f, 18.0f,   /* t_min/t_max N·m  */
    500.0f, 5.0f     /* kp_max/kd_max    */ 
};

/* AK45-10 V3.0 KV75（手册 v3.2.0 未收录！）：
 * 关键参数(kt/速度/扭矩限幅/极对数)必须从厂商参数文件获取后，
 * 用 AK_Motor_MakeModel() 生成模型再使用，切勿直接使用本占位模型。 */
const AK_MotorModel AK_MODEL_AK45_10 = {
    "AK45-10 KV75",  /* name      */
    75.0f,           /* kv        */
    0.0f,            /* kt        ：待填 */
    10.0f,           /* gear      ：官网 10:1 */
    0.0f,            /* pole_pairs：待填 */
    -2.827431f, 2.827431f,      /* p_min/max ：待填 */
    0.0f, 0.0f,      /* v_min/max ：待填 */
    0.0f, 0.0f,      /* t_min/max ：待填 */
    500.0f, 5.0f     /* kp/kd max */
};

/* ------------------------------------------------------------------ */
/* 内部工具                                                           */
/* ------------------------------------------------------------------ */

#define AK_PI          3.14159265358979f
#define AK_DEG2RAD     0.0174532925199433f   /* deg->rad */
#define AK_RAD2DEG     57.2957795130823f     /* rad->deg */
#define AK_RADS2RPM    (60.0f / (2.0f * AK_PI))   /* rad/s -> rpm */

static float AK_FClamp(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

/* float -> 定点（手册 float_to_uint，1:1 复刻） */
static uint32_t AK_Float_To_Uint(float x, float x_min, float x_max,
                                 uint32_t bits, uint8_t *ok)
{
    float span;
    if (ok != 0) { *ok = AK_OK; }
    span = x_max - x_min;
    if (span <= 0.0f) {
        if (ok != 0) { *ok = AK_ERR_MODEL; }
        return 0u;
    }
    x = AK_FClamp(x, x_min, x_max);
    return (uint32_t)((x - x_min) * ((float)(1u << bits) / span));
}

/* 大端写入 int16/int32（与手册 buffer_append_int* 一致） */
static void AK_Append_I16(uint8_t *buf, int16_t v)
{
    buf[0] = (uint8_t)((uint16_t)v >> 8);
    buf[1] = (uint8_t)((uint16_t)v & 0xFFu);
}
static void AK_Append_I32(uint8_t *buf, int32_t v)
{
    buf[0] = (uint8_t)((uint32_t)v >> 24);
    buf[1] = (uint8_t)((uint32_t)v >> 16);
    buf[2] = (uint8_t)((uint32_t)v >> 8);
    buf[3] = (uint8_t)((uint32_t)v & 0xFFu);
}
/* 大端读取 */
static int16_t AK_Read_I16(const uint8_t *d)
{
    return (int16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);
}
static int32_t AK_Read_I32(const uint8_t *d)
{
    return (((int32_t)d[0]) << 24) | (((int32_t)d[1]) << 16)
         | (((int32_t)d[2]) << 8)  |  ((int32_t)d[3]);
}

/* ------------------------------------------------------------------ */
/* 底层发送（唯一需要按平台适配的函数）                               */
/* 发送 CAN 扩展帧，len<=8；经典 CAN（非 FD）                          */
/* ------------------------------------------------------------------ */
static uint8_t AK_Bus_Send(void *bus, uint32_t ext_id,
                           const uint8_t *data, uint8_t len)
{
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)bus;
    FDCAN_TxHeaderTypeDef tx = {0};   /* 清零，避免 TxEventFifoControl/MessageMarker 为栈垃圾 */
    uint8_t buf[8];
    uint8_t i;
    HAL_StatusTypeDef st;

    if (hfdcan == 0) {
        return AK_ERR_PARAM;
    }
    if (len > 8u) {
        len = 8u;
    }
    for (i = 0u; i < 8u; i++) {
        buf[i] = (i < len) ? data[i] : 0u;
    }

    tx.Identifier            = ext_id;
    tx.IdType                = FDCAN_EXTENDED_ID;
    tx.TxFrameType           = FDCAN_DATA_FRAME;
    tx.DataLength            = (uint32_t)len;   /* <=8, 经典帧 DLC */
    tx.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch         = FDCAN_BRS_OFF;
    tx.FDFormat              = FDCAN_FD_CAN;    /* 经典 CAN 帧 */

    /* 注意：新版 CubeH7 FDCAN HAL 无 AddMessageToTxMailBox，
     * 发送方式需与 hfdcan1.Init.TxFifoQueueMode 匹配：
     *   FDCAN_TX_FIFO_OPERATION    -> HAL_FDCAN_AddMessageToTxFifoQ
     *   FDCAN_TX_BUFFER_OPERATION  -> HAL_FDCAN_AddMessageToTxBuffer */
    st = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, buf);
    if (st != HAL_OK) {
        return AK_ERR_BUS;
    }
    return AK_OK;
}

/* ------------------------------------------------------------------ */
/* 模型生成                                                           */
/* ------------------------------------------------------------------ */
AK_MotorModel AK_Motor_MakeModel(const char *name,
                                 float kv, float kt,
                                 float gear, float pole_pairs,
                                 float p_min, float p_max,
                                 float v_min, float v_max,
                                 float t_min, float t_max,
                                 float kp_max, float kd_max)
{
    AK_MotorModel mdl;

    mdl.name       = name;
    mdl.kv         = kv;
    mdl.kt         = kt;
    mdl.gear       = gear;
    mdl.pole_pairs = pole_pairs;
    mdl.p_min      = p_min;
    mdl.p_max      = p_max;
    mdl.v_min      = v_min;
    mdl.v_max      = v_max;
    mdl.t_min      = t_min;
    mdl.t_max      = t_max;
    mdl.kp_max     = kp_max;
    mdl.kd_max     = kd_max;
    return mdl;
}

void AK_Motor_Init(AK_Motor *m, void *fdcan_bus,
                   uint8_t drive_id, const AK_MotorModel *model)
{
    if (m == 0) {
        return;
    }
    m->bus   = fdcan_bus;
    m->id    = drive_id;
    m->model = (model != 0) ? model : &AK_MODEL_AK80_9;
    m->pos_rad  = 0.0f;
    m->vel_rads = 0.0f;
    m->iq_a     = 0.0f;
    m->temp_c   = 0;
    m->err_code = 0u;
    m->flags    = 0u;
    m->raw_pos32= 0;
}

uint8_t AK_Motor_SetModel(AK_Motor *m, const AK_MotorModel *model)
{
    if ((m == 0) || (model == 0)) {
        return AK_ERR_PARAM;
    }
    m->model = model;
    return AK_OK;
}

/* ------------------------------------------------------------------ */
/* 伺服模式（手册 4.1，发送均为 int32 大端）                          */
/* ------------------------------------------------------------------ */

uint8_t AK_Motor_SetDuty(AK_Motor *m, float duty)         /* ID=0 */
{
    uint8_t b[4];

    if (m == 0) { return AK_ERR_PARAM; }
    AK_Append_I32(b, (int32_t)(AK_FClamp(duty, -0.95f, 0.95f) * 100000.0f));
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_DUTY << 8) | m->id, b, 4u);
}

uint8_t AK_Motor_SetCurrent(AK_Motor *m, float amp)       /* ID=1，A */
{
    uint8_t b[4];

    if (m == 0) { return AK_ERR_PARAM; }
    AK_Append_I32(b, (int32_t)(AK_FClamp(amp, -60.0f, 60.0f) * 1000.0f));
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_CURRENT << 8) | m->id, b, 4u);
}

uint8_t AK_Motor_SetBrake(AK_Motor *m, float amp)         /* ID=2，A */
{
    uint8_t b[4];

    if (m == 0) { return AK_ERR_PARAM; }
    AK_Append_I32(b, (int32_t)(AK_FClamp(amp, -60.0f, 60.0f) * 1000.0f));
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_CURRENT_BRAKE << 8) | m->id, b, 4u);
}

uint8_t AK_Motor_SetSpeedERPM(AK_Motor *m, int32_t erpm)  /* ID=3 */
{
    uint8_t b[4];

    if (m == 0) { return AK_ERR_PARAM; }
    if (erpm >  100000) { erpm =  100000; }
    if (erpm < -100000) { erpm = -100000; }
    AK_Append_I32(b, erpm);
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_SPEED << 8) | m->id, b, 4u);
}

uint8_t AK_Motor_SetSpeed(AK_Motor *m, float rad_s)       /* ID=3，SI */
{
    float erpm;
    const AK_MotorModel *mdl;

    if (m == 0) { return AK_ERR_PARAM; }
    mdl = m->model;
    if ((mdl->gear <= 0.0f) || (mdl->pole_pairs <= 0.0f)) {
        return AK_ERR_MODEL;   /* 减速比/极对数未配置 */
    }
    erpm = rad_s * AK_RADS2RPM * mdl->gear * mdl->pole_pairs;
    return AK_Motor_SetSpeedERPM(m, (int32_t)(erpm + 0.5f));
}

uint8_t AK_Motor_SetPosition(AK_Motor *m, float rad)      /* ID=4，SI */
{
    uint8_t b[4];
    int32_t v;

    if (m == 0) { return AK_ERR_PARAM; }
    /* 位置环内部为 °x10000，范围 ±360000000 -> ±36000° */
    v = (int32_t)(rad * AK_RAD2DEG * 10000.0f);
    if (v >  360000000) { v =  360000000; }
    if (v < -360000000) { v = -360000000; }
    AK_Append_I32(b, v);
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_POS << 8) | m->id, b, 4u);
}

uint8_t AK_Motor_SetOrigin(AK_Motor *m, uint8_t permanent) /* ID=5 */
{
    uint8_t b = (permanent != 0u) ? 1u : 0u;

    if (m == 0) { return AK_ERR_PARAM; }
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_ORIGIN << 8) | m->id, &b, 1u);
}

uint8_t AK_Motor_SetPositionVelocity(AK_Motor *m,
                                     float pos_rad,
                                     float vel_rad_s,
                                     float acc_rad_s2)     /* ID=6，SI */
{
    uint8_t b[8];
    int32_t pv;
    int16_t sv, av;
    float erpm, aerpm;
    const AK_MotorModel *mdl;

    if (m == 0) { return AK_ERR_PARAM; }
    mdl = m->model;
    if ((mdl->gear <= 0.0f) || (mdl->pole_pairs <= 0.0f)) {
        return AK_ERR_MODEL;
    }
    /* 位置 int32: °x10000 */
    pv = (int32_t)(pos_rad * AK_RAD2DEG * 10000.0f);
    if (pv >  360000000) { pv =  360000000; }
    if (pv < -360000000) { pv = -360000000; }
    /* 速度 int16: ERPM/10（-32767..32767） */
    erpm = vel_rad_s * AK_RADS2RPM * mdl->gear * mdl->pole_pairs;
    sv = (int16_t)(AK_FClamp(erpm / 10.0f, -32767.0f, 32767.0f));
    /* 加速度 int16: (ERPM/s²)/10（0..32767，1单位=10ERPM/s²） */
    aerpm = acc_rad_s2 * AK_RADS2RPM * mdl->gear * mdl->pole_pairs;
    av = (int16_t)AK_FClamp(aerpm / 10.0f, 0.0f, 32767.0f);

    AK_Append_I32(b, pv);
    AK_Append_I16(&b[4], sv);
    AK_Append_I16(&b[6], av);
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_POS_SPD << 8) | m->id, b, 8u);
}

/* ------------------------------------------------------------------ */
/* 失能 / 反馈配置                                                    */
/* ------------------------------------------------------------------ */

uint8_t AK_Motor_Disable(AK_Motor *m)                      /* ID=15 */
{
    if (m == 0) { return AK_ERR_PARAM; }
    m->flags &= (uint8_t)~AK_FLAG_DISABLE_ACKED;
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_DISABLE << 8) | m->id, 0, 0u);
}

uint8_t AK_Motor_ConfigFeedback(AK_Motor *m, uint16_t cfg) /* ID=16 */
{
    uint8_t b[8];

    if (m == 0) { return AK_ERR_PARAM; }
    b[0] = 0u; b[1] = 0u; b[2] = 0u; b[3] = 0u;
    b[4] = 0u; b[5] = 0u;
    b[6] = (uint8_t)(cfg >> 8);
    b[7] = (uint8_t)(cfg & 0xFFu);
    /* 该指令写入 Flash，勿高频发送 */
    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_FRAME_CFG << 8) | m->id, b, 8u);
}

/* ------------------------------------------------------------------ */
/* 力控 / MIT 模式（手册 4.2，ID=8）                                  */
/* ------------------------------------------------------------------ */
uint8_t AK_Motor_MIT(AK_Motor *m, float pos_rad, float vel_rad_s,
                     float kp, float kd, float tor_nm)
{
    const AK_MotorModel *mdl;
    uint8_t b[8];
    uint32_t p_i, v_i, kp_i, kd_i, t_i;
    uint8_t ok = AK_OK;

    if (m == 0) { return AK_ERR_PARAM; }
    mdl = m->model;

    p_i = AK_Float_To_Uint(pos_rad, mdl->p_min, mdl->p_max, 16u, &ok);
    if (ok != AK_OK) { return ok; }
    v_i = AK_Float_To_Uint(vel_rad_s, mdl->v_min, mdl->v_max, 12u, &ok);
    if (ok != AK_OK) { return ok; }
    t_i = AK_Float_To_Uint(tor_nm, mdl->t_min, mdl->t_max, 12u, &ok);
    if (ok != AK_OK) { return ok; }
    kp_i = AK_Float_To_Uint(AK_FClamp(kp, 0.0f, mdl->kp_max),
                            0.0f, mdl->kp_max, 12u, &ok);
    if (ok != AK_OK) { return ok; }
    kd_i = AK_Float_To_Uint(AK_FClamp(kd, 0.0f, mdl->kd_max),
                            0.0f, mdl->kd_max, 12u, &ok);
    if (ok != AK_OK) { return ok; }

    /* 打包：与手册/上位机一致 */
    b[0] = (uint8_t)(kp_i >> 4);
    b[1] = (uint8_t)(((kp_i & 0x0Fu) << 4) | (kd_i >> 8));
    b[2] = (uint8_t)(kd_i & 0xFFu);
    b[3] = (uint8_t)(p_i >> 8);
    b[4] = (uint8_t)(p_i & 0xFFu);
    b[5] = (uint8_t)(v_i >> 4);
    b[6] = (uint8_t)(((v_i & 0x0Fu) << 4) | (t_i >> 8));
    b[7] = (uint8_t)(t_i & 0xFFu);

    return AK_Bus_Send(m->bus,
                       ((uint32_t)AK_MODE_MIT << 8) | m->id, b, 8u);
}

/* ------------------------------------------------------------------ */
/* 接收解析（手册 4.3.1）                                             */
/* ------------------------------------------------------------------ */
uint8_t AK_Motor_OnCanRx(AK_Motor *m, uint32_t ext_id,
                         const uint8_t *data, uint8_t dlc)
{
    uint32_t func;
    float deg;

    if ((m == 0) || (data == 0) || (dlc < 8u)) {
        return AK_ERR_PARAM;
    }
    func = ext_id >> 8;         /* 功能 ID 位于 [28:8] */
    if (((uint8_t)(ext_id & 0xFFu)) != m->id) {
        return AK_ERR_PARAM;    /* 驱动器 ID 不匹配 */
    }

    switch (func) {
    case AK_FB_STATE:           /* 0x29 实时状态 */
        deg = (float)AK_Read_I16(&data[0]) * 0.1f;      /* ° */
        m->pos_rad  = deg * AK_DEG2RAD;
        m->vel_rads = AK_ERPM_To_RadS((float)((int32_t)AK_Read_I16(&data[2]) * 10),
                                      (m->model != 0) ? m->model->pole_pairs : 0.0f,
                                      (m->model != 0) ? m->model->gear : 0.0f);
        m->iq_a     = (float)AK_Read_I16(&data[4]) * 0.01f;
        m->temp_c   = (int8_t)data[6];
        m->err_code = data[7];
        if (data[7] == AK_DISABLE_ACK) {
            m->flags |= AK_FLAG_DISABLE_ACKED;   /* 失能成功应答 */
        }
        return AK_OK;
    case AK_FB_POS32:           /* 0x2A 4字节位置 */
        m->raw_pos32 = AK_Read_I32(&data[0]);
        m->pos_rad   = ((float)m->raw_pos32 * AK_POS32_LSB_DEG) * AK_DEG2RAD;
        return AK_OK;
    case AK_FB_ENTER:           /* 0x2C 伺服起始应答 */
    default:
        return AK_OK;           /* 无需解析 */
    }
}

uint8_t AK_Motor_RxDispatch(AK_Motor *motors, uint8_t count,
                            uint32_t ext_id,
                            const uint8_t *data, uint8_t dlc)
{
    uint8_t i;
    uint8_t id;

    if (motors == 0) {
        return AK_ERR_PARAM;
    }
    id = (uint8_t)(ext_id & 0xFFu);
    for (i = 0u; i < count; i++) {
        if (motors[i].id == id) {
            return AK_Motor_OnCanRx(&motors[i], ext_id, data, dlc);
        }
    }
    return AK_ERR_PARAM;        /* 未找到对应电机 */
}

/* ------------------------------------------------------------------ */
/* 单位换算                                                           */
/* ------------------------------------------------------------------ */
float AK_ERPM_To_RadS(float erpm, float pole_pairs, float gear)
{
    if ((pole_pairs <= 0.0f) || (gear <= 0.0f)) {
        return 0.0f;
    }
    return erpm / AK_RADS2RPM / pole_pairs / gear;  /* 输出端 rad/s */
}

float AK_RadS_To_ERPM(float rad_s, float pole_pairs, float gear)
{
    if ((pole_pairs <= 0.0f) || (gear <= 0.0f)) {
        return 0.0f;
    }
    return rad_s * AK_RADS2RPM * pole_pairs * gear; /* 电气转速 ERPM */
}
