#include "Joint_EL05.h"
#include "bsp_fdcan.h"
#include "string.h"

EL05_Handle_t g_el05 = {0};

/* ==================== 工具函数 ==================== */

/* 浮点限幅 */
static float el05_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 对称量程编码：x ∈ [-x_max, x_max] → u16 ∈ [0, 0xFFFE]（中点 0x7FFF） */
static uint16_t el05_encode_sym(float x, float x_max)
{
    float v = (x / x_max + 1.0f) * 32767.0f;
    if (v < 0.0f)     v = 0.0f;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)(v + 0.5f);
}

/* 非负量程编码：x ∈ [0, x_max] → u16 ∈ [0, 0xFFFF] */
static uint16_t el05_encode_pos(float x, float x_max)
{
    if (x_max <= 0.0f) return 0;
    float v = (x / x_max) * 65535.0f;
    if (v < 0.0f)     v = 0.0f;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)(v + 0.5f);
}

/* 对称量程解码：u16 → x ∈ [-x_max, x_max] */
static float el05_decode_sym(uint16_t u, float x_max)
{
    return ((float)u / 32767.0f - 1.0f) * x_max;
}

/* ==================== 反馈解析（通信类型 2） ==================== */
void EL05_Parse_Feedback(EL05_Handle_t *el05, uint32_t ext_id, uint8_t *data)
{
    uint8_t comm = (ext_id >> 24) & 0x1F;
    if (comm != EL05_COMM_STATUS) {
        return;
    }

    uint16_t extra = (ext_id >> 8) & 0xFFFF;

    uint16_t p_u    = (data[0] << 8) | data[1];
    uint16_t v_u    = (data[2] << 8) | data[3];
    uint16_t t_u    = (data[4] << 8) | data[5];
    uint16_t temp_u = (data[6] << 8) | data[7];

    el05->status.position    = el05_decode_sym(p_u, el05->params.pos_max);
    el05->status.speed       = el05_decode_sym(v_u, el05->params.vel_max);
    el05->status.torque      = el05_decode_sym(t_u, el05->params.tor_max);
    el05->status.temperature = (float)temp_u * 0.1f;      /* 手册：Temp(℃)*10 */
    el05->status.mode_state  = (extra >> 14) & 0x03;      /* bit22~23 模式状态 */
    el05->status.fault       = (extra >> 8) & 0x3F;       /* bit16~21 故障位 */
    el05->status.id          = extra & 0xFF;              /* bit8~15 当前电机 ID */
}

/* ==================== 函数1：初始化 ==================== */
HAL_StatusTypeDef EL05_Motion_Init(EL05_Handle_t *el05, FDCAN_HandleTypeDef *pcan, uint8_t motor_id)
{
    if (el05->is_initialized) {
        return HAL_OK;
    }

    if (pcan == NULL) {
        return HAL_ERROR;
    }

    if (motor_id == 0) {
        motor_id = EL05_ID;
    }

    el05->pcan_handle   = pcan;
    el05->motor_id      = motor_id;
    el05->host_id       = EL05_HOST_ID;
    el05->is_initialized = 1;
    el05->is_entered    = 0;

    /* 初始化量程 */
    el05->params.pos_min = EL05_P_MIN;
    el05->params.pos_max = EL05_P_MAX;
    el05->params.vel_min = EL05_V_MIN;
    el05->params.vel_max = EL05_V_MAX;
    el05->params.tor_min = EL05_T_MIN;
    el05->params.tor_max = EL05_T_MAX;
    el05->params.kp_min  = EL05_KP_MIN;
    el05->params.kp_max  = EL05_KP_MAX;
    el05->params.kd_min  = EL05_KD_MIN;
    el05->params.kd_max  = EL05_KD_MAX;

    /* 清空状态 */
    memset(&el05->status, 0, sizeof(EL05_MotorStatus_t));
    el05->status.id = motor_id;

    return HAL_OK;
}

/* ==================== 函数2：使能 ==================== */
HAL_StatusTypeDef EL05_Motion_Enter(EL05_Handle_t *el05)
{
    if (!el05->is_initialized || el05->pcan_handle == NULL) {
        return HAL_ERROR;
    }

    if (el05->is_entered) {
        return HAL_OK;
    }

    uint8_t data[8] = {0};
    uint32_t ext_id = (EL05_COMM_ENABLE << 24) | ((uint32_t)el05->host_id << 8) | el05->motor_id;

    HAL_StatusTypeDef ret = can_send_ext_data(el05->pcan_handle, ext_id, data, 8);
    el05->is_entered = 1;

    return ret;
}

/* ==================== 函数3：停止 ==================== */
HAL_StatusTypeDef EL05_Motion_Exit(EL05_Handle_t *el05)
{
    if (!el05->is_initialized || el05->pcan_handle == NULL) {
        return HAL_ERROR;
    }

    if (!el05->is_entered) {
        return HAL_OK;
    }

    uint8_t data[8] = {0};
    uint32_t ext_id = (EL05_COMM_DISABLE << 24) | ((uint32_t)el05->host_id << 8) | el05->motor_id;

    HAL_StatusTypeDef ret = can_send_ext_data(el05->pcan_handle, ext_id, data, 8);
    el05->is_entered = 0;

    return ret;
}

/* ==================== 函数4：运控控制 ==================== */
HAL_StatusTypeDef EL05_Motion_Control(EL05_Handle_t *el05, float position, float speed,
                                      float kp, float kd, float torque)
{
    if (!el05->is_initialized || el05->pcan_handle == NULL) {
        return HAL_ERROR;
    }

    if (!el05->is_entered) {
        return HAL_ERROR;
    }

    /* 限幅 */
    position = el05_clamp(position, el05->params.pos_min, el05->params.pos_max);
    speed    = el05_clamp(speed,    el05->params.vel_min, el05->params.vel_max);
    kp       = el05_clamp(kp,       el05->params.kp_min,  el05->params.kp_max);
    kd       = el05_clamp(kd,       el05->params.kd_min,  el05->params.kd_max);
    torque   = el05_clamp(torque,   el05->params.tor_min, el05->params.tor_max);

    /* 编码 */
    uint16_t p_u  = el05_encode_sym(position, el05->params.pos_max);
    uint16_t v_u  = el05_encode_sym(speed,    el05->params.vel_max);
    uint16_t kp_u = el05_encode_pos(kp,       el05->params.kp_max);
    uint16_t kd_u = el05_encode_pos(kd,       el05->params.kd_max);
    uint16_t t_u  = el05_encode_sym(torque,   el05->params.tor_max);

    /* 数据区（大端）：位置 / 速度 / Kp / Kd */
    uint8_t data[8];
    data[0] = (p_u  >> 8) & 0xFF;
    data[1] =  p_u        & 0xFF;
    data[2] = (v_u  >> 8) & 0xFF;
    data[3] =  v_u        & 0xFF;
    data[4] = (kp_u >> 8) & 0xFF;
    data[5] =  kp_u       & 0xFF;
    data[6] = (kd_u >> 8) & 0xFF;
    data[7] =  kd_u       & 0xFF;

    /* 扩展 ID：通信类型1 | 力矩(bit23~8) | 电机ID(bit7~0) */
    uint32_t ext_id = (EL05_COMM_OPERATION_CONTROL << 24) | ((uint32_t)t_u << 8) | el05->motor_id;

    return can_send_ext_data(el05->pcan_handle, ext_id, data, 8);
}
