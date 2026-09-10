#include "Joint_AK80.h"
//功能函数：
static int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    
    if (offset < 0.0f) offset = 0.0f;
    if (offset > span) offset = span;
    
    float max_int = (float)((1 << bits) - 1);
    float result = offset * max_int / span;
    
    return (int)(result + 0.5f);  /* 四舍五入 */
}
static float uint_to_float(int x_int, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    float max_int = (float)((1 << bits) - 1);
    
    if (x_int < 0) x_int = 0;
    if (x_int > (int)max_int) x_int = (int)max_int;
    
    return (float)x_int * span / max_int + x_min;
}
//发送最后命令码函数：
static void send_special_command(AK_Handle_t*ak,uint8_t last_byte)
{
    uint8_t cmd[8];
    for (int i = 0; i < 7; i++) {
        cmd[i] = 0xFF;
    }
    cmd[7] = last_byte;
    can_send_data(ak,ak->motor_id, cmd, 8);
}
//解析反馈数据：
void parse_motion_feedback(AK_Handle_t*ak,uint8_t *data)
{
    AK_MotorParams_t p = ak->params;
    uint8_t id = data[0];
    int p_int = (data[1] << 8) | data[2];
    int v_int = (data[3] << 4) | (data[4] >> 4);
    int i_int = ((data[4] & 0x0F) << 8) | data[5];
    ak->status.position = uint_to_float(p_int, p.pos_min, p.pos_max, 16);
    ak->status.speed = uint_to_float(v_int, p.vel_min, p.vel_max, 12);
    ak->status.current = uint_to_float(i_int, -60.0f, 60.0f, 12);
    ak->status.temperature = (int8_t)data[6];
    ak->status.error_code = data[7];
    ak->status.id = id;
}
//驱动函数：
//函数1：初始化函数：
HAL_StatusTypeDef AK_Motion_Init(AK_Handle_t*ak,FDCAN_HandleTypeDef *pcan, uint8_t motor_id)
{
    if (ak->is_initialized) {
        return HAL_OK;
    }
    
    if (pcan == NULL) {
        return HAL_ERROR;
    }
    
    ak->pcan_handle = pcan;
    ak->motor_id = motor_id;
    ak->motor_type = 1;   /* 1 表示 AK80 */
    ak->is_initialized = 1;
    ak->is_entered = 0;
    
    /* 初始化电机参数（软限位与量程）：不赋值会导致限幅全钳 0 且除零 */
    ak->params.pos_min = P_MIN;
    ak->params.pos_max = P_MAX;
    ak->params.vel_min = V_MIN;
    ak->params.vel_max = V_MAX;
    ak->params.tor_min = T_MIN;
    ak->params.tor_max = T_MAX;
    ak->params.kp_min  = KP_MIN;
    ak->params.kp_max  = KP_MAX;
    ak->params.kd_min  = KD_MIN;
    ak->params.kd_max  = KD_MAX;
    
    /* 清空状态 */
    memset(&ak->status, 0, sizeof(AK_MotorStatus_t));
    ak->status.id = motor_id;
    
    return HAL_OK;
}
//函数2：进入电机控制模式
HAL_StatusTypeDef AK_Motion_Enter(AK_Handle_t*ak)
{
    if (!ak->is_initialized || ak->pcan_handle == NULL) {
        return HAL_ERROR;
    }
    
    if (ak->is_entered) {
        return HAL_OK;
    }
    
    send_special_command(ak,CMD_ENTER_MODE);
    ak->is_entered = 1;
    
    return HAL_OK;
}
//函数3：退出电机控制模式
HAL_StatusTypeDef AK_Motion_Exit(AK_Handle_t*ak)
{
    if (!ak->is_initialized || ak->pcan_handle == NULL) {
        return HAL_ERROR;
    }
    
    if (!ak->is_entered) {
        return HAL_OK;
    }
    
    send_special_command(ak,CMD_EXIT_MODE);
    ak->is_entered = 0;
    
    return HAL_OK;
}
//函数4：最终控制函数
HAL_StatusTypeDef AK_Motion_Control(AK_Handle_t*ak,float position, float speed, 
                                      float kp, float kd, float torque)
{
    if (!ak->is_initialized || ak->pcan_handle == NULL) {
        return HAL_ERROR;
    }
    
    if (!ak->is_entered) {
        return HAL_ERROR;
    }
    
    AK_MotorParams_t *p = &ak->params;
    
    /* 参数限幅 */
    position = CLAMP(position, p->pos_min, p->pos_max);
    speed = CLAMP(speed, p->vel_min, p->vel_max);
    kp = CLAMP(kp, p->kp_min, p->kp_max);
    kd = CLAMP(kd, p->kd_min, p->kd_max);
    torque = CLAMP(torque, p->tor_min, p->tor_max);
    
    /* 浮点数转整数 */
    int p_int = float_to_uint(position, p->pos_min, p->pos_max, 16);
    int v_int = float_to_uint(speed, p->vel_min, p->vel_max, 12);
    int kp_int = float_to_uint(kp, p->kp_min, p->kp_max, 12);
    int kd_int = float_to_uint(kd, p->kd_min, p->kd_max, 12);
    int t_int = float_to_uint(torque, p->tor_min, p->tor_max, 12);
    
    /* 打包数据 (协议格式见手册5.3节) */
		//需要确认
    uint8_t data[8];
    data[0] = (p_int >> 8) & 0xFF;
    data[1] = p_int & 0xFF;
    data[2] = (v_int >> 4) & 0xFF;
    data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);
    data[4] = kp_int & 0xFF;
    data[5] = (kd_int >> 4) & 0xFF;
    data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);
    data[7] = t_int & 0xFF;
    
    return can_send_data(ak,ak->motor_id, data, 8);
}