#include "chassis_task.h"
#include "vesc_motor.h"
#include "chassis_config.h"
#include "can_router.h"
#include "osal_time.h"
#include "chassis_types.h"
#include "rc_receiver.h"   /* 遥控器：只暴露 RC_Data / RC_RegisterHandle / RC_Init / RC_Get */

VESC_Motor vesc_motor[APP_VESC_NUM];
static ChassisPosition target_speed = {0.0f, 0.0f, 0.0f};

/* ★ 遥控器数据：这里是【实例】，不是指针。
 *   类型在 ser 定义，实例由 app 定义 —— 和 vesc_motor[] / AK_Motor 一样。
 *   写成指针（RC_Data *rc）再传 &rc 会给 RC_RegisterHandle 一个 RC_Data**，
 *   被当成 RC_Data* 用 -> 每帧解包都往指针变量上写 -> 踩内存。 */
volatile RC_Data rc;

/* 调试观测量（Watch 用） */
uint8_t rc_lost = 1u;   /* 失联标志：1 = 失联，0 = 在线 */

ChassisVelocity abcd={0};

#define VESC_MOTOR_NUM    ((uint8_t)(sizeof(vesc_motor) / sizeof(vesc_motor[0])))

/* ============================================================================
 * 遥控失联保护
 *
 *   rc.frames 是 ser 层累计的【有效帧数】—— 帧长、帧头、校验和都过了才 +1。
 *   只要它在增长就说明遥控在线；超过 APP_RC_LOST_TIMEOUT_MS 没增长就判失联，
 *   把 x / y / yaw 全部清零。
 *
 *   ★ 关键：失联时不是"不更新 target_speed"，而是【主动写 0 并下发】。
 *     否则 target_speed 会保持最后一拍的值，操作手松手/接收机掉电后
 *     底盘会一直按最后一条指令跑下去。
 * ========================================================================== */
static uint32_t rc_frames_last = 0u;   /* 上次看到的 frames 值 */
static uint32_t rc_last_rx_ms  = 0u;   /* 最后一次收到新帧的时刻(ms) */

static ChassisVelocity speed_to_rpm(ChassisVelocity wheel_speed)
{
    ChassisVelocity rpm_speed;

    for (uint8_t i = 0; i < APP_VESC_NUM; i++)
    {
        rpm_speed.speed[i] = wheel_speed.speed[i] * 60.0f/(PI * WHEEL_D);
    }

    return rpm_speed;
}

static ChassisVelocity chassis_inverse_kinematics(ChassisPosition target_speed)
{
    ChassisVelocity wheel_speed;

    // 计算每个轮子的速度
    wheel_speed.speed[0] = (-target_speed.y * 1.41421 + target_speed.x * 1.41421 + target_speed.yaw * CHASSIS_R) * GEAR_RATIO;
    wheel_speed.speed[1] = ( target_speed.y * 1.41421 + target_speed.x * 1.41421 + target_speed.yaw * CHASSIS_R) * GEAR_RATIO;
    wheel_speed.speed[2] = ( target_speed.y * 1.41421 - target_speed.x * 1.41421 + target_speed.yaw * CHASSIS_R) * GEAR_RATIO;
    wheel_speed.speed[3] = (-target_speed.y * 1.41421 - target_speed.x * 1.41421 + target_speed.yaw * CHASSIS_R) * GEAR_RATIO;

    return wheel_speed;
}

static void motor_speed_control(ChassisVelocity wheel_speed)
{
    wheel_speed = speed_to_rpm(wheel_speed);
	  abcd = wheel_speed;
    for (uint8_t i = 0; i < APP_VESC_NUM; i++)
    {
        VESC_Set_Rpm_Struct(&vesc_motor[i], (int32_t)(wheel_speed.speed[i]));
    }
}

static void chassis_motor(ChassisPosition target_speed){
    ChassisVelocity motor_speed;
    motor_speed = chassis_inverse_kinematics(target_speed);
    motor_speed_control(motor_speed);
}

static void chassis_motor_stop(void){

	  for (uint8_t i = 0; i < APP_VESC_NUM; i++)
    {
        VESC_Set_Current_Handbrake_Struct(&vesc_motor[i],BRAKE_CURRENT);
	}
}

void chassis_task(void *argument)
{
    (void)argument;

    /* ---- 遥控器：注册句柄 -> 初始化（顺序不能反） ---- */
     (void)RC_RegisterHandle(&rc);   /* 传【实例的地址】，和 CAN_Router_BindAk 一个意思 */
     (void)RC_Init();

	  VESC_Motor_Struct_Init(&vesc_motor[0], BSP_CAN_BUS_2, 91);   //右上
	  VESC_Motor_Struct_Init(&vesc_motor[1], BSP_CAN_BUS_2, 103);  //左上
    VESC_Motor_Struct_Init(&vesc_motor[2], BSP_CAN_BUS_2, 121);  //左下
    VESC_Motor_Struct_Init(&vesc_motor[3], BSP_CAN_BUS_2, 80);   //右下	
    (void)VESC_CAN_Router_Init(BSP_CAN_BUS_2);
    CAN_Router_VESC(vesc_motor, VESC_MOTOR_NUM);

    /* 失联保护从这一刻开始计时：上电就一直没信号也算失联 */
    rc_last_rx_ms = OSAL_Time_GetMs();

    for (;;)
    {
        uint32_t now_ms = OSAL_Time_GetMs();

        /* ---- 失联判定：frames 有变化 -> 刷新时间戳 ---- */
        if (rc.frames != rc_frames_last) {
            rc_frames_last = rc.frames;
            rc_last_rx_ms  = now_ms;
        }

        /* 无符号相减，tick 回绕也安全 */
        rc_lost = ((uint32_t)(now_ms - rc_last_rx_ms) > APP_RC_LOST_TIMEOUT_MS) ? 1u : 0u;

        
        if (rc_lost != 0u)
        {
            /* 失联：三轴目标全部清零（主动下发 0，不能只是不更新） */
            target_speed.x   = 0.0f;
            target_speed.y   = 0.0f;
            target_speed.yaw = 0.0f;
            chassis_motor(target_speed);
        }
        else
        {
            /* 在线：由遥控数据算目标 */
            if(rc.ch[0] < 10 && rc.ch[0] > -10 && rc.ch[1] < 10 && rc.ch[1] > -10 && rc.ch[2] < 10 && rc.ch[2] > -10){
                chassis_motor_stop();
            }
            else{
				if(rc.s[2] == 0){
                    target_speed.x   = (float)(rc.ch[0] / 500.0f * 1.0f);
                    target_speed.y   = (float)(rc.ch[1] / 500.0f * 1.0f);
                    target_speed.yaw = (float)(rc.ch[2] / 500.0f * 6.28f);
                
                    chassis_motor(target_speed);
				}
            }
        }

        OSAL_Time_DelayMs(1u);
    }
}
