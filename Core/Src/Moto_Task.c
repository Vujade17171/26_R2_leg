#include "Moto_Task.h"

/* ==================== 全局控制变量定义 ==================== */

/* 当前控制模式：上电默认进"归零模式"，你可在 debug 里改成 1 切到位置控制 */
volatile control_mode_t g_ctrl_mode   = MODE_ZERO;

/* 位置控制指令：debug 里改这四个字段 */
pos_cmd_t g_pos_cmd = 
{
    .target_x_s = 0.22f,   /* 默认末端落点 x = 0 */
    .target_z_s = 0.23f,   /* 默认末端落点 z = 0 */
    .target_yaw = 0.0f,   /* 0 = 保持当前末端姿态 */
    .duration   = 0.5f,   /* 默认 0.5 秒走完 */
};

/* 电机位置环控制参数：debug 里可调，觉得软就加大 Kp，抖就加大 Kd */
float g_kp     = 10.0f;   /* 位置环 Kp */
float g_kd     = 1.5f;    /* 位置环 Kd */
float g_torque = 0.0f;    /* 前馈力矩，默认 0 */

/* 归零动作时长：1 秒平滑回到零点 */
#define ZERO_DURATION 1.0f

/* ==================== 电机初始化 ==================== */
static void Motor_Init(void)
{
    /* 配置 CAN 接收过滤器，启动 CAN 并开启接收中断 */
    can_filter_init();

    /* 大臂电机 AK80：初始化 + 进入运控模式（进入后开始回传反馈） */
    AK_Motion_Init(&g_ak80, &hfdcan1, AK80_ID);
    AK_Motion_Enter(&g_ak80);

    /* 小臂电机 AK45：初始化 + 进入运控模式 */
    AK_Motion_Init(&g_ak45, &hfdcan1, AK45_ID);
    AK_Motion_Enter(&g_ak45);

    /* 腕部电机 EL05：初始化 + 进入运控模式 */
		osDelay(10);
    EL05_Motion_Init(&g_el05, &hfdcan1, EL05_ID);
    EL05_Motion_Enter(&g_el05);
}

/* ==================== 任务主体 ==================== */
void Moto_Diver(void *argument)
{
    /* 1. 初始化三个电机 */
    Motor_Init();
    for (;;)
    {
        /* ========== 第一步：电机错误监测 ==========*/
        AK_Error_Monitor();

        /* ========== 第二步：正运动学刷新 ==========*/
        Forward_Kinematics(NULL);

        if (g_ctrl_mode == MODE_POSITION)
        {
            /* 用 debug 里填的"末端落点"构造目标 */
            leg_pos_t target = {0};
            target.x_s = g_pos_cmd.target_x_s;   /* 目标末端 x */
            target.z_s = g_pos_cmd.target_z_s;   /* 目标末端 z */
            target.yaw = g_pos_cmd.target_yaw;   /* 目标末端姿态（0=保持） */

            /* 填的是 x_s/z_s（末端落点*/
            int ret = Inverse_Kinematics_EE(&target, ELBOW_AUTO);

            if (ret == 0)
            {
                /* 下发"电机角"（已含零偏换算），不是关节角 q1/q2 */
                AK_Motion_Control(&g_ak80, leg_motion.motor1_target_angle, 0, g_kp, g_kd, 0);
                AK_Motion_Control(&g_ak45, leg_motion.motor2_target_angle, 0, g_kp, g_kd, 0);
								osDelay(1);
                /* 暂不控制灵足05，这里仅保持当前姿态 */
                EL05_Motion_Control(&g_el05, leg_motion.motor3_target_angle , 0.0f, g_kp, g_kd, 0.0f);
            }
        }
        else
        {
            /* MODE_ZERO：归零（电机角直接给 0） */
            AK_Motion_Control(&g_ak80, 0, 0, g_kp, g_kd, 0);
            AK_Motion_Control(&g_ak45, 0, 0, g_kp, g_kd, 0);
						osDelay(1);
            EL05_Motion_Control(&g_el05, 0, 0.0f, g_kp, g_kd, 0.0f);
        }

        osDelay(1);
    }
}
