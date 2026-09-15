#include "Moto_Task.h"
#include "Control_torque.h"   /* 重力补偿前馈力矩 Control_Torque_FeedForward */
#include "bsp_dwt.h"          /* DWT 高精度计时器 */

/* ==================== 控制周期定义 ==================== */
/* 固定 1kHz 控制频率。注意：CPU_CYCLE_PER_US 必须和 DWT_Init(240) 的主频一致 */
#define CONTROL_PERIOD_US  1000U   /* 控制周期 1000us = 1kHz */
#define CPU_CYCLE_PER_US   240U    /* 每微秒的时钟周期数：240MHz 下为 240 */

/* ==================== 全局控制变量定义 ==================== */

/* 当前控制模式：上电默认进"归零模式"，你可在 debug 里改成 1 切到位置控制 */
volatile control_mode_t g_ctrl_mode   = MODE_ZERO;
float tau_sh;   /* 大臂（肩）前馈力矩 */
float tau_el;   /* 小臂（肘）前馈力矩 */
float el05_target,dt;


/* 位置控制指令：debug 里改这四个字段 */
pos_cmd_t g_pos_cmd = 
{
    .target_x_s = 0.22f,   /* 默认末端落点 x */
    .target_z_s = 0.23f,   /* 默认末端落点 z */
    .target_yaw = 0.0f,   /* 0 = 保持当前末端姿态 */
    .duration   = 0.5f,   /* 默认 0.5 秒走完 */
};

/* 电机位置环控制参数：debug 里可调，觉得软就加大 Kp，抖就加大 Kd */
float g_kp     = 15.0f;   /* 位置环 Kp */
float g_kd     = 1.5f;    /* 位置环 Kd */
float g_torque = 0.0f;    /* 前馈力矩，默认 0 */

float g_kp_EL     = 40.0f;   /* 位置环 Kp */
float g_kd_EL     = 1.0f;    /* 位置环 Kd */
float g_torque_EL = 0.0f;    /* 前馈力矩，默认 0 */

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

    /* 腕部电机 EL05：先延时 10ms 等 EL05 启动，再初始化 + 进入运控模式 */
    osDelay(1);
    EL05_Motion_Init(&g_el05, &hfdcan1, EL05_ID);
    EL05_Motion_Enter(&g_el05);
}

/* ==================== 任务主体 ==================== */
void Moto_Diver(void *argument)
{
    /* 1. 初始化三个电机 */
    Motor_Init();

    /* 2. 初始化 DWT 计时器（CPU 240MHz），并记录周期基准用于测 dt */
    DWT_Init(240);
    uint32_t last_cycle = DWT->CYCCNT;   /* 上次周期时刻，供 DWT_GetDeltaT 更新 */

    for (;;)
    {
        /* 记录本周期开始时刻（用于后面固定周期补齐） */
        uint32_t cycle_start = DWT->CYCCNT;

        /* ========== 第一步：用 DWT 测本周期真实耗时 dt（秒） ==========*/
        /* dt 是"距离上一次循环"的真实时间，供重力补偿变化率限制和后续力控使用 */
        dt = DWT_GetDeltaT(&last_cycle);

        /* ========== 第二步：电机错误监测 ==========*/
        AK_Error_Monitor();

        /* ========== 第三步：正运动学刷新 ==========*/
        Forward_Kinematics(NULL);

        /* ========== EL05 速度前馈：预测大小臂运动，让 EL05 提前跟上 ==========*/
        /* 末端绝对角 abs 保持恒定时，EL05 角速度应等于小臂绝对角速度 q12_dot
         * = q1_dot + q2_dot = AK80_DIR*speed1 + AK45_DIR*speed2 */
        float el05_vel = AK80_DIR * g_ak80.status.speed + AK45_DIR * g_ak45.status.speed;

        if (g_ctrl_mode == MODE_POSITION)
        {
            /* 用 debug 里填的"末端落点"构造目标 */
            leg_pos_t target = {0};
            target.x_s = g_pos_cmd.target_x_s;   /* 目标末端 x */
            target.z_s = g_pos_cmd.target_z_s;   /* 目标末端 z */
            target.yaw = EL05_HORIZONTAL_ABS_ANGLE;   /* 末端姿态固定：始终指向 -x、平行 x 轴 */

            /* 填的是 x_s/z_s（末端落点），用 EE 版逆解算 */
            int ret = Inverse_Kinematics_EE(&target, ELBOW_AUTO);

            if (ret == 0)
            {
								tau_sh = 0.0f;   /* 大臂（肩）前馈力矩 */
								tau_el = 0.0f;   /* 小臂（肘）前馈力矩 */
							  Control_Torque_FeedForward(&tau_sh, &tau_el, 0, dt);
                /* 下发"电机角"（已含零偏换算），不是关节角 q1/q2 */
                AK_Motion_Control(&g_ak80, leg_motion.motor1_target_angle, 0, g_kp, g_kd, tau_sh);
                AK_Motion_Control(&g_ak45, leg_motion.motor2_target_angle, 0, g_kp, g_kd, tau_el);
							
//								AK_Motion_Control(&g_ak80, 0, 0, 0, 0, 0);
//                AK_Motion_Control(&g_ak45, 0, 0, 0, 0, 0);

                /* 腕部 EL05：始终保持水平且指向 -x，目标角由目标关节角反推 */
                el05_target = EL05_Calc_Horizontal_Angle(motor1_to_joint(g_ak80.status.position),motor2_to_joint(g_ak45.status.position));
								osDelay(1);
								EL05_Motion_Control(&g_el05, el05_target, 0, g_kp_EL, g_kd_EL, 0.0f);
            }
        }
        else if (g_ctrl_mode == MODE_TORQUE)
        {
            /* ===== 纯力矩重力补偿测试（零力拖动） =====
             * 目的：验证重力补偿是否准确。kp=kd=0，电机只输出重力补偿力矩，
             *       理想情况下把臂掰到哪，它就该停在哪（像"失重"一样）。 */
            tau_sh = 0.0f;   /* 大臂（肩）前馈力矩 */
            tau_el = 0.0f;   /* 小臂（肘）前馈力矩 */

            /* 计算重力补偿前馈力矩（0=空载，1=带负载），用 DWT 实测 dt */
            Control_Torque_FeedForward(&tau_sh, &tau_el, 0, dt);

            /* 纯力矩下发：kp=0、kd=0，只给 tau 前馈。
             * 若放开后振荡，可把 kd 改成 0.1~0.5 加一点阻尼 */
            AK_Motion_Control(&g_ak80, 0, 0, 0, 0, tau_sh);
            AK_Motion_Control(&g_ak45, 0, 0, 0, 0, tau_el);

            /* 腕部 EL05 暂不参与测试，保持当前姿态 */
						el05_target = EL05_Calc_Horizontal_Angle(motor1_to_joint(g_ak80.status.position), motor2_to_joint(g_ak45.status.position));
					osDelay(1);
						            EL05_Motion_Control(&g_el05, el05_target, 0, g_kp_EL, g_kd_EL, 0.0f);
        }
        else
        {
            /* MODE_ZERO：归零（电机角直接给 0） */
                AK_Motion_Control(&g_ak80, 0, 0, g_kp, g_kd, 0);
                AK_Motion_Control(&g_ak45, 0, 0, g_kp, g_kd, 0);
            el05_target = EL05_Calc_Horizontal_Angle(motor1_to_joint(g_ak80.status.position), motor2_to_joint(g_ak45.status.position));
					osDelay(1);
						            EL05_Motion_Control(&g_el05, el05_target, 0, g_kp_EL, g_kd_EL, 0.0f);
        }

//        /* ========== 第四步：用 DWT 精确补齐到固定 1kHz 周期 ==========*/
//        /* 计算本周期已耗时，若不足 1ms 就用 DWT 忙等补齐（替代原来的 osDelay(1)） */
//        uint32_t elapsed_us = (DWT->CYCCNT - cycle_start) / CPU_CYCLE_PER_US;
//        if (elapsed_us < CONTROL_PERIOD_US)
//        {
//            DWT_Delay_us(CONTROL_PERIOD_US - elapsed_us);
//        }
    }
}
