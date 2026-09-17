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
/* 位置规划触发标志：debug 里置 1，触发一次新的末端位置规划（触发后自动清零） */
volatile int            g_cmd_trigger = 0;
float tau_sh;   /* 大臂（肩）前馈力矩 */
float tau_el;   /* 小臂（肘）前馈力矩 */
float el05_target, dt;   /* EL05 目标角；dt 本周期真实耗时（秒） */
int ret;

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
            /* ===== 触发检测：debug 里置 g_cmd_trigger=1，启动一次新的位置规划 ===== */
            if (g_cmd_trigger)
            {
                g_cmd_trigger = 0;   /* 立即清标志，避免下个周期重复触发 */

                /* 用 debug 里填的"末端落点"构造目标 */
                leg_pos_t target = {0};
                target.x_s = g_pos_cmd.target_x_s;   /* 目标末端 x */
                target.z_s = g_pos_cmd.target_z_s;   /* 目标末端 z */
                target.yaw = EL05_HORIZONTAL_ABS_ANGLE;   /* 末端姿态固定：始终指向 -x、平行 x 轴 */

                /* 填的是 x_s/z_s（末端落点），用 EE 版逆解算，得到目标关节角并写进 leg_motion */
								if(target.x_s<=0)
								{
									ret = Inverse_Kinematics_EE(&target, ELBOW_AUTO);
								}
								else if(target.x_s>=0)
								{
									ret = Inverse_Kinematics_EE(&target, ELBOW_UP);
								}
                if (ret == 0)
                {
                    /* 读当前实际关节角（已减零偏、方向换算），作为轨迹起点 */
                    float q1_cur = motor1_to_joint(g_ak80.status.position);   /* 大臂当前关节角 */
                    float q2_cur = motor2_to_joint(g_ak45.status.position);   /* 小臂当前关节角 */

                    /* 自动算轨迹时长：按最大关节速度 + 两个关节的转角差 */
                    float T = calc_motion_time(q1_cur, q2_cur,
                                               leg_motion.q1_target, leg_motion.q2_target);

                    /* 启动五次多项式轨迹：从当前角平滑走到目标角（内部预计算 6 个系数） */
                    jtraj_start(q1_cur, q2_cur,
                                leg_motion.q1_target, leg_motion.q2_target, T);
                }
            }

            /* ===== 每周期推进轨迹，得到平滑的位置/速度/加速度 ===== */
            /* 返回值：0=运行中，1=刚完成，-1=未激活 */
            int traj_state = jtraj_update(&leg_motion);

            /* ===== 根据轨迹状态决定本周期下发的目标 ===== */
            float motor1_cmd, motor2_cmd;   /* 下发的大臂/小臂电机角 (rad) */
            float vel1_cmd,   vel2_cmd;     /* 下发的大臂/小臂速度前馈 (rad/s) */

            if (traj_state >= 0)
            {
                /* 轨迹有效（运行中/刚完成）：用平滑后的电机角 + 速度前馈 */
                motor1_cmd = leg_motion.motor1_target_angle;   /* 大臂平滑目标电机角 */
                motor2_cmd = leg_motion.motor2_target_angle;   /* 小臂平滑目标电机角 */
                vel1_cmd   = AK80_DIR * leg_motion.joint1_speed_target;   /* 关节速度 → 电机速度 */
                vel2_cmd   = AK45_DIR * leg_motion.joint2_speed_target;   /* 关节速度 → 电机速度 */
            }
            else
            {
                /* 轨迹未激活：保持当前电机角不动，防止长时间无指令导致电机超时失能 */
                motor1_cmd = g_ak80.status.position;   /* 大臂保持当前反馈角 */
                motor2_cmd = g_ak45.status.position;   /* 小臂保持当前反馈角 */
                vel1_cmd   = 0.0f;                      /* 无前馈速度 */
                vel2_cmd   = 0.0f;                      /* 无前馈速度 */
            }

            /* 重力补偿前馈力矩（空载） */
            tau_sh = 0.0f;   /* 大臂（肩）前馈力矩先清零 */
            tau_el = 0.0f;   /* 小臂（肘）前馈力矩先清零 */
            Control_Torque_FeedForward(&tau_sh, &tau_el, 0, dt);

            /* 下发大臂/小臂：位置 + 速度前馈 + 重力补偿力矩 */
            AK_Motion_Control(&g_ak80, motor1_cmd, vel1_cmd, g_kp, g_kd, tau_sh);
            AK_Motion_Control(&g_ak45, motor2_cmd, vel2_cmd, g_kp, g_kd, tau_el);

            /* 腕部 EL05：始终保持水平且指向 -x，目标角由当前关节角反推 */
            el05_target = EL05_Calc_Horizontal_Angle(motor1_to_joint(g_ak80.status.position),
                                                     motor2_to_joint(g_ak45.status.position));
            osDelay(1);
            EL05_Motion_Control(&g_el05, el05_target, 0, g_kp_EL, g_kd_EL, 0.0f);
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
            el05_target = EL05_Calc_Horizontal_Angle(motor1_to_joint(g_ak80.status.position),
                                                     motor2_to_joint(g_ak45.status.position));
            osDelay(1);
            EL05_Motion_Control(&g_el05, el05_target, 0, g_kp_EL, g_kd_EL, 0.0f);
        }
        else
        {
            /* MODE_ZERO：归零（电机角直接给 0） */
            AK_Motion_Control(&g_ak80, 0, 0, g_kp, g_kd, 0);
            AK_Motion_Control(&g_ak45, 0, 0, g_kp, g_kd, 0);
            el05_target = EL05_Calc_Horizontal_Angle(motor1_to_joint(g_ak80.status.position),
                                                     motor2_to_joint(g_ak45.status.position));
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
