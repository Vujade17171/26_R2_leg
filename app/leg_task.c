/*****************************************************************************
 * leg_task.c —— 应用层：Leg_Task 任务实现
 *
 * 分层位置：app —— 只做"业务决策 + 节拍编排 + 对外接口"，
 *   具体控制算法全部下沉到 ser 层（LegMotion）。
 *   允许依赖：ser、osal、common（以及 bsp 的资源标识符如 BSP_CAN_BUS_1）
 *   禁止依赖：HAL 具体外设调用
 *
 * 本文件的职责边界（旧版 359 行 -> 现约 200 行，且不再有任何控制算法）：
 *   1) 创建/初始化设备实例并注入给各服务（依赖注入，模块内不 extern）
 *   2) 电机使能、等待反馈、上电姿态初始化
 *   3) 主循环节拍：实测 dt -> LegMotion_Step -> 1ms -> WristStep -> 1ms
 *   4) 调试量发布（保持旧变量名，MDK Watch 表达式无需改动）
 *   5) 对外业务接口 arm_control / leg_move_to
 *****************************************************************************/
#include "leg_task.h"

#include <stddef.h>   /* NULL：LegMotion_Init 的 cfg / grav_cfg 传 NULL 表示用默认值 */

#include "leg_config.h"
#include "leg_types.h"

#include "bsp_can.h"      /* 仅用总线编号常量；外设初始化在 main.c */
#include "osal_time.h"
#include "osal_task.h"

#include "ak_motor.h"
#include "el05_motor.h"
#include "can_router.h"
#include "rc_receiver.h"

/* leg_motion.h 已内含 kinematics / joint_traj / motion_alg：
 * app 只依赖 ser/leg 的对外接口，不直接包含 ser/algo 的内部头。 */
#include "leg_motion.h"

/* ================== 设备实例（由 app 持有并注入） ==================
 * 变量名与旧版保持一致，便于沿用原来的 Watch 表达式。 */
AK_Motor   motors[APP_AK_NUM];              /* motors[0]=大臂(肩) motors[1]=小臂(肘) */
EL05_Motor el05_motors[APP_EL05_NUM];       /* el05_motors[0]=腕 */

extern RC_Data rc;

#define AK_MOTOR_NUM    ((uint8_t)(sizeof(motors) / sizeof(motors[0])))
#define EL05_MOTOR_NUM  ((uint8_t)(sizeof(el05_motors) / sizeof(el05_motors[0])))

/* ================== 运动服务实例 ================== */
static LegMotion g_leg;
float X=0.2f;
float Z=0.3f;

/* 连杆参数：大臂/小臂/腕部长度，单位 m */
static const LegLinkParam s_link = { D_L1, D_L2, D_L3 };

/* ================== 调试观测量（Watch 用，名称与旧版一致） ================== */
volatile float         b, c, d;                  /* b=肩反馈角 c=肘反馈角 d=腕前馈力矩 */
volatile LegJointAngles q_cmd_dbg;               /* 限速后的指令关节角 */
volatile LegJointVec   v_cmd_dbg;                /* 本拍前馈速度 rad/s */
volatile LegJointVec   a_cmd_dbg;                /* 本拍前馈加速度 rad/s² */
volatile float         dbg_tau;                  /* 轨迹进度 0~1 */
volatile uint8_t       dbg_traj_state;           /* JTRAJ_* */
volatile float         dbg_dt;                   /* 实测控制周期 s */
LegJointAngles motor_angles;                     /* 反馈换算出的关节角（q3 恒 0） */
FootPosition   FK_pos;                           /* 正解得到的当前末端位置 */
FootPosition   target_pos;                       /* 目标末端位置 x,z */

/* ================== 上电使能握手观测量（Watch 用） ================== */
volatile uint32_t dbg_fb_mask;       /* 谁回过话：bit0=AK肩 bit1=AK肘 bit2=EL05腕，7=都答了 */
volatile uint32_t dbg_enable_sends;  /* 握手一共发了几轮使能 */
volatile uint32_t dbg_bus1_frames;   /* 握手结束时 CAN1 收到的帧数 */
volatile uint8_t  dbg_bus1_busoff;   /* CAN1 是否 Bus-Off（1=是） */

/* ================== 内部函数 ================== */
static void app_publish_debug(float dt);

/* ============================================================================
 * 上电使能握手用的小工具
 * ========================================================================== */
#define FB_MASK_AK_SHOULDER  (1u << 0)
#define FB_MASK_AK_ELBOW     (1u << 1)
#define FB_MASK_EL05_WRIST   (1u << 2)
#define FB_MASK_ALL          (FB_MASK_AK_SHOULDER | FB_MASK_AK_ELBOW | FB_MASK_EL05_WRIST)

/* 哪些驱动板已经真的回过反馈帧。
 * 用各驱动自己的 rx_cnt，而不是"CAN1 上有没有帧"：必须确认 AK×2 + EL05
 * 都答过话，否则可能只等到其中一块板的帧就以为"都好了"。 */
static uint32_t leg_fb_mask(void)
{
    uint32_t m = 0u;

    if (motors[0].rx_cnt != 0u)      { m |= FB_MASK_AK_SHOULDER; }
    if (motors[1].rx_cnt != 0u)      { m |= FB_MASK_AK_ELBOW;    }
    if (el05_motors[0].rx_cnt != 0u) { m |= FB_MASK_EL05_WRIST;  }
    return m;
}

/* 发一遍三个电机的使能：AK = 进入运控模式(FF×7+FC)，EL05 = 通信类型3 */
static void leg_enable_all(void)
{
    (void)AK_Motor_Enable(&motors[0]);
    (void)AK_Motor_Enable(&motors[1]);
    (void)EL05_Enable(&el05_motors[0]);
}

/* ============================================================================
 * 对外业务接口
 * ============================================================================ */

/* 设置笛卡尔目标点 x,z（签名与原来一致）：只登记目标，不规划、不下发。
 * duration_s 传 0 表示沿用当前规划时长。 */
void arm_control(float x, float y)
{
    LegMotion_SetTarget(&g_leg, x, y, 0.0f);
}

/* 带时长的目标入口：duration_s <= 0 时用默认规划时长 */
void leg_move_to(float x, float z, float duration_s)
{
    LegMotion_SetTarget(&g_leg, x, z,
                        (duration_s > 0.0f) ? duration_s : LEG_MOTION_CFG_DEFAULT.traj_duration_s);
}

/* ============================================================================
 * 调试量发布：把服务层状态映射到全局变量，保持旧 Watch 体验
 * ============================================================================ */
static void app_publish_debug(float dt)
{
#if (APP_PUBLISH_DEBUG != 0)
    motor_angles = g_leg.fb;
    motor_angles.q3 = 0.0f;          /* 与旧实现一致：FK 用的关节角腕通道记 0 */
    FK_pos         = g_leg.foot_fb;
    target_pos     = g_leg.target;
    q_cmd_dbg      = g_leg.q_cmd;
    v_cmd_dbg      = g_leg.v_cmd;
    a_cmd_dbg      = g_leg.a_cmd;
    dbg_tau        = g_leg.traj_tau;
    dbg_traj_state = g_leg.traj_state;
    dbg_dt         = dt;
    b              = g_leg.fb.q1;
    c              = g_leg.fb.q2;
    d              = g_leg.tau_wr;
#else
    (void)dt;
#endif
}

/* ============================================================================
 * 任务入口（强定义，覆盖 freertos.c 中的 __weak leg_task）
 * ============================================================================ */
void leg_task(void *argument)
{   
    int16_t last_ch2 = 0u; 
    uint32_t dwt_cnt = 0u;
    uint32_t tick;
    float    dt;

    (void)argument;

    /* ---- 1. 设备初始化 ---- */
	OSAL_Time_DelayMs(400u);
    (void)AK_Motor_Init(&motors[0], BSP_CAN_BUS_1, APP_AK_ID_SHOULDER, &AK_MODEL_AK80_9);   /* 大臂 */
    (void)AK_Motor_Init(&motors[1], BSP_CAN_BUS_1, APP_AK_ID_ELBOW,    &AK_MODEL_AK45_10);  /* 小臂 */
    (void)EL05_Init(&el05_motors[0], BSP_CAN_BUS_1, APP_EL05_ID_WRIST,
                    APP_EL05_MASTER_ID, &EL05_MODEL);                                       /* 腕   */

    /* ---- 2. 帧路由：先注册接收者，再把句柄数组注入（依赖注入） ---- */
    (void)CAN_Router_Init(BSP_CAN_BUS_1);
    (void)CAN_Router_BindAk(motors, AK_MOTOR_NUM);
    (void)CAN_Router_BindEl05(el05_motors, EL05_MOTOR_NUM);

    /* ---- 3. 运动服务：注入设备句柄与连杆参数（内部会调用 Kinematics_Init） ---- */
    (void)LegMotion_Init(&g_leg, motors, AK_MOTOR_NUM, el05_motors, EL05_MOTOR_NUM,
                         &s_link, NULL, NULL);

    /* ---- 4. 电机使能 ---- */
    (void)AK_Motor_Enable(&motors[0]);
    (void)AK_Motor_Enable(&motors[1]);
    OSAL_Time_DelayMs(1u);
    (void)EL05_Enable(&el05_motors[0]);
     
		

    /* ---- 4.1 使能握手（★ 上电"机械臂完全没反应"就出在这里）----
     * 上面那几条只发了【一次】——驱动板还没起来就丢了，丢了不补发就永远失能。
     * 这里先留 APP_DRIVER_BOOT_DELAY_MS 的上电时间，然后每
     * APP_ENABLE_RETRY_INTERVAL_MS 重发一遍使能，直到 AK×2 + EL05 都回过反馈帧
     * （上限 APP_ENABLE_RETRY_MAX 轮 ≈ 3s，超时也往下走，不卡死腿任务）。
     * 期间处理 Bus-Off：没人 ACK 时连续重发会把 FDCAN1 打进 Bus-Off，
     * 进去之后不恢复就再也发不出去、也收不到。 */
    OSAL_Time_DelayMs(APP_DRIVER_BOOT_DELAY_MS);

    dbg_enable_sends = 1u;                 /* 上面已经发过一轮 */
    dbg_fb_mask      = 0u;
    while (dbg_enable_sends < APP_ENABLE_RETRY_MAX)
    {
        leg_enable_all();

        (void)BSP_CAN_RecoverIfBusOff(BSP_CAN_BUS_1);

        OSAL_Time_DelayMs(APP_ENABLE_RETRY_INTERVAL_MS);

        dbg_fb_mask = leg_fb_mask();
        if (dbg_fb_mask == FB_MASK_ALL) { break; }   /* 三块板都回话了：握手完成 */

        dbg_enable_sends++;
    }

    dbg_bus1_frames = CAN_Router_GetFrameCount(BSP_CAN_BUS_1);
    dbg_bus1_busoff = BSP_CAN_IsBusOff(BSP_CAN_BUS_1);

    /* ---- 5. 等驱动板回传反馈（最多 APP_RX_WAIT_TIMEOUT_MS）----
     * 有真实反馈后再取初始姿态，否则 pos_rad 仍为 0，
     * 会算出虚假姿态当目标 → 上电大角度跳动 */
    {
        uint32_t wait_ms = 0u;

        while ((CAN_Router_GetFrameCount(BSP_CAN_BUS_1) == 0u) && (wait_ms < APP_RX_WAIT_TIMEOUT_MS)) {
            OSAL_Time_DelayMs(1u);
            wait_ms++;
        }
    }

    /* ---- 6. 上电初值：以"当前实际末端位置"为目标，先保持姿态（避免上电跳变） ---- */
//    (void)LegMotion_InitPose(&g_leg);

    /* ---- 7. 预置 DWT 计数，避免首拍算出巨大的 dt ---- */
     OSAL_Time_ResetDt(&dwt_cnt);
       
    tick = OSAL_Time_GetTick();

    for (;;)
    {
        /* 实测控制周期：轨迹推进 / 限速 / 前馈共用同一个 dt */ //(rc.num-1000) / 1000.0f *0.4f +0.2f)
			if(rc.s[2] == 1 && rc.s[0] == 0){
                while(rc.ch[2] >7 || rc.ch[2] < -7){
                    if(rc.ch[2]>0 && rc.ch[2] > last_ch2){
                        X= rc.ch[2]/500.0f * 0.3f ;
                        Z=(rc.num-1000) / 1000.0f * 0.4f +0.2f;
                    }
                    else if(rc.ch[2]<0 && rc.ch[2] < last_ch2){
                        X=rc.ch[2]/500.0f * 0.3f;
                        Z=(rc.num-1000) / 1000.0f * 0.4f +0.2f;
                    }
                    last_ch2 = rc.ch[2];
										OSAL_Time_DelayMs(1u);
                }
			}
		  	arm_control(X,Z);
            dt = OSAL_Time_Dt(&dwt_cnt);

			if (dt <= 0.0f)             { dt = APP_CTRL_DT_FALLBACK_S; }  /* 异常周期：退回名义值 */
			if (dt > APP_CTRL_DT_MAX_S) { dt = APP_CTRL_DT_MAX_S; }        /* 卡顿/断点：上限 50ms */

			/* AK 侧：目标管理 -> 轨迹 -> 限速 -> 重力补偿 -> 下发 */
			(void)LegMotion_Step(&g_leg, dt);
			app_publish_debug(dt);

			/* 第 1 个 1ms 节拍 */
			(void)OSAL_Task_DelayUntil(&tick, 1u);
 
//			/* 两次节拍之间：EL05 腕侧（保持原实现的时隙位置） */
            if(rc.s[0] == 1){
			    (void)LegMotion_WristStep(&g_leg, 1, (rc.num-1500.0f) / 500.0f * (PI/3.0f));
            }else{
                /* 手腕控制关闭 */
                (void)LegMotion_WristStep(&g_leg, 0, 0.0f);
            }

			/* 第 2 个 1ms 节拍：回到 AK 帧，故 AK 实际周期 = 2ms */
			(void)OSAL_Task_DelayUntil(&tick, 1u);
			
    }
}