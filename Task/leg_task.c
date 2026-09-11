/**
  ******************************************************************************
  * @file    leg_task.c
  * @brief   Leg_Task FreeRTOS 任务实现（空任务骨架）
  *
  * 说明：
  *  - 本文件提供 leg_task() 的强定义，覆盖 CubeMX 在 freertos.c 中生成的
  *    __weak void leg_task(void *argument)；
  *  - 任务配置（CubeMX .ioc）：名称 Leg_Task，优先级 osPriorityLow(8)，
  *    堆栈 256*4 字节，CMSIS-RTOS v2；
  *  - 在下方两个 USER CODE 区域中填写实际控制逻辑即可。
  ******************************************************************************
  */
#include "leg_task.h"
#include "ak_motor.h"
#include "fdcan.h"    
#include "kinematics.h"  /* 腿部运动学接口：FK/IK、关节角<->电机角换算、重力补偿等 */
#include "bsp_dwt.h"     /* DWT_GetDeltaT：实测调用周期 */
#include "Mycan.h"       /* g_rx_frames：反馈帧计数，用于等待驱动板反馈 */
#include "EL05_motor.h"  /* EL05 电机驱动（扩展帧私有协议，Mycan 分流用） */

/* 故障时"保持位姿"用的刚度/阻尼（比正常跟踪低，避免故障下继续大力输出） */
#define ARM_FAULT_HOLD_KP   40.0f
#define ARM_FAULT_HOLD_KD    1.5f

AK_Motor motors[2];  /* 电机句柄数组：motors[0]=大臂(肩)，motors[1]=小臂(肘) */

LegLinkParam leg_link_param = { D_L1, D_L2, D_L3 };  /* 连杆参数：大臂/小臂/腕部长度，单位 m */

LegJointAngles motor_angles = { 0.0f, 0.0f, 0.0f }; //当前关节角(由电机反馈换算)

FootPosition target_pos = { 0.1f, 0.3f }; //目标末端位置 x,z

FootPosition FK_pos = { 0.0f, 0.0f }; //正解得到的当前末端位置 x,z

float b,c,d;   /* 调试用：b=肩前馈, c=肘前馈, d=腕(EL05)前馈 */

/* 电机数量：供 Mycan 接收回调按 sizeof 自动计算，新增电机无需改这里 */
const uint8_t g_ak_motor_num = (uint8_t)(sizeof(motors) / sizeof(motors[0]));

/* EL05 电机句柄（扩展帧协议）；ID/主机ID 在 Init 时按实际设置 */
EL05_Motor el05_motors[1];
const uint8_t g_el05_motor_num = (uint8_t)(sizeof(el05_motors) / sizeof(el05_motors[0]));

/* 机械臂控制：输入目标末端位置 x,y，内部完成 逆解→前馈/重力补偿→正解→MIT 下发 */
void arm_control(float x, float y);


void leg_task(void *argument)
{


  AK_Motor_Init(&motors[0], &hfdcan1, 1, &AK_MODEL_AK80_9);   /* 大臂 */
  AK_Motor_Init(&motors[1], &hfdcan1, 2, &AK_MODEL_AK45_10);  /* 小臂 */
  EL05_Init(&el05_motors[0], &hfdcan1, 3, 0xFF, &EL05_MODEL);   /* EL05: 电机CAN_ID=3 */
  
  AK_Motor_Enable(&motors[0]); 
  AK_Motor_Enable(&motors[1]); 
  EL05_Enable(&el05_motors[0]);

  Kinematics_Init(&leg_link_param);
  DWT_Init(480);          /* CPU 480MHz：供 DWT_GetDeltaT 算实际周期 */
  osDelay(10);

  /* 等驱动板回传反馈（最多 500ms）：有真实反馈后再取初始姿态，
   * 否则 pos_rad 仍为 0，会算出虚假姿态当目标 → 上电大角度跳动 */
  {
    uint32_t wait_ms = 0u;
    while ((g_rx_frames == 0u) && (wait_ms < 500u)) { osDelay(1); wait_ms++; }
  }

  /* 上电初值：以"当前实际末端位置"为目标，先保持姿态(避免上电跳变) */
  motor_angles.q1 = motor_to_joint_1(motors[0].pos_rad);
  motor_angles.q2 = motor_to_joint_2(motors[1].pos_rad);
  motor_angles.q3 = 0.0f;
  Kinematics_Forward(&motor_angles, &target_pos);

  {
    uint32_t tick = osKernelGetTickCount();

    for (;;)
    {
      /* TODO: 目标点后续接轨迹规划输出；现在用 target_pos 里的常值 */
      arm_control(target_pos.x, target_pos.z);
      tick += 1u;              /* 1 tick = 1ms（configTICK_RATE_HZ = 1000） */
      osDelayUntil(tick);      /* 固定 1kHz 节拍，避免 osDelay(1) 的周期抖动 */
			EL05_MotionControl(&el05_motors[0], 0.0f, 0.0f,0.0f,0.0f, 0.0f);
    }
  }
}

/* ============================================================================
 *  机械臂控制算法（逆解 + 前馈/重力补偿 + 正解 + MIT 下发）
 *  输入：x, y —— 目标末端位置(m)，y 对应竖直方向(即 FK 的 z)
 *  motors[0] = 大臂(肩)，motors[1] = 小臂(肘)
 *  使用文件内已定义：motors[] / motor_angles / target_pos / FK_pos
 * ==========================================================================*/
void arm_control(float x, float y)
{
    float q3_now = 0.0f;                                  /* 第三关节(EL05)角度 */
    float tau_sh = 0.0f, tau_el = 0.0f, tau_wr = 0.0f;    /* 重力补偿力矩 N·m */
    FootPosition target;
    LegJointAngles q_des;

    /* 0. 安全检查：任一电机报错 → 保持当前位姿(不再跟踪目标)，等故障消失自动恢复 */
    if ((motors[0].err_code != 0u) || (motors[1].err_code != 0u)) {
        float q1_hold = motor_to_joint_1(motors[0].pos_rad);
        float q2_hold = motor_to_joint_2(motors[1].pos_rad);

        AK_Motor_MIT(&motors[0], joint_to_motor_1(q1_hold), 0.0f,
                     ARM_FAULT_HOLD_KP, ARM_FAULT_HOLD_KD, 0.0f);
        AK_Motor_MIT(&motors[1], joint_to_motor_2(q2_hold), 0.0f,
                     ARM_FAULT_HOLD_KP, ARM_FAULT_HOLD_KD, 0.0f);
        return;
    }

    /* 1. 输入目标位置（x,y -> x,z） */
    target.x = x;
    target.z = y;
    target_pos = target;

    /* 2. 反馈 -> 当前关节角（电机角->关节角，含零位/方向换算） */
    motor_angles.q1 = motor_to_joint_1(motors[0].pos_rad);   /* 大臂 */
    motor_angles.q2 = motor_to_joint_2(motors[1].pos_rad);   /* 小臂 */
    motor_angles.q3 = 0.0f;

    /* 3. 正解：当前关节角 -> 当前末端位置 */
    Kinematics_Forward(&motor_angles, &FK_pos);

    /* 4. 第三关节角：EL05 电机反馈角 = −关节角 → q3 = −pos_rad */
    q3_now = -el05_motors[0].pos_rad;

    /* 5. 逆解：目标位置 -> 期望关节角（解算失败则保持当前关节角） */
    q_des = motor_angles;
    (void)Kinematics_Inverse(&target, &q_des, -1);

    /* 6. 重力补偿（杠杆原理）：τ = Σ 质量 × g × 到质心的水平力臂 */
    get_gravity_comp_torque(motor_angles.q1, motor_angles.q2, q3_now,
                            &tau_sh, &tau_el, &tau_wr);
    b = tau_sh;
    c = tau_el;
    d = tau_wr;
    

    AK_Motor_MIT(&motors[0], joint_to_motor_1(q_des.q1), 0.0f, 15.0f, 1.5f, tau_sh);
    AK_Motor_MIT(&motors[1], joint_to_motor_2(q_des.q2), 0.0f, 15.0f, 1.2f, tau_el);
    /* 7. 下发前馈力矩（kp=kd=0 → 纯重力补偿测试）
     *   注意：重力补偿现为【输出侧口径】(未除减速比)，与 MIT 的 t 字段一致；
     *         若实测补偿过强/过弱，改 kinematics.c 里的 dir 或质量参数 */
    // AK_Motor_MIT(&motors[0], 0.0f, 0.0f, 0.0f, 0.0f, tau_sh);
    // AK_Motor_MIT(&motors[1], 0.0f, 0.0f, 0.0f, 0.0f, tau_el);
    EL05_MotionControl(&el05_motors[0], el05_motors[0].pos_rad, 0.0f, 0.0f, 0.0f, tau_wr);
}
