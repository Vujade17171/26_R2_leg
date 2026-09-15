#ifndef __MOTO_TASK_H
#define __MOTO_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "Control_pos.h"   /* 正逆运动学、五次多项式轨迹、leg_motion 等 */

/* ==================== 电机 CAN ID ==================== */
/* AK80 是大臂（ID=0x01，已在 Joint_AK80.h 定义）。
 * AK45 是小臂，ID 需按硬件拨码确认，这里暂定 0x02。 */
#define AK45_ID 0x02

/* ==================== 控制模式 ==================== */
typedef enum {
    MODE_ZERO     = 0,   /* 模式1：全归零，用于判断正解算与关节零点偏移 */
    MODE_POSITION = 1,   /* 模式2：末端位置控制（逆解算 + 五次多项式） */
    MODE_TORQUE   = 2,   /* 模式3：纯力矩重力补偿（零力拖动测试） */
} control_mode_t;

/* ==================== 末端位置控制指令（debug 中填写） ==================== */
typedef struct {
    float target_x_s;    /* 目标末端落点 x（m） */
    float target_z_s;    /* 目标末端落点 z（m） */
    float target_yaw;    /* 目标末端姿态角（rad），填 0 = 保持当前姿态 */
    float duration;      /* 轨迹时长（s），越大越慢越平滑 */
} pos_cmd_t;

/* ==================== 全局控制变量（debug 中修改） ==================== */
extern volatile control_mode_t g_ctrl_mode;    /* 当前模式：0=归零，1=位置控制，2=纯力矩重力补偿 */
extern volatile int            g_cmd_trigger;  /* 置 1 = 触发一次新的位置规划 */
extern pos_cmd_t               g_pos_cmd;      /* 位置控制指令 */

/* ==================== 电机控制参数（debug 中可调） ==================== */
extern float g_kp;       /* 位置环 Kp（AK 范围 0~500），越大刚度越高 */
extern float g_kd;       /* 位置环 Kd（AK 范围 0~5），阻尼，抑制震荡 */
extern float g_torque;   /* 前馈力矩（Nm），默认 0 = 纯位置控制 */

/* ==================== 任务函数 ==================== */
extern void Moto_Diver(void *argument);

#endif
