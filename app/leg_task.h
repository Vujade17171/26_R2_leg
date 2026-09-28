/*****************************************************************************
 * leg_task.h —— 应用层任务接口
 *
 * 分层位置：app
 * 说明：
 *  - CubeMX 已在 freertos.c 中生成 __weak void leg_task(void *argument)，
 *    本模块提供强定义版本，编译时自动覆盖弱函数，无需改动 freertos.c；
 *  - CubeMX 重新生成工程不影响本模块；
 *  - 本头文件【不包含 cmsis_os2.h】—— OS 类型只在 osal 层出现，
 *    这样换 RTOS 时 app 的头文件不需要改。
 *****************************************************************************/
#ifndef __LEG_TASK_H
#define __LEG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS 任务入口函数（强符号，覆盖 freertos.c 中的弱函数） */
void leg_task(void *argument);

/* ===================== 对外业务接口 ===================== */

/* 设置笛卡尔目标点 x,z：只登记目标，不规划、不下发。
 * 实际规划由主循环按"目标变化"触发，故可被上位机/调试随时调用。
 * 轨迹运行中重复调用只会锁存最新目标，等本段跑完再执行。 */
void arm_control(float x, float y);

/* 带时长的目标入口：duration_s <= 0 时用默认规划时长 */
void leg_move_to(float x, float z, float duration_s);

#ifdef __cplusplus
}
#endif

#endif /* __LEG_TASK_H */