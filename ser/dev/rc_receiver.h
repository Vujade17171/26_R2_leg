/*****************************************************************************
 * rc_receiver.h —— 遥控接收机服务（ser/dev）
 *
 * 分层位置：ser/dev —— 把 bsp_uart 的原始字节流解成"遥控器数据"。
 *   允许依赖：bsp（bsp_uart）、common（leg_types.h）
 *   禁止依赖：app、FreeRTOS
 *
 * ================== 面向 app 的接口只有 4 个，低耦合 ==================
 *   ① RC_Data              遥控器数据结构体 —— 类型在 ser 定义，【实例由 app 定义】
 *   ② RC_RegisterHandle()  注册句柄：把 app 的结构体交给解包模块（依赖注入）
 *   ③ RC_Init()            初始化：启动串口 DMA 接收（内部走 bsp_uart）
 *   ④ RC_Get()             获取遥控器数据指针
 *
 * 与"电机解耦"完全同构：
 *     电机：app 定义 AK_Motor motors[]  ->  CAN_Router_BindAk(motors, n)  注入
 *     遥控：app 定义 RC_Data  rc        ->  RC_RegisterHandle(&rc)        注入
 *   app 读数据的姿势也一样：直接读自己那个实例的字段（或 RC_Get() 取指针）。
 *
 * 本头文件【不包含 bsp_uart.h / 任何 HAL 头】：
 *   串口、DMA、双缓冲、"帧结束"中断回调全部藏在 rc_receiver.c 里，
 *   app 只看得到这一个结构体和 3 个函数。换串口、改缓冲大小都不用动 app。
 *****************************************************************************/
#ifndef __RC_RECEIVER_H
#define __RC_RECEIVER_H

#include <stdint.h>
#include "leg_types.h"   /* SER_OK / SER_ERR_* */

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== ① 独属于 app 层的遥控器结构体 ====================
 * 类型在这里定义（ser），实例由 app 定义 —— 和 AK_Motor / VESC_Motor 一样。
 * ch[] 已减掉中值 1500，静止时约为 0。 */
typedef struct {
    int16_t  ch[3];     /* 摇杆/通道：ch[0]=x  ch[1]=y  ch[2]=第三通道（已减中值）*/
    int16_t  num;       /* 拨轮/旋钮原始值（未减中值）                            */
    char     s[4];      /* 4 个拨杆开关状态                                       */
    uint32_t frames;    /* 累计有效帧数（app 可用它判在线/离线）                   */
} RC_Data;

/* ==================== ② 注册句柄（依赖注入）====================
 * 对应 CAN_Router_BindAk()：把 app 自己的 RC_Data 交给解包模块，
 * 之后每帧解析结果直接写进这个实例。
 * 调用时会先清零该实例的字段。
 * 【任务上下文调用】，必须在 RC_Init() 之前调用，否则会丢最开始几帧。
 * 返回 SER_OK / SER_ERR_PARAM */
int32_t RC_RegisterHandle(RC_Data *rc);

/* ==================== ③ 初始化 ====================
 * 启动串口 DMA 接收。内部调用 bsp_uart —— app 不需要知道串口/DMA 的存在。
 * 返回 SER_OK / SER_ERR_STATE（还没注册句柄）/ 其它 bsp 错误码
 * 【任务上下文调用】。 */
int32_t RC_Init(void);

/* ==================== ④ 获取遥控器数据 ====================
 * 返回 RC_RegisterHandle() 注册进来的指针；未注册时返回 0。
 * 也可以直接用 app 自己那个实例的地址，二者完全等价（保留本函数是为了
 * 让不持有指针的模块也能取到数据）。 */
const RC_Data *RC_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* __RC_RECEIVER_H */
