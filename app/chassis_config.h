#ifndef __CHASSIS_CONFIG_H
#define __CHASSIS_CONFIG_H

#define APP_VESC_NUM         4u        //电机数量

#define RC_BUF_SIZE   64u              //接收缓冲区大小（已不用：DMA 缓冲归 rc_receiver.c）

#define APP_RC_LOST_TIMEOUT_MS   500u  //遥控失联判定(ms)：超过这么久 frames 没增长就清零目标

#define BRAKE_CURRENT          5.0f       //刹车电流，单位 A

#define CHASSIS_R            0.292f     //底盘半径，单位 m
#define WHEEL_D              0.154f      //轮子直径，单位 m
#define PI                   3.141592f //圆周率
#define GEAR_RATIO           2.3f     //减速比

#endif /* __CHASSIS_CONFIG_H */