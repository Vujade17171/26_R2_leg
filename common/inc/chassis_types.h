#ifndef __CHASSIS_TYPES_H
#define __CHASSIS_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float yaw;
} ChassisPosition;

typedef struct {
    float speed[APP_VESC_NUM];  /* 轮速 m/s */
} ChassisVelocity;





#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_TYPES_H */