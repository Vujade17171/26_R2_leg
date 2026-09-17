#ifndef __ARM_GRAVITY_H
#define __ARM_GRAVITY_H

/*
 * 肩关节/肘关节静态重力前馈。
 * 这些 volatile 变量都可在 Keil Watch 窗口直接修改。
 */
extern volatile float arm_gravity_scale;          /* 总补偿比例，1.0=按模型满补偿 */
extern volatile float arm_gravity_shoulder_dir;   /* 肩关节方向，只允许 +1/-1 */
extern volatile float arm_gravity_elbow_dir;      /* 肘关节方向，只允许 +1/-1 */

/* 参考工程外部减速比。先按 7.5 测试，可在 Watch 中修改。 */
extern volatile float arm_gravity_gear_sh;
extern volatile float arm_gravity_gear_el;

/* 调试变量：joint 为关节侧计算力矩，motor 为最终发送给 MIT 接口的力矩。 */

extern volatile float arm_gravity_tau_shoulder_motor;
extern volatile float arm_gravity_tau_elbow_motor;

/*
 * 输入当前肩关节 q1、肘关节 q2（rad）。
 * 输出最终发送给 AK80-9、AK45-10 的电机侧前馈力矩（Nm）。
 */
void arm_gravity_get(float q1, float q2,
                     float *tau_shoulder_motor,
                     float *tau_elbow_motor);

#endif /* __ARM_GRAVITY_H */