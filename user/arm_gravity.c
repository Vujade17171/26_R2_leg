/*
 * 机械臂静态重力前馈。
 * 根据肩、肘、腕关节角估算重力力矩，
 * 再将关节侧力矩换算到电机侧并交给 MIT 控制帧。
 */
#include "arm_gravity.h"
#include "arm_kinematics.h"
#include "arm_math.h"
#include <math.h>
#include <stddef.h>

/* 静态重力模型参数：质量 kg，长度 m，重力加速度 m/s^2。 */
#define ARM_GRAVITY_G            9.81f      /* 重力加速度 */
#define ARM_GRAVITY_M_LINK1      0.30f      /* 大臂自身质量 */
#define ARM_GRAVITY_LC1          0.175f     /* 大臂质心到肩关节距离 */
#define ARM_GRAVITY_M_L1_END     0.50f      /* 肩肘连杆末端等效质量 */
#define ARM_GRAVITY_M_L2_END     0.45f      /* 小臂末端等效质量 */
#define ARM_GRAVITY_M_PAYLOAD    0.00f      /* 末端负载质量 */

/* 电机侧前馈限幅，避免参数误设时输出过大。 */
#define ARM_GRAVITY_MAX_SH_FF    3.50f      /* 肩关节电机侧限幅 Nm */
#define ARM_GRAVITY_MAX_EL_FF    1.20f      /* 肘关节电机侧限幅 Nm */
#define ARM_GRAVITY_RAMP_RATE    30.0f      /* 肩、肘前馈爬升速率 Nm/s */

/* 腕关节静态重力模型参数。 */
#define ARM_WRIST_GRAVITY_DIR    1.0f       /* 腕关节补偿方向 */
#define ARM_WRIST_GRAVITY_MASS   0.30f      /* 腕部等效质量 kg */
#define ARM_WRIST_GRAVITY_LCOM   0.08f      /* 等效质心距离 m */
#define ARM_WRIST_GRAVITY_LIMIT  1.0f       /* (重力前馈限幅)腕关节电机侧限幅 Nm */
#define ARM_WRIST_GRAVITY_RATE   0.6f       /* 腕关节前馈爬升速率 Nm/s */

/* Keil Watch 可调参数。 */
volatile float arm_gravity_scale = 1.0f;   /* 初次测试保守值，可在 Watch 中调整 */
volatile float arm_gravity_shoulder_dir = 1.0f;
volatile float arm_gravity_elbow_dir = 1.0f;
volatile float arm_gravity_gear_sh = 1.50f;  /* 肩关节外部减速比 */
volatile float arm_gravity_gear_el = 1.30f;  /* 肘关节外部减速比 */
volatile float arm_wrist_gravity_scale = 1.0f;

/* 当前经过斜坡平滑后的电机侧前馈力矩。 */
static float s_tau_sh = 0.0f;
static float s_tau_el = 0.0f;
static float s_tau_wrist = 0.0f;


/*
 * 计算肩、肘关节静态重力前馈。
 * q1、q2：肩、肘关节角，单位 rad；
 * dt：控制周期，单位 s；
 * tau_shoulder_motor、tau_elbow_motor：输出电机侧前馈力矩 Nm，可为 NULL。
 */
void arm_gravity_get(float q1, float q2, float dt,
                     float *tau_shoulder_motor,
                     float *tau_elbow_motor)
{
    float c1;
    float c12;
    float m_l2;
    float gear_sh;
    float gear_el;
    float tau_sh_joint;
    float tau_el_joint;
    float tau_sh_motor;
    float tau_el_motor;
    float ramp_step;

    /* 计算模型中的两个重力力臂余弦项。 */
    c1 = cosf(q1);
    c12 = cosf(q1 + q2);

    /* 小臂末端等效质量与当前负载质量之和。 */
    m_l2 = ARM_GRAVITY_M_L2_END + ARM_GRAVITY_M_PAYLOAD;

    /*
     * 肩关节：
     *   大臂自身重力 + 大臂末端等效质量 + 小臂末端等效质量。
     * 肘关节：
     *   小臂末端等效质量产生的重力力矩。
     */
    tau_sh_joint = ARM_GRAVITY_G *
                   ((ARM_GRAVITY_M_LINK1 * ARM_GRAVITY_LC1 +
                     ARM_GRAVITY_M_L1_END * ARM_L1 +
                     m_l2 * ARM_L1) * c1 +
                    m_l2 * ARM_L2 * c12);

    tau_el_joint = ARM_GRAVITY_G * m_l2 * ARM_L2 * c12;



    /* 关节侧力矩除以外部减速比，得到电机侧前馈力矩。 */
    gear_sh = fabsf(arm_gravity_gear_sh);
    gear_el = fabsf(arm_gravity_gear_el);

    /* 减速比无效时使用默认值，避免除零。 */
    if (gear_sh < 0.001f) { gear_sh = 7.5f; }
    if (gear_el < 0.001f) { gear_el = 7.5f; }

    /* 应用总补偿比例和各关节方向。 */
    tau_sh_motor = (tau_sh_joint / gear_sh) *
                   arm_gravity_scale * arm_gravity_shoulder_dir;
    tau_el_motor = (tau_el_joint / gear_el) *
                   arm_gravity_scale * arm_gravity_elbow_dir;

    /* 限制最终输出，防止模型参数错误产生过大力矩。 */
    tau_sh_motor = arm_clampf(tau_sh_motor,
                              -ARM_GRAVITY_MAX_SH_FF,
                              ARM_GRAVITY_MAX_SH_FF);
    tau_el_motor = arm_clampf(tau_el_motor,
                              -ARM_GRAVITY_MAX_EL_FF,
                              ARM_GRAVITY_MAX_EL_FF);

    /* dt 异常时回退到默认 2 ms，保证斜坡步长有效。 */
    if (dt <= 0.0f)  { dt = 0.002f; }
    if (dt >  0.05f) { dt = 0.002f; }
    ramp_step = ARM_GRAVITY_RAMP_RATE * dt;

    /* 缓慢爬升，避免修改参数后产生力矩阶跃。 */
    s_tau_sh = arm_move_toward(s_tau_sh, tau_sh_motor, ramp_step);
    s_tau_el = arm_move_toward(s_tau_el, tau_el_motor, ramp_step);

    /* 输出给电机控制帧。 */
    if (tau_shoulder_motor != NULL) { *tau_shoulder_motor = s_tau_sh; }
    if (tau_elbow_motor != NULL) { *tau_elbow_motor = s_tau_el; }
}

/*
 * 计算腕关节静态重力前馈。
 * q0、q1、q2：腕、肩、肘关节角，单位 rad；
 * l3_level_c：L3 水平机构的零位角，单位 rad；
 * dt：控制周期，单位 s；返回电机侧前馈力矩 Nm。
 */
float arm_wrist_gravity_get(float q0, float q1, float q2,
                            float l3_level_c, float dt)
{
    float theta_l3;
    float tau_raw;
    float tau_target;
    float ramp_step;

    /* dt 异常时回退到默认 2 ms，保证斜坡步长有效。 */
    if (dt <= 0.0f)  { dt = 0.002f; }
    if (dt >  0.05f) { dt = 0.002f; }

    /* L3 水平机构相对水平位置的偏移角。 */
    theta_l3 = (q0 + q1 + q2) - l3_level_c;

    /* 简化的腕部重力矩：质量 × 重力 × 等效质心长度 × 余弦项。 */
    tau_raw = ARM_WRIST_GRAVITY_DIR *
              arm_wrist_gravity_scale *
              ARM_WRIST_GRAVITY_MASS *
              ARM_GRAVITY_G *
              ARM_WRIST_GRAVITY_LCOM *
              cosf(theta_l3);

    /* 先限幅，再按固定速率平滑到目标前馈力矩。 */
    tau_target = arm_clampf(tau_raw,
                            -ARM_WRIST_GRAVITY_LIMIT,
                            ARM_WRIST_GRAVITY_LIMIT);
    ramp_step = ARM_WRIST_GRAVITY_RATE * dt;

    s_tau_wrist = arm_move_toward(s_tau_wrist,
                                          tau_target,
                                          ramp_step);

    return s_tau_wrist;
}
