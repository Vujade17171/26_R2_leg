#include "arm_gravity.h"
#include "arm_kinematics.h"
#include <math.h>
#include <stddef.h>

/* 参考工程静态重力模型参数。 */
#define ARM_GRAVITY_G            9.81f
#define ARM_GRAVITY_M_LINK1      0.30f
#define ARM_GRAVITY_LC1          0.175f
#define ARM_GRAVITY_M_L1_END     0.50f
#define ARM_GRAVITY_M_L2_END     0.45f
#define ARM_GRAVITY_M_PAYLOAD    0.00f

/* 电机侧前馈限幅，避免参数误设时输出过大。 */
#define ARM_GRAVITY_MAX_SH_FF    3.50f
#define ARM_GRAVITY_MAX_EL_FF    1.20f
#define ARM_GRAVITY_RAMP_RATE    1.0f


#define ARM_WRIST_GRAVITY_DIR    1.0f
#define ARM_WRIST_GRAVITY_MASS   0.30f
#define ARM_WRIST_GRAVITY_LCOM   0.08f
#define ARM_WRIST_GRAVITY_LIMIT  1.0f
#define ARM_WRIST_GRAVITY_RATE   0.5f

/* Keil Watch 可调参数。 */
volatile float arm_gravity_scale = 1.0f;   /* 初次测试保守值，可在 Watch 中调整 */
volatile float arm_gravity_shoulder_dir = 1.0f;
volatile float arm_gravity_elbow_dir = 1.0f;
volatile float arm_gravity_gear_sh = 1.50f;
volatile float arm_gravity_gear_el = 1.30f;

/* Keil Watch 调试量。 */

volatile float arm_gravity_tau_shoulder_motor = 0.0f;
volatile float arm_gravity_tau_elbow_motor = 0.0f;
volatile float arm_wrist_gravity_scale = 0.0f;

static float s_tau_sh = 0.0f;
static float s_tau_el = 0.0f;
static float s_tau_wrist = 0.0f;

static float arm_gravity_clamp(float value, float limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

static float arm_gravity_move_toward(float current, float target, float step)
{
    float delta = target - current;

    if (delta > step) { return current + step; }
    if (delta < -step) { return current - step; }
    return target;
}

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

    c1 = cosf(q1);
    c12 = cosf(q1 + q2);
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
    if (gear_sh < 0.001f) { gear_sh = 7.5f; }
    if (gear_el < 0.001f) { gear_el = 7.5f; }

    tau_sh_motor = (tau_sh_joint / gear_sh) *
                   arm_gravity_scale * arm_gravity_shoulder_dir;
    tau_el_motor = (tau_el_joint / gear_el) *
                   arm_gravity_scale * arm_gravity_elbow_dir;

    tau_sh_motor = arm_gravity_clamp(tau_sh_motor, ARM_GRAVITY_MAX_SH_FF);
    tau_el_motor = arm_gravity_clamp(tau_el_motor, ARM_GRAVITY_MAX_EL_FF);

    if (dt <= 0.0f)  { dt = 0.003f; }
    if (dt >  0.05f) { dt = 0.003f; }
    ramp_step = ARM_GRAVITY_RAMP_RATE * dt;

    /* 缓慢爬升，避免修改参数后产生力矩阶跃。 */
    s_tau_sh = arm_gravity_move_toward(s_tau_sh, tau_sh_motor, ramp_step);
    s_tau_el = arm_gravity_move_toward(s_tau_el, tau_el_motor, ramp_step);

    arm_gravity_tau_shoulder_motor = s_tau_sh;
    arm_gravity_tau_elbow_motor = s_tau_el;

    if (tau_shoulder_motor != NULL) { *tau_shoulder_motor = s_tau_sh; }
    if (tau_elbow_motor != NULL) { *tau_elbow_motor = s_tau_el; }
}

float arm_wrist_gravity_get(float q0, float q1, float q2,
                            float l3_level_c, float dt)
{
    float theta_l3;
    float tau_raw;
    float tau_target;
    float ramp_step;

    if (dt <= 0.0f)  { dt = 0.003f; }
    if (dt >  0.05f) { dt = 0.003f; }

    theta_l3 = (q0 + q1 + q2) - l3_level_c;

    tau_raw = ARM_WRIST_GRAVITY_DIR *
              arm_wrist_gravity_scale *
              ARM_WRIST_GRAVITY_MASS *
              ARM_GRAVITY_G *
              ARM_WRIST_GRAVITY_LCOM *
              cosf(theta_l3);

    tau_target = arm_gravity_clamp(tau_raw, ARM_WRIST_GRAVITY_LIMIT);
    ramp_step = ARM_WRIST_GRAVITY_RATE * dt;

    s_tau_wrist = arm_gravity_move_toward(s_tau_wrist,
                                          tau_target,
                                          ramp_step);

    return s_tau_wrist;
}