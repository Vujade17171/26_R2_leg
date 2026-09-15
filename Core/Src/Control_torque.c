#include "Control_torque.h"
#include "Control_pos.h"   /* 提供 L1、L2、motor1_to_joint、motor2_to_joint、clampf */
#include "math.h"

/* ================================================================================
 * 文件：Control_torque.c
 * 作用：二连杆（大臂+小臂）的重力补偿 + 力矩限幅。
 *
 * 原理：机械臂静止时，重力会在关节上产生"下垂力矩"，需要电机施加反向力矩才能
 *       保持姿态（力控/零力拖动时尤其重要）。本模块算出这个反向力矩作为前馈，
 *       直接加到 AK_Motion_Control() 的 torque 参数上，位置环只负责跟踪，
 *       重力由前馈兜底，从而减小位置环负担、消除稳态下垂。
 *
 * 坐标约定：
 *   q1 = 大臂相对水平(+x)的角度，q2 = 小臂相对大臂的角度（与你的 FK/IK 一致）。
 *   重力补偿力矩公式 = +m*g*水平距离，正值表示逆时针（托起）方向。
 *   最终输出的力矩是【电机输出轴坐标】，可直接喂给 AK_Motion_Control 的 torque。
 * ================================================================================
 */

/* ==================== 全局配置（默认值） ==================== */
/* 这些数值是参考估算值，务必按你的机械实测标定。
 * 减速比说明：你的 AK 电机 status.position/torque 已经是"输出轴"坐标（内部减速比
 * 已算进去）。若输出轴直接连关节，减速比填 1.0；若输出轴到关节之间还有额外减速
 * N:1，才填 N。 */
grav_comp_cfg_t g_grav_cfg = {
    .m_upper      = 0.30f,   /* 大臂连杆质量 (kg) */
    .lc_upper     = 0.175f,  /* 大臂质心 = L1/2 = 0.35/2 (m) */
    .m_elbow      = 0.50f,   /* 小臂+肘集中质量 (kg) */
    .lc_elbow     = 0.125f,   /* 小臂质心到肘关节的距离 = L2 (m) */
    .m_wrist      = 0.45f,   /* 腕部集中质量 (kg) */
    .payload_mass = 0.30f,   /* 负载质量 (kg) */
    .g            = 9.81f,   /* 重力加速度 (m/s^2) */

    .shoulder_gear = 1.25f,   /* 肩减速比 */
    .elbow_gear    = 0.35f,   /* 肘减速比 */
    .shoulder_dir  = 1.0f,   /* 肩补偿方向 */
    .elbow_dir     = 1.0f,   /* 肘补偿方向 */

    .max_torque_shoulder = 6.0f,  /* 肩前馈上限 (Nm) */
    .max_torque_elbow    = 5.5f,  /* 肘前馈上限 (Nm) */
    .torque_rate_limit   = 50.0f,  /* 力矩变化率限制 (Nm/s)*/
};

/* ==================== 内部辅助：力矩限幅 + 变化率限制 ==================== */
/*
 * 作用：先做绝对限幅，再限制"相对上次"的变化速度，防止力矩突变打手。
 * 入参：target=目标力矩, last=上一次输出的力矩, max_torque=上限, rate_limit=变化率上限, dt=周期。
 * 返回：限制后的力矩。
 */
static float limit_torque(float target, float last, float max_torque, float rate_limit, float dt)
{
    /* 第一步：绝对限幅到 [-max_torque, max_torque] */
    target = clampf(target, -max_torque, max_torque);

    /* 本周期最多允许变化多少（rate_limit 单位是 Nm/s） */
    float max_delta = rate_limit * dt;

    /* 与上次输出做差 */
    float diff = target - last;

    /* 变化超出上限，就钳到最大允许变化量 */
    if (diff > max_delta)       return last + max_delta;
    else if (diff < -max_delta) return last - max_delta;
    else                        return target;   /* 没超限，正常返回 */
}

/* ==================== 主函数：重力补偿 + 限幅 ==================== */
void Control_Torque_FeedForward(float* tau_sh, float* tau_el, int has_payload, float dt)
{
    /* ---- 1. 读当前实际关节角（用反馈，不用目标，因为重力取决于真实姿态） ---- */
    float q1 = motor1_to_joint(g_ak80.status.position);   /* 大臂（肩）关节角 */
    float q2 = motor2_to_joint(g_ak45.status.position);   /* 小臂（肘）关节角 */

    /* 预计算三角函数，避免重复计算 */
    float c1  = cosf(q1);            /* 大臂绝对角的余弦 */
    float c12 = cosf(q1 + q2);       /* 小臂绝对角的余弦 */

    /* ---- 2. 重力力矩（关节坐标，单位 Nm） ----
     * T1 = 小臂质量产生的肩力矩 + 腕部质量产生的肩力矩
     * T2 = 小臂质量 + 腕部质量产生的肘力矩
     * 公式依据：质量在关节处的重力力矩 = m * g * (质量到关节的水平距离) */
    float T1 = g_grav_cfg.m_elbow * g_grav_cfg.g * L1 * c1
             + g_grav_cfg.m_wrist * g_grav_cfg.g * (L1 * c1 + L2 * c12);
    float T2 =  g_grav_cfg.m_wrist * g_grav_cfg.g  * L2 * c12;

    /* ---- 3. 追加大臂连杆自身重力（质心在 lc_upper 处） ---- */
    T1 += g_grav_cfg.m_upper * g_grav_cfg.g * g_grav_cfg.lc_upper * c1;

    /* ---- 4. 追加负载重力（可选，has_payload=1 时） ---- */
    if (has_payload) {
        /* 负载近似作用在小臂末端，产生的肩、肘力矩分别叠加 */
        T1 += g_grav_cfg.payload_mass * g_grav_cfg.g * (L1 * c1 + L2 * c12);
        T2 += g_grav_cfg.payload_mass * g_grav_cfg.g * L2 * c12;
    }

    /* ---- 5. 关节力矩 → 电机输出轴力矩（除以减速比、乘以方向） ---- */
    float tg_sh = T1 * g_grav_cfg.shoulder_dir / g_grav_cfg.shoulder_gear;   /* 肩重力前馈 */
    float tg_el = T2 * g_grav_cfg.elbow_dir    / g_grav_cfg.elbow_gear;      /* 肘重力前馈 */

    /* ---- 6. 总前馈 = 重力前馈 ---- */
    float total_sh = tg_sh;
    float total_el = tg_el;

    /* ---- 7. 限幅 + 变化率限制（用 static 记住上一次输出） ---- */
    static float last_sh = 0.0f;   /* 肩上次输出力矩 */
    static float last_el = 0.0f;   /* 肘上次输出力矩 */

    total_sh = limit_torque(total_sh, last_sh, g_grav_cfg.max_torque_shoulder,
                            g_grav_cfg.torque_rate_limit, dt);
    total_el = limit_torque(total_el, last_el, g_grav_cfg.max_torque_elbow,
                            g_grav_cfg.torque_rate_limit, dt);

    /* 记住本次输出，供下次做变化率限制 */
    last_sh = total_sh;
    last_el = total_el;

    /* ---- 8. 输出结果（指针非空才写，方便调用方只取其中一个） ---- */
    if (tau_sh) *tau_sh = total_sh;
    if (tau_el) *tau_el = total_el;
}
