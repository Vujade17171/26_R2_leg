/**
  ******************************************************************************
  * @file    arm_traj.c
  * @brief   Quintic polynomial point-to-point joint trajectory
  ******************************************************************************
  * Formula (s = normalized 0..1):
  *   s     = 10t^3 - 15t^4 + 6t^5
  *   s'    = (1/T) t^2 (30 - 60t + 30t^2)
  *   s''   = (1/T^2) t (60 - 180t + 120t^2)
  */
#include "arm_traj.h"

static float s_qs[3]   = {0, 0, 0};  //起点关节角
static float s_qe[3]   = {0, 0, 0};  //终点关节角
static float s_dur     = 1.0f;       //轨迹持续时间
static float s_elap    = 0.0f;       //已过时间
static int   s_active  = 0;          //轨迹是否在跑


//初始化函数
void arm_traj_start(float q0_s, float q1_s, float q2_s,
                    float q0_e, float q1_e, float q2_e,
                    float T_sec)
{
    s_qs[0] = q0_s; s_qs[1] = q1_s; s_qs[2] = q2_s;
    s_qe[0] = q0_e; s_qe[1] = q1_e; s_qe[2] = q2_e;
    s_dur   = (T_sec > 0.0f) ? T_sec : 0.01f;
    s_elap  = 0.0f;
    s_active = 1;
}


//目标角度，目标角速度
int arm_traj_update(float dt,
                    float *q0, float *q1, float *q2,
                    float *v0, float *v1, float *v2)
{
    float tau, tau2, s, sdot, sddot, inv_T, inv_T2;
    float d0, d1, d2;

    if (!s_active) { return 1; }

    s_elap += dt;
    tau = s_elap / s_dur;
    if (tau >= 1.0f) { tau = 1.0f; s_active = 0; }

    tau2 = tau * tau;
    inv_T  = 1.0f / s_dur;
    inv_T2 = inv_T * inv_T;

    /* quintic shape */
    s    = tau * tau2 * (10.0f + tau * (-15.0f + 6.0f * tau));
    sdot = tau2 * inv_T  * (30.0f + tau * (-60.0f + 30.0f * tau));
    sddot= tau  * inv_T2 * (60.0f + tau * (-180.0f + 120.0f * tau));

    d0 = s_qe[0] - s_qs[0];    //关节1角度差
    d1 = s_qe[1] - s_qs[1];    //关节2角度差
    d2 = s_qe[2] - s_qs[2];    //关节3角度差

    if (q0) *q0 = s_qs[0] + d0 * s;
    if (q1) *q1 = s_qs[1] + d1 * s;
    if (q2) *q2 = s_qs[2] + d2 * s;

    if (v0) *v0 = d0 * sdot;
    if (v1) *v1 = d1 * sdot;
    if (v2) *v2 = d2 * sdot;
    return s_active ? 0 : 1;
}

void arm_traj_stop(void) { s_active = 0; }
int  arm_traj_is_active(void) { return s_active; }
