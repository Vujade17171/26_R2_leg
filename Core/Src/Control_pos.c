#include "Control_pos.h"

/* ==================== 全局对象 ==================== */
/* pos        ：正运动学算出来的"当前实际位置"，每次 Forward_Kinematics 刷新 */
/* leg_motion ：逆运动学算出来的"目标角度"，供电机控制任务读取下发 */
leg_pos_t   pos        = {0};
LegMotion_t leg_motion = {0};

/* jtraj      ：五次多项式轨迹规划句柄，jtraj_start 启动、jtraj_update 推进 */
JTraj_t     jtraj      = {0};

/* ==================== 内部静态工具 ==================== */

/*
 * 自己写的一个开平方函数（牛顿迭代法）。
 * 为什么不直接用 sqrtf？—— 因为 STM32 上有时没开 FPU 的 arm_math，
 * 这里用纯循环实现，保证一定能编译、能跑。
 *
 * 原理（牛顿法求 √x）：
 *   假设 r 是 √x 的近似值，那么更精确的值 = (r + x/r) / 2
 *   迭代 8 次，精度足够我们这种 0.35m 的机械臂用。
 */
static float my_sqrtf(float x)
{
    if (x <= 0.0f) return 0.0f;   /* 负数/零直接返回 0，避免除零 */

    float r = x;                  /* 初始猜测就取 x 本身 */
    for (int i = 0; i < 8; ++i) {
        r = 0.5f * (r + x / r);   /* 每次更接近真实平方根 */
    }
    return r;
}

/*
 * 计算两个角度的"最短差值"，结果始终在 (-π, π] 之间。
 * 例：a = 170°，b = -170°，直接相减是 340°，但实际最短只差 20°。
 * 这个函数就是用来把角度差"归一化"到最短的那一圈。
 */
static float ang_diff(float a, float b)
{
    float d = a - b;                          /* 先直接相减 */
    while (d >  3.14159265f) d -= 6.28318530f; /* 太大就减 2π */
    while (d < -3.14159265f) d += 6.28318530f; /* 太小就加 2π */
    return d;
}

/*
 * 把关节角钳到"软限位"以内，防止机械结构撞到极限。
 * 软限位在 Control_pos.h 里定义：Q1 ±Π/4°，Q2 ±0.9Π°。
 */
static void clamp_to_joint_limit(float *q1, float *q2)
{
    if (q1) *q1 = clampf(*q1, Q1_MIN, Q1_MAX);   /* clampf 见 .h 文件 */
    if (q2) *q2 = clampf(*q2, Q2_MIN, Q2_MAX);
}

/* ==================== 正运动学解算 ==================== */
/*
 * 正运动学：已知各个电机的角度，算出机械臂末端在空间中的位置。
 *
 * 电机与关节的对应关系：
 *   g_ak80.status.position  = 大臂角 q1（相对基座，是绝对角）
 *   g_ak45.status.position  = 小臂角 q2（相对大臂，是相对角）
 *   g_el05.status.position  = 腕部角 q3（相对小臂，是相对角）
 *
 * 所以：
 *   q1      = 大臂与地面的夹角
 *   q12     = q1 + q2 = 小臂与地面的绝对夹角
 *   abs_angle = q12 + q3 = 末端姿态的绝对角
 *
 * 几何关系（画图更直观）：
 *   L1 是大臂，一头在原点，方向角 q1
 *   L2 是小臂，一头接在大臂末端，方向角 q12
 *   L3 是腕部，一头接在小臂末端，方向角 abs_angle
 *
 *   —— 这些 L1/L2/L3 是长度常量，在 Control_pos.h 里定义。
 */
void Forward_Kinematics(leg_pos_t *my_pos)
{
    /* 1. 从电机反馈里取当前关节角（必须经方向系数和零偏换算，
     *    否则零偏会被当成关节角，导致正解算位置全是错的） */
    float q1  = motor1_to_joint(g_ak80.status.position);       /* 大臂关节角（已减零偏、方向换算） */
    float q12 = q1 + motor2_to_joint(g_ak45.status.position); /* 小臂关节角（已减零偏、方向换算） */

    /* 2. 算"腕部"坐标（大臂末端 + 小臂末端），也就是没有 L3 那一段的位置 */
    /*    x 分量 = 大臂水平投影 + 小臂水平投影 */
    pos.x = L1 * cosf(q1) + L2 * cosf(q12);
    /*    因为是 XZ 平面机械臂，y 永远是 0 */
    pos.y = 0.0f;
    /*    z 分量 = 大臂垂直投影 + 小臂垂直投影 */
    pos.z = L1 * sinf(q1) + L2 * sinf(q12);

    /* 3. 算"末端落点"（再往外延伸 L3 那一段） */
    /* EL05 反装（EL05_DIR=-1）且零位偏移 EL05_OFFSET=0.07，末端绝对角需按此换算 */
    float abs_angle = q12 + EL05_DIR * (g_el05.status.position - EL05_OFFSET);/* 末端绝对姿态角 */

    /*    末端 = 腕部 + L3 沿 abs_angle 方向的偏移 */
    pos.x_s = pos.x + L3 * cosf(abs_angle);
    pos.z_s = pos.z + L3 * sinf(abs_angle);

    pos.yaw = 0.0f;

    /* 顺手把结果同步回调用方传进来的指针（可选参数） */
    if (my_pos) {
        my_pos->x    = pos.x;
        my_pos->y    = pos.y;
        my_pos->z    = pos.z;
        my_pos->x_s  = pos.x_s;
        my_pos->z_s  = pos.z_s;
        my_pos->yaw  = pos.yaw;
    }
}

/* ==================== 逆运动学主函数 ==================== */
/*
 * 逆运动学：已知想要的腕部位置 (target_x, target_z)，反推出电机该转多少度。
 *
 * 这是整个文件的核心，下面一步步讲：
 *
 * 【第 1 步：拿到目标点】
 *   把目标填进 leg_pos->target_x / target_z，
 *   这里 target 是"腕部位置"，不是末端落点。
 *
 * 【第 2 步：可达性检查】
 *   两根连杆 L1、L2 首尾相连，能伸到的距离范围是：
 *      最短 = |L1 - L2|   （两臂完全对折）
 *      最长 =  L1 + L2    （两臂完全伸直）
 *   所以目标点到原点的距离 D = √(x?+z?) 必须满足：
 *      |L1 - L2| ≤ D ≤ L1 + L2
 *   超出这个范围就"够不着"，直接返回 -1。
 *
 * 【第 3 步：算小臂角 q2】
 *   余弦定理：已知三角形两边 L1、L2 和第三边 D，求夹角。
 *       D? = L1? + L2? + 2·L1·L2·cos(q2')
 *   注意这里 q2' 是"两臂之间的夹角"，跟我们定义的关节角 q2 有关。
 *   化简得到：
 *       cos_q2 = (D? - L1? - L2?) / (2·L1·L2)
 *   一个 cos 值对应两个 sin 符号（+ 和 -），
 *   这就是"肘部向上"和"肘部向下"两个解的来源。
 *
 * 【第 4 步：算大臂角 q1】
 *   phi = atan2(z, x)          —— 目标点相对原点的方向角
 *   psi = atan2(L2·sin_q2, L1 + L2·cos_q2) —— 小臂相对大臂的偏角
 *   q1  = phi - psi            —— 大臂角 = 目标方向 - 肘部偏角
 *
 * 【第 5 步：多解选择】
 *   q2 有两个解（肘部上/下），每个 q2 对应一个 q1，
 *   用"哪个离当前电机角度更近 + 哪个在限位内"来决定选哪个。
 *
 * 返回值：0 成功，-1 失败。
 */
int leg_inverse(const leg_pos_t *leg_pos, int elbow_up)
{
    /* 参数合法性检查：指针为空就返回失败 */
    if (!leg_pos) {
        leg_motion.success = -1;
        return -1;
    }

    /* ---------- 第 1 步：取出目标腕部位置 ---------- */
    float x = leg_pos->target_x;
    float z = leg_pos->target_z;

    /* 机械臂在 XZ 平面工作，所以水平方向直接用 x（带符号）。
     * 注意：这里不取绝对值！x 可以是负数，表示伸到身体的另一侧。*/
    float r_eff = x;
    float z_eff = z;

    /* ---------- 第 2 步：可达性检查 ---------- */
    /* D? 就是目标点到原点的距离平方（不用开方，省计算，也避免精度损失） */
    float D_sq  = r_eff * r_eff + z_eff * z_eff;

    /* Lsum  = 两臂最长 = L1+L2 */
    /* Ldiff = 两臂最短 = |L1-L2| */
    float Lsum  = L1 + L2;
    float Ldiff = fabsf(L1 - L2);

    /* EPS 是一个很小的容差，用来吸收浮点误差 */
    const float EPS = 1e-6f;

    /* 如果距离平方比"最长?"还大，或比"最短?"还小，就是够不着 */
    if (D_sq > Lsum * Lsum + EPS || D_sq < Ldiff * Ldiff - EPS) {
        leg_motion.success = -1;
        leg_motion.nijie_cost = 1e10f;
        return -1;   /* 目标不可达 */
    }

    /* ---------- 第 3 步：用余弦定理求小臂角 q2 ---------- */
    /* 由余弦定理反解 cos(q2) */
    float cos_q2 = (D_sq - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

    /* 浮点误差可能导致 cos 略超出 [-1,1]，钳一下 */
    cos_q2 = clampf(cos_q2, -1.0f, 1.0f);

    /* sin? = 1 - cos?，先保证非负，再开方得到 |sin_q2| */
    float s2_sq = 1.0f - cos_q2 * cos_q2;
    if (s2_sq < 0.0f) s2_sq = 0.0f;
    float sin_q2_abs = my_sqrtf(s2_sq);

    /* ---------- 第 4 步：确定 sin(q2) 的符号（多解） ---------- */
    /* 一个 cos 值对应两个 sin 符号：
     *   +sin_q2  → 肘部向上（elbow up）
     *   -sin_q2  → 肘部向下（elbow down）
     * 这里根据用户传入的 elbow_up 参数，决定要试哪些分支。*/
    float sin_candidates[2];

    if (elbow_up == ELBOW_DOWN) {
        /* 强制肘部向下：两个候选都填 -sin */
        sin_candidates[0] = -sin_q2_abs;
        sin_candidates[1] = -sin_q2_abs;
    } else if (elbow_up == ELBOW_UP) {
        /* 强制肘部向上：两个候选都填 +sin */
        sin_candidates[0] = +sin_q2_abs;
        sin_candidates[1] = +sin_q2_abs;
    } else {
        /* 自动模式：两个分支都试，后面用代价函数选最优 */
        sin_candidates[0] = +sin_q2_abs;
        sin_candidates[1] = -sin_q2_abs;
    }

    /* ---------- 第 5 步：枚举分支，选最优解 ---------- */
    /* 记录当前关节角，用来衡量"哪个解离当前位姿最近" */
    float q1_cur = motor1_to_joint(g_ak80.status.position);   /* 已减零偏、方向换算 */
    float q2_cur = motor2_to_joint(g_ak45.status.position);   /* AK45 已减零偏、方向换算 */

    /* best_* 存最终选中的解，best_cost 存最小代价 */
    float best_q1   = 0.0f, best_q2 = 0.0f;
    float best_cost = 1e10f;   /* 初始设一个很大的数 */
    int   found     = 0;       /* 是否找到可行解 */

    for (int k = 0; k < 2; ++k) {
        /* 取出当前分支的 sin 值 */
        float s2 = sin_candidates[k];

        /* q2 = atan2(sin, cos)，正好还原出带正确符号的角度 */
        float q2 = atan2f(s2, cos_q2);

        /* --- 求对应的大臂角 q1 --- */
        /* phi = 目标点相对原点的方向角（atan2 自动处理 x 的符号/象限） */
        float phi = atan2f(z_eff, r_eff);

        /* psi = 小臂相对大臂的偏角，用三角形几何算出来 */
        float psi = atan2f(L2 * s2, L1 + L2 * cos_q2);

        /* q1 = 目标方向角 - 肘部偏角 */
        float q1 = phi - psi;

        /* 把角度归一化到 (-π, π]，避免出现 3π 这种转了好几圈的值 */
        q1 = wrap_pi(q1);
        q2 = wrap_pi(q2);

        /* --- 检查这个解是否在关节限位内 --- */
        int  ok1     = in_limit(q1, 1);   /* 大臂角是否合法 */
        int  ok2     = in_limit(q2, 2);   /* 小臂角是否合法 */
        float penalty = (ok1 && ok2) ? 0.0f : 10000.0f;  /* 不合法就加巨大惩罚 */

        /* --- 计算代价：越小越好 --- */
        /* 1. 离当前角度的"距离"（变化越小越好，运动更平滑） */
        float dist_q1 = fabsf(ang_diff(q1, q1_cur));
        float dist_q2 = fabsf(ang_diff(q2, q2_cur));
        /* 2. 代价 = 限位惩罚 + 大臂变化量 + 0.3×小臂变化量
         *    （0.3 是小臂的权重，表示更倾向动小臂、少动大臂） */
        float cost = penalty + dist_q1 + 0.3f * dist_q2;

        /* 比当前最优更好就更新 */
        if (cost < best_cost) {
            best_cost = cost;
            best_q1   = q1;
            best_q2   = q2;
            found     = 1;
        }
    }

    /* 记录这次解的代价（供调试用） */
    leg_motion.nijie_cost = best_cost;

    /* 如果没找到解，或者最优解还是带惩罚（说明所有解都超限位），失败 */
    if (!found || best_cost >= 9999.0f) {
        leg_motion.success = -1;
        return -1;
    }

    /* ---------- 第 6 步：最终安全处理并写入结果 ---------- */
    /* 最后再钳一次限位 + 归一化角度，双保险 */
    clamp_to_joint_limit(&best_q1, &best_q2);
    best_q1 = wrap_pi(best_q1);
    best_q2 = wrap_pi(best_q2);

    /* 写入 leg_motion 供电机控制任务读取 */
    leg_motion.q0_target = 0.0f;                       /* 预留，本机械臂未用 */
    leg_motion.q1_target = best_q1;                    /* 大臂目标角 */
    leg_motion.q2_target = best_q2;                   /* 小臂目标角 */
    leg_motion.q3_target = g_el05.status.position;     /* 腕部角暂保持当前值 */

    /* 关节角 → 电机角（当前是 1:1，机械零位有偏差时可在这里加偏移） */
    leg_motion.motor1_target_angle = joint_to_motor_1(best_q1);
    leg_motion.motor2_target_angle = joint_to_motor_2(best_q2);

    /* EL05 腕部电机暂保持当前姿态，需要改姿态时外部单独设置 */
    leg_motion.motor3_target_angle = g_el05.status.position;

    /* 末端绝对姿态 = 小臂绝对角 + EL05（含反装与零位换算） */
    leg_motion.yaw_target_angle = best_q1 + best_q2 + EL05_DIR * (g_el05.status.position - EL05_OFFSET);

    leg_motion.success = 0;
    return 0;
}

/* ==================== 公共封装（腕部目标模式） ==================== */
/*
 * 这是给外部调用的"腕部目标"版逆运动学。
 * 你只需要填好 my_pos->target_x 和 target_z，它就帮你解算。
 */
int Inverse_Kinematics(leg_pos_t *my_pos, int elbow_up)
{
    if (!my_pos) {
        leg_motion.success = -1;
        return -1;
    }

    /* 调用核心解算函数 */
    int ret = leg_inverse(my_pos, elbow_up);

    /* 成功就累加一次成功计数 */
    if (ret == 0) leg_motion.success++;

    return ret;
}

/* ==================== 公共封装（末端目标模式） ==================== */
/*
 * 这是给外部调用的"末端落点"版逆运动学。
 * 区别：末端落点 (x_s, z_s) 比腕部位置 (target_x, target_z) 多了一截 L3。
 * 所以这里先根据末端姿态 yaw，把末端目标"退回"到腕部目标，再解算。
 *
 * 用法：
 *   my_pos->x_s  = 末端落点 x
 *   my_pos->z_s  = 末端落点 z
 *   my_pos->yaw  = 末端姿态角（填 0 表示沿用当前姿态）
 */
int Inverse_Kinematics_EE(leg_pos_t *my_pos, int elbow_up)
{
    if (!my_pos) {
        leg_motion.success = -1;
        return -1;
    }

    float abs_angle;
    if (my_pos->yaw != 0.0f) {
        /* 用户指定了末端姿态 */
        abs_angle = my_pos->yaw;
    } else {
        /* 没指定就用当前末端姿态，保证动作连续 */
        abs_angle = motor1_to_joint(g_ak80.status.position)    /* 已减零偏、方向换算 */
                  + motor2_to_joint(g_ak45.status.position)   /* AK45 已减零偏、方向换算 */
                  + EL05_DIR * (g_el05.status.position - EL05_OFFSET);  /* EL05 含反装与零位 */
    }

    /* 末端目标 → 腕部目标：往回减掉 L3 那一段 */
    my_pos->target_x = my_pos->x_s - L3 * cosf(abs_angle);
    my_pos->target_z = my_pos->z_s - L3 * sinf(abs_angle);

    /* 再用腕部目标解算 */
    int ret = leg_inverse(my_pos, elbow_up);
    if (ret == 0) leg_motion.success++;

    return ret;
}

/* ============================================================ */
/* ==================== 五次多项式轨迹规划 ==================== */
/* ============================================================ */
/*
 * 为什么要用轨迹规划？
 *   逆运动学算出来的 q1/q2 是一个"阶跃"目标：电机如果直接跳过去，
 *   会突然加减速、抖动、冲击。轨迹规划的作用就是在这两点之间，
 *   铺一条平滑的曲线，让位置、速度、加速度都连续过渡。
 *
 * 五次多项式的原理：
 *   定义归一化时间 tau = elapsed / duration，范围 [0,1]。
 *   位置曲线 s(tau) = 10·tau? - 15·tau? + 6·tau?
 *
 *   这条曲线有一个非常好的性质：
 *     s(0)=0,  s(1)=1            → 起点在 0，终点在 1
 *     s'(0)=0, s'(1)=0           → 起点、终点速度都为 0（静止起步、静止停）
 *     s''(0)=0, s''(1)=0         → 起点、终点加速度都为 0（无冲击）
 *   所以用 s(tau) 去插值起点和终点，就能得到"丝滑"的运动。
 */

/*
 * 启动轨迹规划：记录起点、终点、时长，并把计时归零。
 * 起点直接取当前电机的实际角度，保证从当前位姿无缝衔接。
 */
void jtraj_start(float q1_end, float q2_end, float duration)
{
    /* 起点：取当前关节角（必须经方向系数和零偏换算，否则 jtraj 起点
     * 用电机角而终点用关节角，轨迹计算就完全错了） */
    jtraj.q1_start = motor1_to_joint(g_ak80.status.position);   /* 大臂当前关节角 */
    jtraj.q2_start = motor2_to_joint(g_ak45.status.position);   /* 小臂当前关节角 */

    /* 终点：用户指定的目标角 */
    jtraj.q1_end = q1_end;                     /* 大臂目标角 */
    jtraj.q2_end = q2_end;                     /* 小臂目标角 */

    /* 时长：防止用户传 0 或负数导致除零，兜底成 0.001s */
    jtraj.duration = (duration > 0.0f) ? duration : 0.001f;

    /* 计时归零：从头开始走这条轨迹 */
    jtraj.elapsed = 0.0f;

    /* 激活轨迹：让 jtraj_update 开始工作 */
    jtraj.active = 1;
}

/*
 * 每周期调用一次，推进轨迹并把结果写进 motion。
 * 返回值：
 *   0  = 轨迹还在运行中（没走完）
 *   1  = 本次轨迹刚好走完（到达终点）
 *  -1  = 轨迹未激活（没调用过 jtraj_start）
 */
int jtraj_update(LegMotion_t *motion)
{
    /* 未激活：说明没启动过轨迹，直接返回 -1 */
    if (!jtraj.active) return -1;

    /* 累加时间：每个控制周期前进一个 CONTROL_DT */
    jtraj.elapsed += CONTROL_DT;

    /* 计算归一化时间 tau = 已走时间 / 总时长，范围 [0,1] */
    float tau = jtraj.elapsed / jtraj.duration;

    /* 判断是否到达终点 */
    if (tau >= 1.0f) {
        /* 时间到了：把 tau 钳到 1，并关闭轨迹 */
        tau = 1.0f;
        jtraj.active = 0;
    }

    /* ---- 五次多项式的中间变量，减少重复计算 ---- */
    float tau2 = tau * tau;      
    float tau3 = tau2 * tau;     

    /* 时长相关倒数：把"对归一化时间的导数"换算成"对真实时间的导数" */
    float inv_T  = 1.0f / jtraj.duration;   /* 1/T */
    float inv_T2 = inv_T * inv_T;           /* 1/T? */

    /* ---- 1. 位置 s(tau) = 10t? - 15t? + 6t? ----
     * 写成 Horner 形式：t?·(10 + t·(-15 + 6t))，省乘法、省精度损失 */
    float s = tau3 * (10.0f + tau * (-15.0f + 6.0f * tau));

    /* ---- 2. 速度 ds/dt = s'(tau) / T ----
     * 对 tau 求导再除以 T，得到对真实时间 t 的导数。
     * s'(tau) = 30t? - 60t? + 30t? = t?·(30 + t·(-60 + 30t)) */
    float s_dot = tau2 * inv_T * (30.0f + tau * (-60.0f + 30.0f * tau));

    /* ---- 3. 加速度 d方s/dt方 = s''(tau) / T方 ----
     * 对 tau 求二阶导再除以 T方，得到真实加速度。
     * s''(tau) = 60t - 180t方 + 120t方 = t·(60 + t·(-180 + 120t)) */
    float s_ddot = tau * inv_T2 * (60.0f + tau * (-180.0f + 120.0f * tau));

    /* ---- 把归一化轨迹 s 映射到两个关节的真实角度区间 ---- */
    /* 位移量 = 终点 - 起点 */
    float delta1 = jtraj.q1_end - jtraj.q1_start;   /* 大臂要转的总角度 */
    float delta2 = jtraj.q2_end - jtraj.q2_start;   /* 小臂要转的总角度 */

    /* 目标位置 = 起点 + 位移量 × 归一化位置 s */
    motion->q1_target = jtraj.q1_start + delta1 * s;   /* 大臂当前应处角度 */
    motion->q2_target = jtraj.q2_start + delta2 * s;   /* 小臂当前应处角度 */

    /* 目标速度 = 位移量 × 归一化速度 s_dot（前馈速度，供 MIT 的 V_des） */
    motion->joint1_speed_target = delta1 * s_dot;      /* 大臂期望速度 */
    motion->joint2_speed_target = delta2 * s_dot;      /* 小臂期望速度 */

    /* 目标加速度 = 位移量 × 归一化加速度 s_ddot（前馈加速度，供力矩前馈） */
    motion->joint1_acc_target = delta1 * s_ddot;       /* 大臂期望加速度 */
    motion->joint2_acc_target = delta2 * s_ddot;       /* 小臂期望加速度 */

    /* ---- 安全处理：限位钳位 + 角度换算 ---- */
    /* 先钳到软限位内，防止轨迹终点越过机械极限 */
    clamp_to_joint_limit(&motion->q1_target, &motion->q2_target);

    /* 关节角 → 电机角（当前 1:1，机械零位有偏差时改 joint_to_motor_*） */
    motion->motor1_target_angle = joint_to_motor_1(motion->q1_target);
    motion->motor2_target_angle = joint_to_motor_2(motion->q2_target);

    /* 返回：tau 已经到 1 → 完成返回 1；否则还在走 → 返回 0 */
    return (tau >= 1.0f) ? 1 : 0;
}
