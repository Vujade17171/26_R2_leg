#include "Control_pos.h"
leg_pos_t pos;
void Forward_Kinematics(leg_pos_t*my_pos)
{
    float q1 = g_ak80.status.position;
    float q12 = q1 + g_ak45.status.position;//需更改为AK45

    // 2. 计算小臂端点坐标 
    pos.x = L1 * cosf(q1) + L2 * cosf(q12);
    pos.y = 0.0f;
    pos.z = L1 * sinf(q1) + L2 * sinf(q12);

    // 3. 计算绝对姿态角并叠加 L3 的分量得到最终末端落点 (xs, zs)
    float abs_angle = q12 + g_ak80.status.position;//需更改为灵足05
    
    pos.x_s = pos.x + L3 * cosf(abs_angle);
    pos.z_s = pos.z + L3 * sinf(abs_angle);

    pos.yaw = 0.0f;
}



