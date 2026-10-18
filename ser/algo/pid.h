#ifndef __PID_H
#define __PID_H


typedef struct
{
	float Kp,Ki,Kd;//比例，积分，微分系数
	
	float total_val;//总计值
	float last_val;//历史值
	float target_val;//目标值
    float actual_val;//实际值
	
	float err;//当前偏差
	float err_last;//上次偏差
	float err_sum;//误差累计值
	
	float pidoutput;//pid输出
	
}pid_type;


void PID_init(pid_type *pid,float Kp,float Ki,float Kd);

float PID_realize(pid_type  * pid,float actual_val);


#endif

