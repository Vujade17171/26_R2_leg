#include "pid.h"


void PID_init(pid_type *pid,float Kp,float Ki,float Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	
	pid->last_val   = 0.00;
	pid->actual_val = 0.00;
	pid->target_val = 0.00;
	pid->total_val  = 0.00;
	
	pid->err      = 0.00;
	pid->err_last = 0.00;
	pid->err_sum  = 0.00;

	pid->pidoutput = 0.00;
	
}

float PID_realize(pid_type  * pid,float actual_val)
{
	pid->actual_val           = actual_val;
	pid->err                  = pid->target_val - pid->actual_val;
	pid->err_sum              += pid->err;
	pid->err_last             = pid->err;
	
	if(pid->err_sum >= 10000)
	{
		pid->err_sum = 10000;
	}
	if(pid->err_sum <= -10000)
	{
		pid->err_sum = -10000;
	}

	pid->pidoutput = (pid->Kp*pid->err) + (pid->Ki*pid->err_sum) + (pid->Kd*(pid->err - pid->err_last));
	
	if(pid->pidoutput>10000){
	  pid->pidoutput=10000;
	}
	else if(pid->pidoutput<-10000){
	  pid->pidoutput=-10000;
	}
	return pid->pidoutput;
}


