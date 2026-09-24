#include "PID.h"

tPid pid_1;

void Pid_init(void)		//pid转弯
{
	pid_1.Kp=10;
	pid_1.Ki=0;
	pid_1.Kd=2;	
}

void Pid_init_2(void)	//pid绕行
{
	pid_1.Kp=20;//17
	pid_1.Ki=0;
	pid_1.Kd=1;//4
}


float Pid_Compute(tPid *pid, float setpoint, float actual_value) 	//pid算动力
{
    pid->error = setpoint - actual_value;
    if(pid->error>100){pid->error-=360;}
    if(pid->error<-100){pid->error+=360;}
    pid->int_error += pid->error;
    pid->out = pid->Kp * pid->error + pid->Ki * pid->int_error + pid->Kd * (pid->error-pid->pre_error);
    pid->pre_error = pid->error;
    if(pid->out>600) {pid->out=600;}
    if(pid->out<-600) {pid->out=-600;}
    return pid->out;
}
