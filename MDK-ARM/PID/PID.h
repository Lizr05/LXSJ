#ifndef PID_H_
#define PID_H_

#include "main.h"

typedef struct 
{
    float target,actual,out;
    float Kp,Ki,Kd;
    float error,pre_error,int_error;
} tPid;

void Pid_init(void);
void Pid_init_2(void);
float Pid_Compute(tPid *pid, float setpoint, float actual_value);



#endif 


