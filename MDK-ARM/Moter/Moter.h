#ifndef MOTER_H_
#define MOTER_H_

#include "main.h"

void Moter_Stop_Ramp(void);
void Moter_Go(void);
void Moter_Go_(uint16_t Moter);
void Moter_Go_Up(void);
void Moter_Go_Down(void);
void Moter_Back(void);
void Moter_Left__(void);
void Moter_Right__(void);
void Moter_Left(void);
void Moter_Right(void);
void Moter_Left_(uint16_t Moter);
void Moter_Right_(uint16_t Moter);
void Moter_Back_(uint16_t Moter);
void Moter_Stop(void);
void PID_Moter_Left(float out);
void PID_Moter_Right(float out);
void Moter_Control(int left,int right);
void Moter_Control_(int left_,int right_);
void Mv_Moter_Left(void);
void Mv_Moter_Right(void);
void Mv_Moter_Left_(void);
void Mv_Moter_Right_(void);


#endif 
