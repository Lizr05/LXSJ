#include "Moter.h"


void Moter_Stop_Ramp(void)
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,100);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,100); 
}

void Moter_Go(void)//前进
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,170);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,170); 
}

void Moter_Go_Up(void)//前进
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,450);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,450);
}

void Moter_Go_Down(void)//前进
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,100);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,100);
}

void Moter_Back(void)//后退
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,200);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,200);
}

void Moter_Left__(void)//绕行左
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,400);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
}

void Moter_Right__(void)//绕行右
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,0);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,400);
}

void Moter_Left(void)//左转
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,600);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,600);

}

void Moter_Right(void)//右转
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,650);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,600);
}

void Moter_Go_(uint16_t Moter)//蓝牙前进
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,Moter);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Moter);
}

void Moter_Back_(uint16_t Moter)//蓝牙后退
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,Moter);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Moter);
}

void Moter_Left_(uint16_t Moter)//蓝牙左转
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,Moter);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Moter);

}

void Moter_Right_(uint16_t Moter)//蓝牙右转
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,Moter);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Moter);
}

void Moter_Stop(void)//停止
{
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);
}

void PID_Moter_Left(float out)//左转
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,out);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,out);

}

void PID_Moter_Right(float out)//右转
{
    HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,out);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,out);
}

void Moter_Control(int left,int right)
{
	HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,left);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,right); 
}

void Moter_Control_(int left_,int right_)
{
	HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,left_);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,right_); 
}

void Mv_Moter_Left(void)//左转
{
        HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
        HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
        __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,100);
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,500);
}



void Mv_Moter_Right(void)//右转
{
        HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
        HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
        __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,500);
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,100);
}

void Mv_Moter_Left_(void)//左转
{
        HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_RESET);
        HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
        __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,80);
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,300);
}

void Mv_Moter_Right_(void)//右转
{
        HAL_GPIO_WritePin(M11_GPIO_Port,M11_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M12_GPIO_Port,M12_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M21_GPIO_Port,M21_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M22_GPIO_Port,M22_Pin,GPIO_PIN_SET);
        HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
        __HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,300);
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
        __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,80);
}


