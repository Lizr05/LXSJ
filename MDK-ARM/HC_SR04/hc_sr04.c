#include "hc_sr04.h"

void Distance_detect(void)
{
    HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);
    __HAL_TIM_SetCounter(&htim2,0);
    HAL_Delay(20);

    if(distance>=4&&distance<=15&&stop==0)	//前方有障碍物&&data!=0x0A&&data!=0x0B
    {
        Moter_Stop();
        stop=1;
		ESP8266_Send_ESP32("10",100);
        Voice_Broadcast(0x01);
        HAL_UART_Transmit(&huart1, hc_stop , 1, 200);
        hc=1;		//不执行左右障碍物逼停
		cnt_start_l=0;
		cnt_start_r=0;
//		L=0;
//		R=0;
		LED_OFF();
		HAL_UART_Transmit(&huart1, mv_observe_way, 1, 200); 
		Observe=1;
    }
    else if(distance>20){//恢复左右障碍物的逼停
		if(hc==1)Voice_Broadcast(0x1B);
		hc=0;
	}							
}

void Voice_Broadcast(uint8_t voice) 								//播报前方障碍物
{
    Voice[2] = voice;
    HAL_UART_Transmit_IT(&huart8, Voice, sizeof(Voice));
}
