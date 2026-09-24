#include "led.h"


void LED_LEFT_ON(void)//×óµÆÁÁ
{
    HAL_GPIO_WritePin(LED_LEFT_GPIO_Port,LED_LEFT_Pin,GPIO_PIN_SET);
}

void LED_RIGHT_ON(void)//ÓÒµÆÁÁ
{
    HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port,LED_RIGHT_Pin,GPIO_PIN_SET);
}

void LED_OFF(void)//ÃðµÆ
{
    HAL_GPIO_WritePin(LED_LEFT_GPIO_Port,LED_LEFT_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port,LED_RIGHT_Pin,GPIO_PIN_RESET);
}

void Body_check(void)//¼ì²âÐÐÈË
{
    if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0) == GPIO_PIN_SET)//YES
	{

	}
	else //NO
    {
		
    }
}

void Brightness_Senser(void)//ÕÕÃ÷µÆ
{
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1&&Senser_led==0)
    {
		light_cnt++;
        if(light_cnt>=10){
			HAL_UART_Transmit(&huart1, mv_light_on, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_light_on, 1, 200); 
			Voice_Broadcast(0x1A);
			light_cnt=0;
			Senser_led=1;
		}
    }
    else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0&&Senser_led==1)
    {
        HAL_UART_Transmit(&huart1, mv_light_off, 1, 200); 
        HAL_UART_Transmit(&huart3, mv_light_off, 1, 200); 
        Senser_led=0;
    }
}


