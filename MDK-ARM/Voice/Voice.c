#include "Voice.h"


uint16_t max0 = 0, min0 = 65535;
uint16_t max1 = 0, min1 = 65535;
uint16_t max2 = 0, min2 = 65535;
uint16_t max3 = 0, min3 = 65535;

uint32_t last_stat_time = 0;

void Voice_Detect(void){
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)==0){
		if(voice%2==1){
			voice+=2;
			if(voice>=40)voice=0;
			}
		uint16_t scaled_value0 = (AD_Value[0] * 10000) / 65535;
		uint16_t scaled_value3 = (AD_Value[3] * 10000) / 65535;
    
		sprintf(str, "left=%d  right=%d\r\n", scaled_value0,scaled_value3);
		HAL_UART_Transmit(&huart6, (uint8_t*)str, strlen(str), 100);

		if (scaled_value0 > max0) max0 = scaled_value0;
		if (scaled_value0 < min0) min0 = scaled_value0;

		if (scaled_value3 > max3) max3 = scaled_value3;
		if (scaled_value3 < min3) min3 = scaled_value3;		
		
		uint32_t current_time = HAL_GetTick();


		if (current_time - last_stat_time >= 1000){
			last_stat_time = current_time;

			uint16_t diff0 = max0 - min0;
			uint16_t diff1 = max1 - min1;
			uint16_t diff2 = max2 - min2;
			uint16_t diff3 = max3 - min3;
        
			if(diff0>5000 ||diff1>5000 ||diff2>5000 ||diff3>5000){
				char str[8];

				if (diff0 > diff1 && diff0>5000 && voice==0){			//◊Û
					ESP8266_SendToTCPServer("C", 100);
					voice=1;
				} 
				else if (diff3 > diff2 && diff3>5000 && voice==0){		//”“
					ESP8266_SendToTCPServer("D", 100);
					voice=1;
				}
        
				max0 = min0 = scaled_value0;
				max3 = min3 = scaled_value3;

			}
		}
	}
}
