#include "dht11.h"

//extern TIM_HandleTypeDef htim2;

uint8_t th_voice[7] = {0xAA, 0x55, 0x02, 0x00, 0x00, 0x55, 0xAA};
uint8_t Humidity_Broadcast=0;

void Delay_us(uint16_t delay)
{
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_ENABLE(&htim2);
    uint16_t curCnt = 0;
    while(1)
    {
        curCnt = __HAL_TIM_GET_COUNTER(&htim2);
        if(curCnt >= delay)
            break;
    }
    __HAL_TIM_DISABLE(&htim2);
}

void Dht11_DATA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* Configure GPIO pin : PB12 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// 输入input
void Dht11_DATA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* Configure GPIO pin : PB12 */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void DHT11_Rst(void)
{
    Dht11_DATA_OUT();
    // 设置为输出
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // 拉低引脚
    HAL_Delay(20); // 延迟20ms
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // 拉高引脚
    Delay_us(30); // 延迟30us
}

uint8_t DHT11_Check(void)
{
    uint8_t retry = 0;
    Dht11_DATA_IN(); // 设置为输入
    while(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    if(retry >= 100)
        return 1;
    else
        retry = 0;
    while(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    if(retry >= 100)
        return 1;
    return 0;
}

uint8_t DHT11_Read_Bit(void)
{
    uint8_t retry = 0;
    while(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    retry = 0;
    while(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) && retry < 100)
    {
        retry++;
        Delay_us(1);
    }
    Delay_us(40);
    if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
        return 1;
    else
        return 0;
}

uint8_t DHT11_Read_Byte(void)
{
    uint8_t dat = 0;
    for(uint8_t i = 0; i < 8; i++)
    {
        dat <<= 1;
        dat |= DHT11_Read_Bit();
    }
    return dat;
}

uint8_t DHT11_Read_Data(uint8_t* humi, uint8_t* temp, uint8_t* temp_xs)
{
    uint8_t buf[5];
    DHT11_Rst();
    if(DHT11_Check() == 0)
    {
        for(uint8_t i = 0; i < 5; i++)
            buf[i] = DHT11_Read_Byte();
        if((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            *humi = buf[0];
            // 这里省略小数部分
            *temp = buf[2];
            *temp_xs = buf[3];
        }
    }
    else
        return 1;
    return 0;
}


void Speech_dht11(void)	//播报温湿度
{
    th_voice[3]=temperature;
    th_voice[4]=humidity;
    HAL_UART_Transmit_IT(&huart8, th_voice, sizeof(th_voice));
}

void Humidity_measure(void){	//高湿度警告
    if(humidity>=63&&Humidity_Broadcast==0){
		ESP8266_Send_ESP32("12",100);
        Voice_Broadcast(0x19);
		data_H=1;
        Humidity_Broadcast=1;
    }
	else if(humidity<=55){
		data_H=0;
	}
}
