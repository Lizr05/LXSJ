#include "headfile.h"

uint8_t ESP_Buff[BUFF_SIZE]={0};
char cmd[128];
uint8_t TCP=0;

extern UART_HandleTypeDef huart5;//ESP8266
extern UART_HandleTypeDef huart3;//调试

//中断接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	if (huart==&huart5)
	{	
		// 读取接收到的数据
		if(ParseUsartFrame((char *)ESP_Buff,"connect")){
			Voice_Broadcast(0x1F);
			TCP=1;
		}
		
		if(ParseUsartFrame((char *)ESP_Buff,"1")){
			data=0x01;
			Moter_Go(); 
			LED_OFF();
			HAL_UART_Transmit(&huart1, mv_go, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_go, 1, 200); 
			stop=0;  
			
		}
		if(ParseUsartFrame((char *)ESP_Buff,"2")){
			data=0x05;
			previous = jy901_Angle.Angle[2]; 
			setpoint = previous+90;
			vioce_turn=1;
			Pid_init();
			
		}
		if(ParseUsartFrame((char *)ESP_Buff,"3")){
			data=0x04;
			previous = jy901_Angle.Angle[2]; 
			setpoint = previous-90;
			vioce_turn=1;
			Pid_init();
		}
		if(ParseUsartFrame((char *)ESP_Buff,"4")){
			data=0x02;
			Moter_Back();
		}
		if(ParseUsartFrame((char *)ESP_Buff,"5")){
			data=0x03;
			Moter_Stop();
			stop=1;
			cnt_start_l=0;
			cnt_start_r=0;
			L=0;
			R=0;
			LED_OFF();
		}
		if(ParseUsartFrame((char *)ESP_Buff,"6")){//观察路况
			Left_Way=0;
            Right_Way=0;
            On_Nway=0;
            Left_Nway=0;
            Right_Nway=0;
            On_Stairs=0;
            Left_Stairs=0;
            Right_Stairs=0;
			On_Line=0;
			Left_Line=0;
			Right_Line=0;
			HAL_UART_Transmit(&huart1, mv_go, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_go, 1, 200); 
			HAL_UART_Transmit(&huart1, mv_observe_way, 1, 200); 
			Observe=1;
		}
		if(ParseUsartFrame((char *)ESP_Buff,"7")){//观察信号灯
			Green=0;
            Red=0;
			HAL_UART_Transmit(&huart1, mv_observe, 1, 200); 
		}
		if(ParseUsartFrame((char *)ESP_Buff,"8")){
			data=0x0A;
			Initial_trend=jy901_Angle.Angle[2];
		}
		if(ParseUsartFrame((char *)ESP_Buff,"9")){
			data=0x0B;
			Initial_trend=jy901_Angle.Angle[2];
		}
		if(ParseUsartFrame((char *)ESP_Buff,"A")){
			Data=0;
			HAL_UART_Transmit(&huart1, mv_lead, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_lead, 1, 200);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_SET);	
			data=0x01;
			Moter_Go(); 
			LED_OFF();
			HAL_UART_Transmit(&huart1, mv_go, 1, 200); 
			stop=0;  			
			rescue=0;
		}
		if(ParseUsartFrame((char *)ESP_Buff,"B")){
			Moter_Stop();
			stop=1;
			LED_OFF();
			HAL_UART_Transmit(&huart1, mv_rescue, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_rescue, 1, 200); 
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_RESET);
			rescue=1;
		}
		if(ParseUsartFrame((char *)ESP_Buff,"C")){//按键松开
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_RESET);
		}
		if(ParseUsartFrame((char *)ESP_Buff,"D")){//按键按下
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_SET);
		}
		if(ParseUsartFrame((char *)ESP_Buff,"E")){
			sim800c_init();
		}
		
		memset((char *)ESP_Buff,0,1024);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5,ESP_Buff,BUFF_SIZE);
	}
}

//验证并接收客户端消息
int ParseUsartFrame(const char *buf,const char *check_data)
{
    /* 1. 固定头 7 字节校验 */
    if (strstr((char *)buf, "+IPD") == NULL){
		return 0;
	}
    /* 2. 业务判断 */
	char *data_start = strchr(buf, ':'); 
	if (data_start) {
		data_start++; // 跳过冒号本身
		if(strstr((char *)data_start,check_data)!=NULL){
			return 1;
			}
	}
	return 0;
}

//启动的接收
void Start_Recv(void)
{
	memset(ESP_Buff,0,BUFF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,ESP_Buff,BUFF_SIZE);
}

//ESP8266通信
int ESP_SendCommand(const char* cmd,const char* resp,uint32_t timeout)
{
	HAL_UART_Transmit(&huart5,(uint8_t *)cmd,strlen(cmd),1000);
	
	uint32_t start_tick = HAL_GetTick();
	while(HAL_GetTick() -start_tick<timeout)
	{
		if(strstr((char *)ESP_Buff,resp)!=NULL)
		{
			return 0;
		}
	}
	return -1;
}

////设置模式
int ESP8266_Setmode(uint8_t mode) 
{
    // 清空缓冲区
    memset(ESP_Buff, 0, BUFF_SIZE);
   
    // 设置模式
    switch (mode) 
    {
        case 1:
            if(ESP_SendCommand("AT+CWMODE=1\r\n", "OK", 1000)==0)
            {
                printf("Set mode1 succcess\r\n");
                return 0;
            }
            break;
        case 2:
            if(ESP_SendCommand("AT+CWMODE=2\r\n", "OK", 1000)==0)
            {
                printf("Set mode2 succcess\r\n");
                return 0;
            }
            break;
        case 3:
            if(ESP_SendCommand("AT+CWMODE=3\r\n", "OK", 1000)==0)
            {
                printf("Set mode3 succcess\r\n");
                return 0;
            }
            break;
    }
    printf("Set mode failed\r\n");
    return -1;
}

////建立WIFI
int ESP8266_Set_Wifi(void)
{
	memset(ESP_Buff, 0, BUFF_SIZE);
    memset(cmd, 0, 128);
  

    sprintf(cmd,"AT+CWSAP=\"%s\",\"%s\",1,4\r\n", TCP_SSID, TCP_PASS);
	if (ESP_SendCommand(cmd, "OK", 1000) == 0) 
	{
		printf("WIFI set success!\r\n");
		return 0;
	}
	printf("WIFI set error!\r\n");
    return -1;
}

////建立多连接,TCP服务器
int ESP8266_Set_TCP(void)
{
	memset(ESP_Buff, 0, BUFF_SIZE);
    memset(cmd, 0, 128);
    

    //建立多连接
    if (ESP_SendCommand("AT+CIPMUX=1\r\n", "OK", 1000) == 0) 
    {
        printf("MultiConnection success\r\n");
    }
    else
    {
        printf("MultiConnection error\r\n");
    }
    
    //建立TCP服务器
    sprintf(cmd,"AT+CIPSERVER=1,8086\r\n");
	if (ESP_SendCommand(cmd, "OK", 1000) == 0) 
	{
		printf("TCP set success!\r\n");
		return 0;
	}
	printf("TCP set error!\r\n");
    return -1;   
}

////连接WIFI
int ESP8266_Connect_Wifi(void)
{
	memset(ESP_Buff, 0, BUFF_SIZE);
    memset(cmd, 0, 128);
    
    sprintf(cmd,"AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASS);
	if (ESP_SendCommand(cmd, "OK", 1000) == 0) 
	{
		printf("WiFi connected success!\r\n");
		return 0;
	}
	printf("WiFi connected error!\r\n");
    return -1;
}

////数据透传
int ESP8266_SendToTCPServer(const char* txData, int timeout) 
{
		memset(ESP_Buff, 0, BUFF_SIZE);
    
    // 等待进入数据模式
    sprintf(cmd, "AT+CIPSEND=0,%d\r\n", strlen(txData));
    if (ESP_SendCommand(cmd, ">", timeout) == -1)
    {
        printf("Enter data mode failed\r\n");
        return -1;
    }
    
    // 发送数据并等待确认
    if (ESP_SendCommand(txData, "SEND OK", timeout) == -1) 
    {
        printf("Send data failed\r\n");
        return -1;
    }
    printf("Send data success\r\n");
    return 0;
}

void ESP8266_Send_ESP32(const char* txData_, int timeout_){
	if(TCP){
		ESP8266_SendToTCPServer(txData_, timeout_);
	}
}

