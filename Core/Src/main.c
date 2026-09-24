/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hc_sr04.h"
#include "led.h"
#include "Moter.h"
#include "dht11.h"
#include "jy901s.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "PID.h"
#include "delay.h"
#include "HW.h"
#include "Transmit_Mv.h"
#include "jy906.h"
#include "headfile.h"
#include "vl53l0.h"
#include "Voice.h"
#include "sim800c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//激光测距
uint8_t L=0;
uint8_t R=0;
uint8_t previous_L=0;
uint8_t previous_R=0;
uint8_t Ahead_irSensor=1;
uint8_t Ahead_irSensor_Time=0;
//超声波模块
int upEdge=0;
int downEdge=0;
float distance=0;
uint8_t stop=1 ;
uint8_t hc=0;
//温湿度模块
uint8_t temperature = 1;
uint8_t humidity = 1;
uint8_t temperature_xs = 1;
uint8_t data_T=0;
uint8_t data_H=0;
//定时器
uint8_t cnt;
uint8_t cnt_trans;
uint8_t cnt2;
uint8_t cnt4;
//陀螺仪
float previous;
float setpoint;
float Initial_trend;
float Target_trend;
uint8_t frame1 = 0xFF;
uint8_t frame2 = 0xAA;
uint8_t delta = 0;
uint8_t voice_broadcast=1;
float PidOut1=0.00f;
float Pid_Out1=0.00f;
float Pid_Out2=0.00f;
uint8_t up=0;
//语音模块
uint8_t data;
uint8_t Voice[5]={0xAA, 0x55, 0x00, 0x55, 0xAA};
uint8_t vioce_turn=0;
//蓝牙模块
uint8_t Data=0;
uint16_t moter=300;
//Mv
uint8_t data4=0;         //修正
uint8_t data5=0;
uint8_t Way_Left=0;
uint8_t Way_Right=0;

uint8_t Left_Way=0;      //路况
uint8_t Right_Way=0;
uint8_t On_Nway=0;
uint8_t Left_Nway=0;
uint8_t Right_Nway=0;
uint8_t On_Stairs=0;
uint8_t Left_Stairs=0;
uint8_t Right_Stairs=0;
uint8_t On_Line=0;
uint8_t Left_Line=0;
uint8_t Right_Line=0;
uint8_t Green=0;
uint8_t Red=0;

uint8_t Observe=0;                             //通信
uint8_t mv_go[1]={0x01};  
uint8_t hc_stop[1]={0x02};
uint8_t mv_lead[1]={0x03};
uint8_t mv_rescue[1]={0x04};
uint8_t mv_observe[1]={0x05};
uint8_t mv_close[1]={0x06};
uint8_t mv_open[1]={0x07};
uint8_t mv_observe_way[1]={0x08};
uint8_t mv_light_on[1]={0x09};
uint8_t mv_light_off[1]={0x0A};
uint8_t mv_left[1]={0x0B};
uint8_t mv_right[1]={0x0C};
uint8_t mv_up[1]={0x0D};
uint8_t mv_down[1]={0x0E};
uint8_t mv_stop[1]={0x0F};
uint8_t mv_middle[1]={0xA0};
//ESP32
uint8_t data_esp;
uint8_t Rx_esp[1];
//LED
uint8_t led_state;
uint8_t led_count;
uint8_t Dance;
char str[100];
//声源检测
uint16_t AD_Value[4];
uint8_t voice=0;
//激光测距
uint32_t millis(void);
volatile uint32_t sysTickUptime = 0;
int L1 = 0,     L2 = 0,     L3 = 0,     R1 = 0,     R2 = 0,     R3 = 0;
int L1_ = 0,    L2_ = 0,    L3_ = 0,    R1_ = 0,    R2_ = 0,    R3_ = 0,   L3__=0,   R3__=0;
int p_L1_ = 0,  p_L2_ = 0,  p_L3_ = 0,  p_R1_ = 0,  p_R2_ = 0,  p_R3_ = 0;
int Target_Distance = 0;
double previous_error = 0;
int min_L3 = 150;  
int min_R3 = 150;  
//临时写死
uint8_t cnt_start_go=0;//pid转向前进
uint8_t cnt_start_go_=0;
uint8_t cnt_start_goline=0;//右转观察型号灯
uint8_t cnt_start_goline_=0;
uint8_t go_pid1=0;//pid绕行强制前进
uint8_t go_pid1_=0;
uint8_t go_pid2=0;//pid绕行强制前进
uint8_t go_pid2_=0;
uint8_t rescue=0;//救援模式
uint8_t Senser_led=0;//补光灯开启状态
uint8_t light_cnt=0;//补光灯
uint8_t cnt_start_l=0;//左绕行
uint8_t cnt_start_l_=0;
uint8_t cnt_start_r=0;//右绕行
uint8_t cnt_start_r_=0;
uint8_t ESP_Right_Way=0;
uint8_t ESP_Left_Stairs=0;
uint8_t ESP_Right_Line=0;
uint8_t ESP_On_Line=0;
uint8_t ESP_Red=0;
uint8_t ESP_Green=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//超声波
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		upEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		downEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		distance = ((downEdge - upEdge) * 0.034) / 2;
	}
}

//语音+蓝牙+陀螺仪
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//语音
	if(huart==&huart8){//前进
		if(data==0x01) {
			Moter_Go(); 
			LED_OFF();
			HAL_UART_Transmit(&huart1, mv_go, 1, 200); 
			stop=0;      
		}
     
		if(data==0x02){//后退
          Moter_Back();
		}
		if(data==0x03){//停
			Moter_Stop();
			stop=1;
			LED_OFF();
		}
		if(data==0x04){//右转
			previous = jy901_Angle.Angle[2]; 
			setpoint = previous-90;
			vioce_turn=1;
			Pid_init();
		}
		if(data==0x05){//左转
			previous = jy901_Angle.Angle[2]; 
			setpoint = previous+90;
			vioce_turn=1;
			Pid_init();
		}
		if(data==0x06){//温湿度
			Speech_dht11();
		}
		if(data==0x07){//导盲模式
			Data=0;
			HAL_UART_Transmit(&huart1, mv_lead, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_lead, 1, 200);
			rescue=0;			
		}
		if(data==0x08){//救援模式
			Moter_Stop();
			stop=1;
			LED_OFF();
			HAL_UART_Transmit(&huart1, mv_rescue, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_rescue, 1, 200); 
			rescue=1;
		}
		if(data==0x09){//观察信号灯情况
			Green=0;
            Red=0;
			HAL_UART_Transmit(&huart1, mv_observe, 1, 200); 
		}
		if(data==0x0A){//向左绕行
			Initial_trend=jy901_Angle.Angle[2];
		}
		if(data==0x0B){//向右绕行
			Initial_trend=jy901_Angle.Angle[2];
		}
		if(data==0x0C){//观察路况
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
		HAL_UART_Receive_IT(&huart8, &data, 1);
	}
//蓝牙
	if(huart==&huart6){
		if(Data==0x01){
			Moter_Go_(moter-200);
//			Moter_Go();
			stop=0;
		}
		if(Data==0x02){
			Moter_Back_(moter-200);
		}
		if(Data==0x03){
			Moter_Stop();
			stop=1;
			LED_OFF();
		}
		if(Data==0x04){
			Moter_Left_(moter);
			LED_LEFT_ON();
		}
		if(Data==0x05){
			Moter_Right_(moter);
			LED_RIGHT_ON();
		}
		if(Data==0x06)moter=300;

		if(Data==0x07)moter=350;
      
		if(Data==0x08)moter=400;
      
		if(Data==0x09)moter=450;
     
		if(Data==0x0A){//左
			HAL_UART_Transmit(&huart1, mv_left, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_left, 1, 200); 
		}
		if(Data==0x0B){//右
			HAL_UART_Transmit(&huart1, mv_right, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_right, 1, 200); 
		}
		if(Data==0x0C){//上
			HAL_UART_Transmit(&huart1, mv_up, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_up, 1, 200); 
		}
		if(Data==0x0D){//下
			HAL_UART_Transmit(&huart1, mv_down, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_down, 1, 200); 
		}
		if(Data==0x0E){//停
			HAL_UART_Transmit(&huart1, mv_stop, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_stop, 1, 200); 
		}
		if(Data==0x0F){//中
			HAL_UART_Transmit(&huart1, mv_middle, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_middle, 1, 200); 
		}
		HAL_UART_Receive_IT(&huart6, &Data, 1);
	}
//陀螺仪
	if(huart==&huart7){
		receive_complete_flag = 1;
		HAL_UART_Receive_IT(&huart7,(uint8_t*)receive_buff,USART_RECEIVE_LENGTH);													
	}
//MV
	if (huart==&huart1&&(Data==0x03||Data==0)){
		if(jy901_Angle.Angle[1]<=7&&data4==0x47&&stop==0&&L==0&&R==0&&data!=0x04&&data!=0x05&&data!=0x02){		//绕行不修正，逼退不修正
			Moter_Go();
		}
		if(jy901_Angle.Angle[1]<=7&&data4==0x6C&&stop==0&&L==0&&R==0&&data!=0x04&&data!=0x05&&data!=0x02&&R1>300){		//左偏
            Moter_Right();        
        }
        else if(jy901_Angle.Angle[1]<=7&&data4==0x4C&&stop==0&&L==0&&R==0&&data!=0x04&&data!=0x05&&data!=0x02&&R1>300){	//航线左偏
            Mv_Moter_Right();        
//			Mv_Moter_Right_();   			
        }
        if(jy901_Angle.Angle[1]<=7&&data4==0x72&&stop==0&&stop==0&&L==0&&R==0&&data!=0x04&&data!=0x05&&data!=0x02&&R1>300){		//右偏
            Moter_Left();        
        }
        else if(jy901_Angle.Angle[1]<=7&&data4==0x52&&stop==0&&stop==0&&L==0&&R==0&&data!=0x04&&data!=0x05&&data!=0x02&&R1>300){	//航线右偏
            Mv_Moter_Left();
//			Mv_Moter_Left_();   
        }
        if(data4==0x77){								//前方可通行                           
            Voice_Broadcast(0x0A);
        }
        if(data4==0x4B&&On_Nway==0){					//前方不可通行                           
            Voice_Broadcast(0x17);
            On_Nway=1;
        }
        if(data4==0x55&&On_Stairs==0){					//前面有楼梯                           
            Voice_Broadcast(0x16);
            On_Stairs=1;
        }
        if(data4==0x41&&On_Line==0){					//前面有斑马线                  
            Voice_Broadcast(0x15);
			ESP_On_Line=1;
            On_Line=1;
        }
        if(data4==0x45&&Red==0){                        //红灯停
            Moter_Stop();            
            Voice_Broadcast(0x09);
			ESP_Red=1;
            stop=1;  
            Red=1;
        }
        if(data4==0x46&&Green==0){						//绿灯行     
            Voice_Broadcast(0x0A);
			ESP_Green=1;
            Green=1;
			data=0x01;
			Moter_Go(); 
			LED_OFF();
			HAL_UART_Transmit(&huart1, mv_go, 1, 200); 
			HAL_UART_Transmit(&huart3, mv_go, 1, 200); 
			stop=0;  
        }
        HAL_UART_Receive_IT(&huart1,&data4,1);   
	}
	if (huart==&huart3){
		if(data5==0x71&&Left_Way==0){					//左边可通行
            Voice_Broadcast(0x11);
            Left_Way=1;
			L=0;
			R=0;
        }
        if(data5==0x51&&Right_Way==0){					//右边可通行  
			ESP_Right_Way=1;
			Voice_Broadcast(0x07);
            Right_Way=1;
			L=0;
			R=0;
        }
        if(Left_Stairs==0&&data5==0x56&&Left_Nway==0){	//左边不可通行                        
//            Voice_Broadcast(0x0B);
            Left_Nway=1;
        }
        if(data5==0x76&&Right_Nway==0){					//右边不可通行
//            Voice_Broadcast(0x14);
            Right_Nway=1;
        }
        if(data5==0x53&&Left_Stairs==0){				//左边有楼梯                   
            Voice_Broadcast(0x06);
			ESP_Left_Stairs=1;
            Left_Stairs=1;
        }
        if(data5==0x54&&Right_Stairs==0){				//右边有楼梯                        
            Voice_Broadcast(0x13);
            Right_Stairs=1;
        }
        if(data5==0x44&&Left_Line==0){					//左边有斑马线                       
            Voice_Broadcast(0x12);
            Left_Line=1;
        }
        if(data5==0x5A&&Right_Line==0){					//右边有斑马线                  
            Voice_Broadcast(0x08);
			ESP_Right_Line=1;
            Right_Line=1;
        }
//		if((Left_Stairs==1&&Right_Nway==1)||(Left_Nway==1&&Right_Nway==1)){
//			Voice_Broadcast(0x21);						//未检测到可通行道路
//		}
        HAL_UART_Receive_IT(&huart3,&data5,1);
  }
//ESP32
	if (huart==&huart4){
//		if (Rx_esp[0] == 'a'){							//导盲模式
//			Data=0;
//			HAL_UART_Transmit(&huart1, mv_lead, 1, 200); 
//			HAL_UART_Transmit(&huart3, mv_lead, 1, 200); 
//			rescue=0;
//		}
//		if (Rx_esp[0] == 'b'){							//救援模式
//			Moter_Stop();
//			stop=1;
//			LED_OFF();
//			HAL_UART_Transmit(&huart1, mv_rescue, 1, 200); 
//			HAL_UART_Transmit(&huart3, mv_rescue, 1, 200); 
//			rescue=1;
//		}
//		if (Rx_esp[0] == 0x41){							//前进
//			data=0x01;
//			Moter_Go(); 
//			LED_OFF();
//			HAL_UART_Transmit(&huart1, mv_go, 1, 200); 
//			HAL_UART_Transmit(&huart3, mv_go, 1, 200); 
//			stop=0;  
//		}
//		if (Rx_esp[0] == 'B'){							//后退
//			data=0x02;
//			Moter_Back();
//		}
//		if (Rx_esp[0] == 'C'){							//左转
//			data=0x05;
//			previous = jy901_Angle.Angle[2]; 
//			setpoint = previous+90;
//			vioce_turn=1;
//			Pid_init();
//		}
//		if (Rx_esp[0] == 'D'){							//右转
//			data=0x04;
//			previous = jy901_Angle.Angle[2]; 
//			setpoint = previous-90;
//			vioce_turn=1;
//			Pid_init();
//		}
//		if (Rx_esp[0] == 'E'){							//停车
//			data=0x03;
//			Moter_Stop();
//			stop=1;
//			LED_OFF();
//		}
		if (Rx_esp[0] == 'F'){							//跳舞
			data=0x03;
			stop=1;
			Moter_Stop();
			LED_OFF();
			Dance=1;
			led_count=1;
		}
		if (Rx_esp[0] == 'G'){							//灯光秀
			led_count=1;
		}
//		if (Rx_esp[0] == 'H'){							//左绕行
//			data=0x0A;
//			Initial_trend=jy901_Angle.Angle[2];
//		}
//		if (Rx_esp[0] == 'I'){							//右绕行		
//			data=0x0B;
//			Initial_trend=jy901_Angle.Angle[2];
//		}
//		if (Rx_esp[0] == 'K'){							//向右通行
//            Right_Way=1;
//		}
//		if (Rx_esp[0] == 'L'){							//前面有斑马线，观察并向前方通行
//            On_Line=1;
//		}
//		if (Rx_esp[0] == 'M'){							//右边有斑马线，观察并向右通行
//            Right_Line=1;
//		}
		memset(Rx_esp, 0, 1);
		HAL_UART_Receive_IT(&huart4, Rx_esp, 1);
	}
}

//0.1s定时器
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim == (&htim13)){
		cnt++;																				//陀螺仪刷新率
        cnt_trans++;																		//MV数据流刷新率
		
		if(Ahead_irSensor_Time==1)cnt4++;													//逼退
		if((Right_Way==1||Right_Line==1)&&On_Line==0){										//右边有盲道或斑马线且无盲道时延时并右转，否则不执行
            cnt2++;
        }
		if(Observe%2==1){																	//观察路况
            Observe+=2;
            if(Observe>=51){Observe=0;HAL_UART_Transmit(&huart3, mv_observe_way, 1, 200);}
        }
        if(!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)&&Ahead_irSensor%2==0&&Ahead_irSensor!=0){	//避障，过斑马线，盲道上1.5秒后开红外对管  Ahead_irSensor!=0（完全关红外）
            Ahead_irSensor+=2;
            if(Ahead_irSensor==24){
                Ahead_irSensor=1;
            }
        }
        else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)&&Ahead_irSensor!=1&&Ahead_irSensor!=0){
            Ahead_irSensor=2;
        }
		
		
        if (led_count%2==1){																//灯光秀
            if(!led_state){
                LED_OFF();
                LED_LEFT_ON();
                led_state = 1;
            }
            else if(led_state){
                LED_OFF();
                LED_RIGHT_ON();
                led_state = 0;
            } 
            led_count+=2;
            if (led_count >= 101){led_count=0;LED_OFF();}
        }
		if(Dance%2==1){																		//跳舞
            Dance+=2;
            if(Dance>=65){Moter_Stop();Dance=0;}
      
            else if(Dance>=57)Moter_Left();
         
            else if(Dance>=47)Moter_Stop();
           
            else if(Dance>=39)Moter_Right();
            
            else if(Dance>=29)Moter_Stop();
            
            else if(Dance>=21)Moter_Right();
            
            else if(Dance>=11)Moter_Stop();

            else if(Dance>=1)Moter_Left();

        }
		if(cnt_start_go==1){
			cnt_start_go_++;
		}
		else if(cnt_start_goline==1){
			cnt_start_goline_++;
		}
		if(go_pid1==1){
			go_pid1_++;
		}
		else if(go_pid2==1){
			go_pid2_++;
		}
		if(cnt_start_l==1){
			cnt_start_l_++;
		}
		else if(cnt_start_r==1){
			cnt_start_r_++;
		}
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
		delay_init(240);
        HAL_Delay(2000);
		
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_UART8_Init();
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//超声波
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
//语音
    HAL_UART_Receive_IT(&huart8, &data,1);
//蓝牙
    HAL_UART_Receive_IT(&huart6, &Data,1);
	Start_Recv();
//陀螺仪
  	HAL_TIM_Base_Start_IT(&htim13);
	HAL_UART_Receive_IT(&huart7,(uint8_t*)receive_buff,USART_RECEIVE_LENGTH);
	vJY901S_Send_Command(KEY,0x88,0xB5);							
	HAL_Delay(200);
	vJY901S_Send_Command(CALSW,0x08,0x00);					
	vJY901S_Send_Command(GYROCALITHR,0x01,0x00);    
	vJY901S_Send_Command(CALSW,0x04,0x00);	   
	HAL_Delay(1000);																	
	vJY901S_Send_Command(SAVE,0x00,0x00);								
//Mv
    HAL_UART_Receive_IT(&huart1,&data4,1);
    HAL_UART_Receive_IT(&huart3,&data5,1);
//ESP32
    HAL_UART_Receive_IT(&huart4, Rx_esp, 1);
//激光测距
	Tof_Init();
//声源检测
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AD_Value, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  DHT11_Rst();
  ESP8266_Setmode(2);
  ESP8266_Set_Wifi();
  ESP8266_Set_TCP();
  Voice_Broadcast(0x0D);
  while (1)
  {
	Obstacle_avoid();						//自动避障
	Distance_detect();						//超声波测距播报
	Control_jy901s();						//陀螺仪控制
	Mv_All();								//视频流传输
	Brightness_Senser();					//照明灯控制
	Way_detect();                          	//逼停
	Temperature_measure();                 	//测温播报
	Humidity_measure();						//湿度播报   
	Voice_Detect();							//声源检测
	if(cnt_start_go_>15)//pid直角转向后，强制前进
	{
		cnt_start_go=0;
		cnt_start_go_=0;
		Moter_Go(); 
        LED_OFF();
        HAL_UART_Transmit(&huart1, mv_go, 1, 200);
        stop=0;            
        Right_Way=0;
        data=0x01;
	}
	if(cnt_start_goline_>15){//pid直角转向后，强制观察信号灯
		cnt_start_goline=0;
		cnt_start_goline_=0;
		Moter_Stop();
        LED_OFF();
        HAL_UART_Transmit(&huart1, mv_go, 1, 200);
        stop=1;  
        Right_Line=3;
        data=0x03;
		Green=0;
        Red=0;
		HAL_UART_Transmit(&huart1, mv_observe, 1, 200); 
		Voice_Broadcast(0x20);
		ESP8266_Send_ESP32("8",100);
	}
	if(go_pid1_>10){//pid绕行强制前进
		go_pid1=0;
		go_pid1_=0;
		if(L==3){
			L=4;
            Moter_Go(); 
            LED_OFF();
            if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
            stop=0;
            data=0x01;
			Ahead_irSensor=2;
		}
		if(R==3){
			R=4;
            Moter_Go(); 
            LED_OFF();
            if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
            stop=0;
            data=0x01;
            Pid_init();
			Ahead_irSensor=2;
		}
	}
	if(go_pid2_>10){//pid绕行强制前进
		go_pid2=0;
		go_pid2_=0;
		L=0;
		R=0;
		ESP8266_Send_ESP32("15",100);
        Moter_Go(); 
        LED_OFF();
        if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
        stop=0;
        data=0x01;
	}
	if(cnt_start_l_>20){
		if(L==0){
			cnt_start_l=0;
			cnt_start_l_=0;
			data=0x0B;
			Initial_trend=jy901_Angle.Angle[2];
		}
		else{
			cnt_start_r=0;
			cnt_start_r_=0;
		}
	}
	else if(cnt_start_r_>20){
		if(R==0){
			cnt_start_r=0;
			cnt_start_r_=0;
			data=0x0A;
			Initial_trend=jy901_Angle.Angle[2];
		}
		else{
			cnt_start_r=0;
			cnt_start_r_=0;
		}
	}
	if(ESP_Right_Way){
		ESP8266_Send_ESP32("4",100);
		ESP_Right_Way=0;
	}
	if(ESP_Left_Stairs){
		ESP8266_Send_ESP32("5",100);
		ESP_Left_Stairs=0;
	}
	if(ESP_Right_Line){
		ESP8266_Send_ESP32("6",100);
		ESP_Right_Line=0;
	}
	if(ESP_On_Line){
		ESP8266_Send_ESP32("7",100);
		ESP_On_Line=0;
	}
	if(ESP_Red){
		ESP8266_Send_ESP32("A",100);
		ESP_Red=0;
	}
	if(ESP_Green){
		ESP8266_Send_ESP32("9",100);
		ESP_Green=0;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1000);
    return ch;
}
uint32_t millis(void)
{
    return sysTickUptime;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
