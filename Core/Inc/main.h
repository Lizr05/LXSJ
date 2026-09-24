/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "dma.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
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
extern char str[100];
//ºìÍâ¶Ô¹Ü
extern uint8_t L;
extern uint8_t R;
extern uint8_t previous_L;
extern uint8_t previous_R;
extern uint16_t tempData1;
extern uint16_t tempData2;
extern uint16_t tempData3;
extern uint16_t tempData4;
extern uint16_t previous_tempData1;
extern uint16_t previous_tempData2;
extern uint16_t previous_tempData3;
extern uint16_t previous_tempData4;
extern uint8_t Ahead_irSensor;
extern uint8_t Ahead_irSensor_Time;
//³¬Éù²¨
extern float distance;
extern uint8_t stop;
extern uint8_t hc;
//ÎÂÊª¶ÈÄ£¿é
extern uint8_t temperature;
extern uint8_t humidity;
extern uint8_t temperature_xs;
extern uint8_t data_T;
extern uint8_t data_H;
//extern TIM_HandleTypeDef htim2;
//¶¨Ê±Æ÷
extern uint8_t cnt;
extern uint8_t cnt_trans;
extern uint8_t cnt2;
extern uint8_t cnt4;
//ÍÓÂÝÒÇ
extern float previous;
extern float setpoint;
extern float Initial_trend;
extern float Target_trend;
extern uint8_t frame1;
extern uint8_t frame2;
extern uint8_t delta;
extern uint8_t voice_broadcast;
extern float PidOut1;
extern float Pid_Out1;
extern float Pid_Out2;
extern uint8_t up;
//ÓïÒôÄ£¿é
extern uint8_t data;
extern uint8_t Voice[5];
extern uint8_t vioce_turn;
//À¶ÑÀÄ£¿é
extern uint8_t Data;
//Mv
extern uint8_t data4;
extern uint8_t data5;


extern uint8_t Left_Way;
extern uint8_t Right_Way;
extern uint8_t On_Nway;
extern uint8_t Left_Nway;
extern uint8_t Right_Nway;
extern uint8_t On_Stairs;
extern uint8_t Left_Stairs;
extern uint8_t Right_Stairs;
extern uint8_t On_Line;
extern uint8_t Left_Line;
extern uint8_t Right_Line;
extern uint8_t Green;
extern uint8_t Red;

extern uint8_t Observe;
extern uint8_t mv_go[1];
extern uint8_t hc_stop[1];
extern uint8_t mv_lead[1];
extern uint8_t mv_rescue[1];
extern uint8_t mv_observe[1];
extern uint8_t mv_close[1];
extern uint8_t mv_observe_way[1];
extern uint8_t mv_light_on[1];
extern uint8_t mv_light_off[1];
extern uint8_t mv_update[1];
//ÉùÔ´¼ì²â
extern uint16_t AD_Value[4];
extern uint8_t voice;
//¼¤¹â²â¾à
extern int L1, L2, L3, R1, R2, R3;
extern int L1_, L2_, L3_, R1_, R2_, R3_, L3__, R3__;
extern int p_L1_, p_L2_, p_L3_, p_R1_, p_R2_, p_R3_;
extern int Target_Distance;
extern double previous_error;
extern int min_L3;
extern int min_R3;
//ÁÙÊ±Ð´ËÀ
extern uint8_t cnt_start_go;
extern uint8_t cnt_start_go_;
extern uint8_t cnt_start_goline;
extern uint8_t cnt_start_goline_;
extern uint8_t go_left;
extern uint8_t go_right;
extern uint8_t go_pid1;
extern uint8_t go_pid1_;
extern uint8_t go_pid2;
extern uint8_t go_pid2_;
extern uint8_t rescue;
extern uint8_t Senser_led;
extern uint8_t light_cnt;
extern uint8_t cnt_raoxing;
extern uint8_t duiguan_l;
extern uint8_t duiguan_r;
extern uint8_t cnt_start_l;
extern uint8_t cnt_start_r;	


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L_1_Pin GPIO_PIN_2
#define L_1_GPIO_Port GPIOE
#define L_2_Pin GPIO_PIN_3
#define L_2_GPIO_Port GPIOE
#define L_3_Pin GPIO_PIN_4
#define L_3_GPIO_Port GPIOE
#define R_1_Pin GPIO_PIN_5
#define R_1_GPIO_Port GPIOE
#define R_3_Pin GPIO_PIN_13
#define R_3_GPIO_Port GPIOC
#define IO_Pin GPIO_PIN_0
#define IO_GPIO_Port GPIOC
#define DATA_Pin GPIO_PIN_1
#define DATA_GPIO_Port GPIOC
#define Light_Senser_Pin GPIO_PIN_0
#define Light_Senser_GPIO_Port GPIOA
#define Trig_Pin GPIO_PIN_4
#define Trig_GPIO_Port GPIOA
#define Echo_Pin GPIO_PIN_5
#define Echo_GPIO_Port GPIOA
#define M21_Pin GPIO_PIN_5
#define M21_GPIO_Port GPIOC
#define M22_Pin GPIO_PIN_0
#define M22_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_1
#define STBY_GPIO_Port GPIOB
#define R_2_Pin GPIO_PIN_10
#define R_2_GPIO_Port GPIOE
#define HWDG2_Pin GPIO_PIN_13
#define HWDG2_GPIO_Port GPIOE
#define HWDG1_Pin GPIO_PIN_14
#define HWDG1_GPIO_Port GPIOE
#define TH_Watch_Pin GPIO_PIN_10
#define TH_Watch_GPIO_Port GPIOD
#define LED_RIGHT_Pin GPIO_PIN_14
#define LED_RIGHT_GPIO_Port GPIOD
#define LED_LEFT_Pin GPIO_PIN_8
#define LED_LEFT_GPIO_Port GPIOA
#define M12_Pin GPIO_PIN_12
#define M12_GPIO_Port GPIOC
#define M11_Pin GPIO_PIN_0
#define M11_GPIO_Port GPIOD
#define Tem_SCL_Pin GPIO_PIN_6
#define Tem_SCL_GPIO_Port GPIOB
#define Tem_SDA_Pin GPIO_PIN_7
#define Tem_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
