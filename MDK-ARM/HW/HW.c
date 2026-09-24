#include "HW.h"

//PD3:DATA3
//PD4:DATA4
uint8_t LR=0;
uint8_t Way_detect_broadcast=0;

void Tof_Init(void){
	HAL_GPIO_WritePin (GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);//L1---1
	HAL_GPIO_WritePin (GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);//L2---2
	HAL_GPIO_WritePin (GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);//L3---3
	HAL_GPIO_WritePin (GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);//R1---4
	HAL_GPIO_WritePin (GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);//R2---5
	HAL_GPIO_WritePin (GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);//R3---6
	
	HAL_GPIO_WritePin (GPIOC,GPIO_PIN_13,GPIO_PIN_SET);//R3---6
	HAL_Delay(200);
	if (!VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR1, true))
	{
			printf("Failed to detect and initialize sensor!");
			while (1) {}
	}
	VL53L0X_setAddress(VL53L0X_DEFAULT_I2C_ADDR1, VL53L0X_DEFAULT_I2C_ADDR6);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10, GPIO_PIN_SET);//R2---5
	HAL_Delay(200);  
	if (!VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR1, true))
	{
			printf("Failed to detect and initialize sensor!");
			while (1) {} 
	}
	VL53L0X_setAddress(VL53L0X_DEFAULT_I2C_ADDR1, VL53L0X_DEFAULT_I2C_ADDR5);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5, GPIO_PIN_SET);//R1---4
	HAL_Delay(200);  
	if (!VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR1, true))
	{
			printf("Failed to detect and initialize sensor!");
			while (1) {} 
	}
	VL53L0X_setAddress(VL53L0X_DEFAULT_I2C_ADDR1, VL53L0X_DEFAULT_I2C_ADDR4);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4, GPIO_PIN_SET);//L3---3
	HAL_Delay(200);  
	if (!VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR1, true))
	{
			printf("Failed to detect and initialize sensor!");
			while (1) {} 
	}
	VL53L0X_setAddress(VL53L0X_DEFAULT_I2C_ADDR1, VL53L0X_DEFAULT_I2C_ADDR3);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);//L2---2
	HAL_Delay(200);  
	if (!VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR1, true))
	{
			printf("Failed to detect and initialize sensor!");
			while (1) {} 
	}
	VL53L0X_setAddress(VL53L0X_DEFAULT_I2C_ADDR1, VL53L0X_DEFAULT_I2C_ADDR2);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2, GPIO_PIN_SET);//L1---1
	HAL_Delay(200);  
	if (!VL53L0X_Init(VL53L0X_DEFAULT_I2C_ADDR1, true))
	{
			printf("Failed to detect and initialize sensor!");
			while (1) {} 
	}
}

void Tof_Data(void){
	//测距
	L1=VL53L0X_readRangeSingleMillimeters(VL53L0X_DEFAULT_I2C_ADDR1);
	L2=VL53L0X_readRangeSingleMillimeters(VL53L0X_DEFAULT_I2C_ADDR2);
	L3=VL53L0X_readRangeSingleMillimeters(VL53L0X_DEFAULT_I2C_ADDR3);
	R1=VL53L0X_readRangeSingleMillimeters(VL53L0X_DEFAULT_I2C_ADDR4);
	R2=VL53L0X_readRangeSingleMillimeters(VL53L0X_DEFAULT_I2C_ADDR5);
	R3=VL53L0X_readRangeSingleMillimeters(VL53L0X_DEFAULT_I2C_ADDR6);
	
	//阈值判断
	if(L1>30&&L1<250)L1_=1;
	else L1_=0;
	if(L2<230)L2_=1;
	else L2_=0;
	if(L3<250)L3_=1;
	else L3_=0;
	if(R1>30&&R1<250)R1_=1;
	else R1_=0;
	if(R2<230)R2_=1;
	else R2_=0;
	if(R3<250)R3_=1;
	else R3_=0;

	//左侧更新最小值
	if(L==1&&L3_==1){
		if (L3 < min_L3) {
			min_L3 = L3;
			L3__ = 0;
		}
		else if (L3 > (min_L3 + 10)) {
			L3__ = 1;
		} 
		else {
			L3__ = 0;
		}
	}
	else{
		L3__=0;
		min_L3=150;
	}
	//右侧更新最小值
	if(R==1&&R3_==1){
		if (R3 < min_R3) {
			min_R3 = R3;
			R3__ = 0;
		}
		else if (R3 > (min_R3 + 10)) {
			R3__ = 1;
		} 
		else {
			R3__ = 0;
		}
	}
	else{
		R3__=0;
		min_R3=150;
	}
}

void Obstacle_avoid(void)
{
	Tof_Data();
	//先判断是障碍物方向，决定从哪里绕行
    if(Data==0&&rescue==0)
    {
        if(L1_==0&&R1_==0){					//清除障碍物播报
			LR=0;
        }
        
		//左侧有障碍物
        if(L1_==1&&R1_==0&&hc==0){													
            if(data==0x0B){																		//语音执行右绕行				
                L=1;
                stop=0;
                Moter_Right__();
				LED_RIGHT_ON();
                HAL_UART_Transmit(&huart1, mv_close, 1, 200); 
            }
            else if(stop==0){											//否则停在原地
                LR++;
				if(LR>=3){
					stop=1;
					Moter_Stop();
					ESP8266_Send_ESP32("1",100);
					Voice_Broadcast(0x0F);
					cnt_start_l=1;
				}
            }   
        }
		if(L==1&&hc==0){
			if(L3__==1){	//后灯到最小值，停车并pid修正,然后前进(p_L3_==0&&p_R3_==0)&&(L3_==1&&R3_==0)&&
				go_pid1=1;
				Moter_Stop();
				L=2;
			}
			if(L1_==0&&R1_==0&&hc==0){
				LED_OFF();
				Moter_Go();
				Ahead_irSensor=2;
			}
		}	
		
		else if(L==4&&hc==0){
			if(L2_==1&&R2_==0){		//中灯亮，记录L2并与墙面并行
				Target_Distance = L2;
				L=5;
				L3__=0;
				Ahead_irSensor=2;//---------------------------------------------------
			}
			else if(L2_==0&&R2_==0){//否者直接转向去寻找盲道
				Moter_Stop();
				L=6;
				Ahead_irSensor=2;//---------------------------------------------------
			}
		}
		
		else if(L==5&&hc==0){												//刚与墙面平行
			if(p_L2_==1 && p_R2_==0 && L2_==0 && R2_==0){  // 中灯灭，停车并pid修正
				go_pid2 = 1;
				Moter_Stop();
				L = 6;
			}
			else{
				double current_distance = L2;  						// 获取当前距离
				double error = current_distance - Target_Distance;  // 计算误差

				// PID控制(这里只用了PD)
				double derivative = error - previous_error;
				double correction = 0.02 * error + 0.005 * derivative;//0.025
				if(correction>1)correction=1;//远
				if(correction<-1)correction=-1;//近
				previous_error = error;
			
			
				// 根据修正值调整电机速度
				if(fabs(error) > 10){//15
					// 需要修正
					Moter_Control(170 * (1 - correction), 170 * (1 + correction));
				}
				else {  // 在允许误差范围内，直行
					LED_OFF();
					Moter_Go();
				}
			}
			Ahead_irSensor=2;//---------------------------------------------------
		}
		
		//右侧有障碍物
        if(L1_==0&&R1_==1&&hc==0){													
            if(data==0x0A){																		//语音执行右绕行				
                R=1;
                stop=0;
                Moter_Left__();
				LED_RIGHT_ON();
                HAL_UART_Transmit(&huart1, mv_close, 1, 200); 
            }
            else if(stop==0){											//否则停在原地
                LR++;
				if(LR>=3){
					stop=1;
					Moter_Stop();
					ESP8266_Send_ESP32("2",100);
					Voice_Broadcast(0x10);
					cnt_start_r=1;
				}
            }   
        }
		if(R==1&&hc==0){
			if(R3__==1){	//后灯到最小值，停车并pid修正,然后前进(p_L3_==0&&p_R3_==0)&&(L3_==1&&R3_==0)&&
				go_pid1=1;
				Moter_Stop();
				R=2;
			}
			if(L1_==0&&R1_==0&&hc==0){
				LED_OFF();
				Moter_Go();
				Ahead_irSensor=2;
			}
		}	
		
		else if(R==4&&hc==0){
			if(L2_==0&&R2_==1){		//中灯亮，记录L2并与墙面并行
				Target_Distance = R2;
				R=5;
				R3__=0;
				Ahead_irSensor=2;//---------------------------------------------------
			}
			else if(L2_==0&&R2_==0){//否者直接转向去寻找盲道
				Moter_Stop();
				R=6;
				Ahead_irSensor=2;//---------------------------------------------------
			}
		}
		
		else if(R==5&&hc==0){												//刚与墙面平行
			if(p_L2_==0 && p_R2_==1 && L2_==0 && R2_==0){  // 中灯灭，停车并pid修正
				go_pid2 = 1;
				Moter_Stop();
				R = 6;
			}
			else{
				double current_distance = R2;  						// 获取当前距离
				double error = current_distance - Target_Distance;  // 计算误差
        
				// PID控制(这里只用了PD)
				double derivative = error - previous_error;
				double correction = 0.025 * error + 0.005 * derivative;
				if(correction>1)correction=1;//远
				if(correction<-1)correction=-1;//近
				previous_error = error;
			
			
				// 根据修正值调整电机速度
				if(fabs(error) > 10){
					// 需要修正
					Moter_Control(170 * (1 - correction), 170 * (1 + correction));
				}
				else{												//前灯灭，前进,在允许误差范围内，直行(L1_==0&&R1_==0&&hc==0)
					LED_OFF();
					Moter_Go();
				}
			}
			Ahead_irSensor=2;//---------------------------------------------------
		}		
        
        p_L3_=L3_;    
		p_R3_=R3_;  
        p_L2_=L2_;
		p_R2_=R2_;
    }
}

void Way_detect(void)
{
	if(On_Line==1){//自动抬头看信号灯
		Green=0;
        Red=0;
		HAL_UART_Transmit(&huart1, mv_observe, 1, 200); 
		On_Line=5;
	}
	if(Green==1&&data==0x01)							//若斑马线绿灯行，则不逼停&&(On_Line==5||On_Line==1||Right_Line==1||Right_Line==2||Right_Line==3||Right_Line==5)
	{
		Ahead_irSensor=2;								//启动红外对管计时，亮1.5秒后开启红外对管
		Right_Line=0;									//检测到绿灯就清除右边有斑马线的标志位
		Green=0;
	}
    else if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14)&&Ahead_irSensor==1&&Ahead_irSensor_Time==0&&stop==0&&Data==0&&rescue==0)	//离开盲道，且未启动红外对管计时，未停车，开蓝牙，开救援
    {																//逼退并检测前方路况
			data=0x02;
			Moter_Back(); 
			Ahead_irSensor_Time=1;
			if(up==1&&Way_detect_broadcast==0){
				ESP8266_Send_ESP32("F",100);
				Voice_Broadcast(0x1C);
				Way_detect_broadcast=1;
			}
			else if(Way_detect_broadcast==0){
				ESP8266_Send_ESP32("3",100);
				Voice_Broadcast(0x05);
				Way_detect_broadcast=1;
			}
			//清标志位
            Left_Way=0;
            if(Right_Way!=2)Right_Way=0;																		//pid转向过程中不置零
            On_Nway=0;
            Left_Nway=0;
            Right_Nway=0;
            On_Stairs=0;
            Left_Stairs=0;
            Right_Stairs=0;
			On_Line=0;
			Left_Line=0;
			if(Right_Line!=2)Right_Line=0;
            Green=0;
            Red=0;
            LED_OFF();
    }
    else if(cnt4>=3&&Ahead_irSensor_Time==1)																	//停车
    {
        HAL_UART_Transmit(&huart1, mv_observe_way, 1, 200); 
        Observe=1;
        data=0x03; 
        Moter_Stop();
        stop=1;
        LED_OFF();
        cnt4=0;
        Ahead_irSensor_Time=0;
        Way_detect_broadcast=0;
    }
}



