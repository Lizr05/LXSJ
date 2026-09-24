#include "jy901s.h"


/*定义发送数据的帧头*/

extern tPid pid_1;

/*定义结构体变量*/
SAcc jy901_Acc = {0,0,0,0};
SGyro jy901_Gyro = {0,0,0,0};
SAngle jy901_Angle = {0,0,0,0};
SMag jy901_Mag = {0,0,0,0};
SParity Parity = {0,0,0,0};
/*JY901S数据处理函数*/
void vJY901S_Data_Proc(void){
	if(receive_complete_flag == 1){
		receive_complete_flag = 0;
		/*校验*/
//		Parity.Acc_parity_bit = (receive_buff[0] + receive_buff[1] + receive_buff[2] + receive_buff[3] + receive_buff[4] + receive_buff[5]+ receive_buff[6] + receive_buff[7] + receive_buff[8] + receive_buff[9]) & 0x00ff;
//		Parity.Gyro_parity_bit = (receive_buff[11] + receive_buff[12] + receive_buff[13] + receive_buff[14] + receive_buff[15] + receive_buff[16]+ receive_buff[17] + receive_buff[18] + receive_buff[19] + receive_buff[20]) & 0x00ff;
		Parity.Angle_parity_bit = (receive_buff[22] + receive_buff[23] + receive_buff[24] + receive_buff[25] + receive_buff[26] + receive_buff[27]+ receive_buff[28] + receive_buff[29] + receive_buff[30] + receive_buff[31]) & 0x00ff;
//		Parity.Mag_parity_bit = (receive_buff[33] + receive_buff[34] + receive_buff[35] + receive_buff[36] + receive_buff[37] + receive_buff[38]+ receive_buff[39] + receive_buff[40] + receive_buff[41] + receive_buff[42]) & 0x00ff;

		/*如果校验正确就读取数据*/
//		if(Parity.Acc_parity_bit == receive_buff[10]){
//			jy901_Acc.a[0] = (short)(((short)receive_buff[3] << 8) | receive_buff[2]) / 32768.0 * 16.0 * 9.8;/*加速度x*/
//			jy901_Acc.a[1] = (short)(((short)receive_buff[5] << 8) | receive_buff[4]) / 32768.0 * 16.0 * 9.8;/*加速度y*/
//			jy901_Acc.a[2] = (short)(((short)receive_buff[7] << 8) | receive_buff[6]) / 32768.0 * 16.0 * 9.8;/*加速度z*/
//			jy901_Acc.T = ((receive_buff[9] << 8) | receive_buff[8]) / 100.0;								   /*温度*/
//		}
//		if(Parity.Gyro_parity_bit == receive_buff[21]){
//			jy901_Gyro.w[0] = (short)(((short)receive_buff[14] << 8) | receive_buff[13]) / 32768.0 * 2000.0;/*角速度x*/
//			jy901_Gyro.w[1] = (short)(((short)receive_buff[16] << 8) | receive_buff[15]) / 32768.0 * 2000.0;/*角速度y*/
//			jy901_Gyro.w[2] = (short)(((short)receive_buff[18] << 8) | receive_buff[17]) / 32768.0 * 2000.0;/*角速度z*/
//		}
		if(Parity.Angle_parity_bit == receive_buff[32]){
            jy901_Angle.Angle[0] = (short)(((short)receive_buff[25] << 8) | receive_buff[24]) / 32768.0 * 180.0;/*角度Roll*/
			jy901_Angle.Angle[1] = (short)(((short)receive_buff[27] << 8) | receive_buff[26]) / 32768.0 * 180.0;/*角度Pitch*/
			jy901_Angle.Angle[2] = (short)(((short)receive_buff[29] << 8) | receive_buff[28]) / 32768.0 * 180.0;/*角度Yaw*/
            jy901_Angle.Version = (short)(((short)receive_buff[31] << 8) | receive_buff[30]);                   /*版本号version*/
		}
//		if(Parity.Mag_parity_bit == receive_buff[43]){
//			jy901_Mag.h[0] = (short)(((short)receive_buff[36] << 8) | receive_buff[35]) * 0.00833;/*磁场x*/
//			jy901_Mag.h[1] = (short)(((short)receive_buff[38] << 8) | receive_buff[37]) * 0.00833;/*磁场y*/
//			jy901_Mag.h[2] = (short)(((short)receive_buff[40] << 8) | receive_buff[39]) * 0.00833;/*磁场z*/
//		}
		memset(receive_buff,0,USART_RECEIVE_LENGTH);
	}
}
/*写指令函数*/
void vJY901S_Send_Command(uint8_t ADDR,uint8_t DATAL,uint8_t DATAH)
{
	HAL_UART_Transmit(&huart7,(uint8_t*)&frame1,1,100); 						//帧头1
	HAL_UART_Transmit(&huart7,(uint8_t*)&frame2,1,100);							//帧头2
	HAL_UART_Transmit(&huart7,(uint8_t*)&ADDR,1,100);							//寄存器地址
	HAL_UART_Transmit(&huart7,(uint8_t*)&DATAL,1,100);							//数据第低八位
	HAL_UART_Transmit(&huart7,(uint8_t*)&DATAH,1,100);							//数据高八位
}


void Control_jy901s(void)
{
    vJY901S_Data_Proc();																
    if(cnt > 10)cnt=0;   
    if(cnt2>10)								//延时右转
    {
        previous = jy901_Angle.Angle[2]; 
        setpoint = previous-90;
        data=0x04;
        Pid_init();
        if(Right_Way==1){Right_Way=2;cnt_start_go=1;}		//右边可通行
        if(Right_Line==1){Right_Line=2;cnt_start_goline=1;}	//右边有斑马线
        cnt2=0;
    }
    if(R==2)//向左绕行
    {
		Target_trend=jy901_Angle.Angle[2]-Initial_trend;
		if(Target_trend<0)Target_trend+=360;
        previous = jy901_Angle.Angle[2]; 
        setpoint = previous-Target_trend;
        data=0x04;
        R=3;
        Pid_init_2();
    }
	else if(R==6){
		go_pid2=1;
        previous = jy901_Angle.Angle[2];
        setpoint = previous-Target_trend-(previous-Initial_trend)-5;
        data=0x04;
        R=7;
		Pid_init_2();
	}
    if(L==2)//向右绕行
    {
		Target_trend=Initial_trend-jy901_Angle.Angle[2];
		if(Target_trend<0)Target_trend+=360;
        previous = jy901_Angle.Angle[2]; 
        setpoint = previous+Target_trend;
        data=0x05;
        L=3;
        Pid_init_2();
    }
	else if(L==6){
		go_pid2=1;
        previous = jy901_Angle.Angle[2]; 
        setpoint = previous+Target_trend+(Initial_trend-previous)+5;
        data=0x05;
        L=7;
		Pid_init_2();
	}
//pid右转
    if(data==0x04)
    { 
        LED_RIGHT_ON();
        if(setpoint<-180)    {setpoint+=360;}
        if(setpoint>180)     {setpoint-=360;}
        if(setpoint-15<=jy901_Angle.Angle[2]&&setpoint+15>=jy901_Angle.Angle[2]&&Right_Way==2)			//右边可通行，前进
        {
			cnt_start_go=0;
			cnt_start_go_=0;
            Moter_Go();
            LED_OFF();
            HAL_UART_Transmit(&huart1, mv_go, 1, 200);
            stop=0;
            Right_Way=0;
            data=0x01;
			R=0;
			L=0;
        }
        else if(setpoint-25<=jy901_Angle.Angle[2]&&setpoint+25>=jy901_Angle.Angle[2]&&Right_Line==2)	//右边有斑马线，停车
        {
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
        else if(setpoint-5<=jy901_Angle.Angle[2]&&setpoint+5>=jy901_Angle.Angle[2]&&R==3)				//左绕行，前进
        {
			go_pid1=0;
			go_pid1_=0;
			R=4;
            Moter_Go(); 
            LED_OFF();
            if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
            stop=0;
            data=0x01;
            Pid_init();
			Ahead_irSensor=2;
        }
		else if(setpoint-5<=jy901_Angle.Angle[2]&&setpoint+5>=jy901_Angle.Angle[2]&&R==7)					//左绕行2，前进
        {
			go_pid2=0;
			go_pid2_=0;
			ESP8266_Send_ESP32("15",100);
			R=0;
            Moter_Go(); 
            LED_OFF();
            if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
            stop=0;
            data=0x01;
        }
		else if(setpoint-25<=jy901_Angle.Angle[2]&&setpoint+25>=jy901_Angle.Angle[2]&&vioce_turn==1){	//语音命令
			Moter_Go();
            LED_OFF();
            HAL_UART_Transmit(&huart1, mv_go, 1, 200);
            stop=0;  
            data=0x01;
			vioce_turn=0;
			R=0;
			L=0;
		}	
        else																							//pid修正
        {
            HAL_UART_Transmit(&huart1, mv_go, 1, 200); 
            HAL_UART_Transmit(&huart3, mv_go, 1, 200); 
            PidOut1 = Pid_Compute(&pid_1,setpoint,jy901_Angle.Angle[2]);
            if(PidOut1>0){PID_Moter_Left(PidOut1);}
            if(PidOut1<0){PID_Moter_Right(-PidOut1);}
			if(R==3&&L2<120){
				go_pid1=0;
				go_pid1_=0;
				R=4;
				Moter_Go(); 
				LED_OFF();
				if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
				stop=0;
				data=0x01;
				Ahead_irSensor=2;
			}
        }
        
    }
//pid左转
    else if(data==0x05)
    {
		if(setpoint<-180)    {setpoint+=360;}
        if(setpoint>180)     {setpoint-=360;}
        LED_LEFT_ON();
        if(setpoint-5<=jy901_Angle.Angle[2]&&setpoint+5>=jy901_Angle.Angle[2]&&L==3)					//右绕行1，前进
        {
			go_pid1=0;
			go_pid1_=0;
			L=4;
            Moter_Go(); 
            LED_OFF();
            if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
            stop=0;
            data=0x01;
			Ahead_irSensor=2;
        }
		else if(setpoint-5<=jy901_Angle.Angle[2]&&setpoint+5>=jy901_Angle.Angle[2]&&L==7)					//右绕行2，前进
        {
			go_pid2=0;
			go_pid2_=0;
			ESP8266_Send_ESP32("15",100);
			L=0;
            Moter_Go(); 
            LED_OFF();
            if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
            stop=0;
            data=0x01;
        }
		else if(setpoint-25<=jy901_Angle.Angle[2]&&setpoint+25>=jy901_Angle.Angle[2]&&vioce_turn==1){	//语音命令
			Moter_Go();
            LED_OFF();
            HAL_UART_Transmit(&huart1, mv_go, 1, 200);
            stop=0;  
            data=0x01;
			vioce_turn=0;
			R=0;
			L=0;
		}				
        else																							//pid修正
        {
            HAL_UART_Transmit(&huart1, mv_go, 1, 200); 
            HAL_UART_Transmit(&huart3, mv_go, 1, 200); 
            PidOut1 = Pid_Compute(&pid_1,setpoint,jy901_Angle.Angle[2]);
            if(PidOut1>0){PID_Moter_Left(PidOut1);}
            if(PidOut1<0){PID_Moter_Right(-PidOut1);}
			if(L==3&&L2<100){
				go_pid1=0;
				go_pid1_=0;
				L=4;
				Moter_Go(); 
				LED_OFF();
				if(Ahead_irSensor==1)HAL_UART_Transmit(&huart1, mv_open, 1, 200); 
				stop=0;
				data=0x01;
				Ahead_irSensor=2;
			}
        }
    }
//坡道检测
    else if(data==0x01&&stop==0&&(data!=0x6C||data!=0x72)&&L==0)													//在前进且不偏航状态下检测坡道调速
    {
        if(jy901_Angle.Angle[1]>=6)																		//上坡
        {
            Moter_Go_Up();
			up=1;
            if(voice_broadcast==1){
				ESP8266_Send_ESP32("13",100);
                Voice_Broadcast(0x0C);
                voice_broadcast=0;
            }
        }
        else if(jy901_Angle.Angle[1]<=-6){																//下坡
            Moter_Go_(100);
			up=0;
            if(voice_broadcast==1){
				ESP8266_Send_ESP32("14",100);
                Voice_Broadcast(0x1E);
                voice_broadcast=0;
            }
        }
        else
        {
            Moter_Go();
            voice_broadcast=1;
        }
    }
//    if(hc==1&&jy901_Angle.Angle[1]>=8)
//    {
//        Moter_Stop_Ramp();
//        data=0x01;
//    }
//    else if(hc==1&&jy901_Angle.Angle[1]<=8)
//    {
//        hc=0;
//        stop=1;        
//        Moter_Stop();
//    }
    
}
