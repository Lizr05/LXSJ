#include "Transmit_Mv.h"

void Mv_All(void)	//定时传播数据
{
    if(cnt_trans>=5&&rescue==1)//
    {
        Mv_trans();
        cnt_trans=0;
    }
}

void Mv_trans(void)	//计算传输数据
{
	//视频流数据传输
    DHT11_Read_Data(&humidity, &temperature, &temperature_xs);
    temp=SMBus_ReadTemp()+3;
	sprintf(str, "%.2f,%.2f,%d,%d\r\n", distance,temp,humidity,rescue);
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str),100);
	
	//绕行状态检测
//	sprintf(str, "%.2f\r\nL1=%d  L2=%d  L3=%d\r\nR1=%d  R2=%d  R3=%d\r\n%d    %d    %.2f    %.2f\r\n",jy901_Angle.Angle[2],L1,L2,L3,R1,R2,R3,L,R,Target_trend,Target_trend+(Initial_trend-previous));
//	HAL_UART_Transmit(&huart6, (uint8_t*)str, strlen(str), 100);
}
