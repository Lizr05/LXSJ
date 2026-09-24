#include "sim800c.h"

uint8_t SIM_Buff[SIM_SIZE]={0};
char sim[128];

void sim800c_init(void)
{
	memset(SIM_Buff, 0, SIM_SIZE);
    memset(sim, 0, 128);
    sprintf(sim,"ATD%s;\r\n", sim_number);
	HAL_UART_Transmit(&huart2, (uint8_t*)sim, strlen(sim), 100);
}