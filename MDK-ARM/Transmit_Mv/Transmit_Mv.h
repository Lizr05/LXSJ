#ifndef __TRANSMIT_MV_H
#define __TRANSMIT_MV_H

#include "main.h"

extern float SMBus_ReadTemp(void); 
extern float temp;

void Mv_trans(void);
void Mv_All(void);

#endif 
