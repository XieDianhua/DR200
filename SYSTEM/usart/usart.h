#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

void USART1_Init(u32 bound);

#endif


