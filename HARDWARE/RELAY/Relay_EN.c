#include "Relay_EN.h"
#include "delay.h"
/*******************************************************************************
* 函 数 名         : RELAY_EN
* 函数功能		   : 继电器使能函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void relayInit(void)
{
	GPIO_InitTypeDef RELAY_ENStructure1;
	GPIO_InitTypeDef RELAY_ENStructure2;

	RCC_APB2PeriphClockCmd(RELAY_PORT_RCC, ENABLE);

	RELAY_ENStructure1.GPIO_Pin = RELAY_PIN11;
	RELAY_ENStructure1.GPIO_Mode = GPIO_Mode_Out_PP;
	RELAY_ENStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RELAY_PORT, &RELAY_ENStructure1);

	RELAY_ENStructure2.GPIO_Pin = RELAY_PIN12;
	RELAY_ENStructure2.GPIO_Mode = GPIO_Mode_Out_PP;
	RELAY_ENStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RELAY_PORT, &RELAY_ENStructure2);

	GPIO_SetBits(RELAY_PORT, RELAY_PIN11);   //使能RELAY
	delay_ms(500);
	GPIO_SetBits(RELAY_PORT, RELAY_PIN12);   //使能RELAY
}
