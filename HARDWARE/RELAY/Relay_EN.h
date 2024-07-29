#ifndef _RELAY_EN_H
#define _RELAY_EN_H

#include "system.h"

/*  RELAY使能引脚时钟端口、引脚定义 */
#define RELAY_PORT 				GPIOD   
#define RELAY_PIN11 			GPIO_Pin_11
#define RELAY_PIN12 			GPIO_Pin_12
#define RELAY_PORT_RCC		    RCC_APB2Periph_GPIOD

#define RELAY2_EN PDout(11)
#define RELAY1_EN PDout(12)

void relayInit(void);

#endif
