#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"
#include "stdint.h"

#define LED_TASK_PRIO		3     //任务优先级
#define LED_STK_SIZE 		128   //任务堆栈大小

/*左右转向灯*/
#define LIGHT_LEFT_PORT GPIOC
#define LIGHT_LEFT_PIN GPIO_Pin_8
#define LIGHT_LEFT PCout(8) 

#define LIGHT_RIGHT_PORT GPIOC
#define LIGHT_RIGHT_PIN GPIO_Pin_7
#define LIGHT_RIGHT PCout(7) 

//指示灯端口
#define LED_PORT GPIOE
#define LED_PIN GPIO_Pin_10
#define LED PEout(10) 

extern int led_count_;

void ledInit(void);
void ledTask(void* pvParameters);
void turnSignalControl(void);

#endif
