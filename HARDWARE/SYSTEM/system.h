#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos 配置文件 */
#include "FreeRTOSConfig.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
/*外设的相关头文件*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "MotionControl.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "user_can.h"
#include "ioi2c.h"
#include "show.h"	
#include "ultrasound.h"
/*一些C库函数的相关头文件*/
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"

// 机械结构参数
#define REDUCTION_RATE 1     //轮毂电机的减速比为1
#define RADIAUS  0.0695f     //定义圆形底盘轮胎半径，单位m
#define WHEELSPACING 0.350f    //轮距

#define PI 3.1415f  //圆周率

/*--------------- 初始化时的相关标志位-----------------------*/


void BSP_Init(void);

#endif /* __SYSTEM_H */
