#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos �����ļ� */
#include "FreeRTOSConfig.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
/*��������ͷ�ļ�*/
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
/*һЩC�⺯�������ͷ�ļ�*/
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"

// ��е�ṹ����
#define REDUCTION_RATE 1     //��챵���ļ��ٱ�Ϊ1
#define RADIAUS  0.0695f     //����Բ�ε�����̥�뾶����λm
#define WHEELSPACING 0.350f    //�־�

#define PI 3.1415f  //Բ����

/*--------------- ��ʼ��ʱ����ر�־λ-----------------------*/


void BSP_Init(void);

#endif /* __SYSTEM_H */
