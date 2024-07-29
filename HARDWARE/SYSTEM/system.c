#include "system.h"

void BSP_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init();					//初始化延时函数
	relayInit();
	ledInit();                     //初始化与LED连接的硬件接口
	USART1_Init(115200);	        //串口初始化为普通的串口，打印调试信息
	usart2Init(115200);            //上下位机通信初始化，串口2
	usart5Init(100000);            //串口5初始化，用于航模控制
	Adc_Init();                     //采集电池电压ADC引脚初始化	
	canDriverInit();    		 	//底层can协议初始化
	// Ultrasound_Init(0xffff, 72 - 1);

	while (g_nMotor_config != 1)
	{
		Motor_Init(MOTOR_CAN_ID);
	}
	g_nMotor_config = 1;
}
