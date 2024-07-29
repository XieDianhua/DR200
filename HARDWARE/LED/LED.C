#include "led.h"

int led_count_;//LED闪烁控制
/**************************************************************************
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void ledInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //使能端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能端口时钟
	
	GPIO_InitStructure.GPIO_Pin = LED_PIN;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(LED_PORT, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	GPIO_SetBits(LED_PORT,LED_PIN);
	
	GPIO_InitStructure.GPIO_Pin = LIGHT_LEFT_PIN|LIGHT_RIGHT_PIN;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(LIGHT_LEFT_PORT, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	GPIO_Init(LIGHT_RIGHT_PORT, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	LIGHT_LEFT=1;
	LIGHT_RIGHT=1;
}

/**************************************************************************
函数功能：LED灯闪烁任务
入口参数：无 
返回  值：无
**************************************************************************/
void ledTask(void *pvParameters)
{
    while(1)
    {
		LED = ~LED;        //LED状态取反，0是点亮，1是熄灭
		turnSignalControl();
		vTaskDelay(led_count_); //相对延时函数，500MS改变一次状态
    }
}
void turnSignalControl(void)
{
	// 转向灯光控制
	if (robot_z_ > 0.3)//左转为正
	{
		LIGHT_LEFT = ~LIGHT_LEFT;
		vTaskDelay(300);
	}
	else if (robot_z_ < -0.3)
	{
		LIGHT_RIGHT = ~LIGHT_RIGHT;
		vTaskDelay(300);
	}
	else
	{
		LIGHT_LEFT = 0;
		LIGHT_RIGHT = 0;
	}
}

