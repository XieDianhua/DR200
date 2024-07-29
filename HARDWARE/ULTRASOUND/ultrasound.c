#include "ultrasound.h"

// 全局变量用于存储捕获值
u8 pulse_width = 0;

//Trig1 PC6 
//Econ  PD15
//Trig2 PD14
//Econ  PD13
void Ultrasound_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef  TIM5_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能端口时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //使能端口时钟  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能TIM5时钟

    // 配置PC6  PD14作为输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		    //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		    //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 配置PD13  PD15作为输入捕获
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //下拉输入，即默认电平为低电平
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);

    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

    //初始化定时器4 TIM4	 
    TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
    TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//预分频器   
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM4输入捕获参数
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM5_ICInitStructure);

    //中断分组初始化
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

    TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC1, ENABLE);//允许更新中断 ,允许CC1IE捕获中断	

    TIM_Cmd(TIM4, ENABLE); 	//使能定时器5
}

//定时器4中断服务程序	 
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
    {
        // 获取捕获值
        pulse_width = TIM_GetCapture1(TIM4);
        printf("脉冲宽度=%d", pulse_width);
        // 清除中断标志位
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    }
}

void Ultrasonic_Rang(void)
{
    u16 Time_Echo_us = 0;
    u16 Len_mm = 0;

    //通过Trig发送脉冲，触发US-100测距
    GPIO_SetBits(GPIOC, GPIO_Pin_6);
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    //设置脉冲宽度为50us（>10us）
    delay_us(50);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);

    Time_Echo_us = pulse_width;
    // printf("脉冲宽度=%d", pulse_width);
    if ((Time_Echo_us < 60000) && (Time_Echo_us > 1))
    {
        Len_mm = (Time_Echo_us * 34 / 100) / 2;//通过脉冲宽度计算距离
        printf("距离为=%d", Len_mm);
    }
    delay_ms(1000);//每秒测量一次
}
