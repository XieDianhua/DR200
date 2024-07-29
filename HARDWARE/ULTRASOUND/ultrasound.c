#include "ultrasound.h"

// ȫ�ֱ������ڴ洢����ֵ
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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ�ܶ˿�ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //ʹ�ܶ˿�ʱ��  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ��TIM5ʱ��

    // ����PC6  PD14��Ϊ���
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		    //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		    //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // ����PD13  PD15��Ϊ���벶��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //�������룬��Ĭ�ϵ�ƽΪ�͵�ƽ
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);

    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

    //��ʼ����ʱ��4 TIM4	 
    TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
    TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//Ԥ��Ƶ��   
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //��ʼ��TIM4���벶�����
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM5_ICInitStructure);

    //�жϷ����ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

    TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC1, ENABLE);//��������ж� ,����CC1IE�����ж�	

    TIM_Cmd(TIM4, ENABLE); 	//ʹ�ܶ�ʱ��5
}

//��ʱ��4�жϷ������	 
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
    {
        // ��ȡ����ֵ
        pulse_width = TIM_GetCapture1(TIM4);
        printf("������=%d", pulse_width);
        // ����жϱ�־λ
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    }
}

void Ultrasonic_Rang(void)
{
    u16 Time_Echo_us = 0;
    u16 Len_mm = 0;

    //ͨ��Trig�������壬����US-100���
    GPIO_SetBits(GPIOC, GPIO_Pin_6);
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    //����������Ϊ50us��>10us��
    delay_us(50);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);

    Time_Echo_us = pulse_width;
    // printf("������=%d", pulse_width);
    if ((Time_Echo_us < 60000) && (Time_Echo_us > 1))
    {
        Len_mm = (Time_Echo_us * 34 / 100) / 2;//ͨ�������ȼ������
        printf("����Ϊ=%d", Len_mm);
    }
    delay_ms(1000);//ÿ�����һ��
}
