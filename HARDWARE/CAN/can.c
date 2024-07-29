#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "usartx.h"

int g_nMotor_config = -1;                           //�������Ƿ�����
union URcv_Vel_Data uVelLeft;
union URcv_Vel_Data uVelRight;                      //�������ʵʱ�ٶȵı���
int g_nReInitMotor = 0;                             //�����������ϵ��������δ�ϵ磬���³�ʼ��������������,Ϊ1������Ҫ���³�ʼ����Ϊ0�������Ҫ���³�ʼ��
int g_nHeart_count = 0;                             //���������������ڹ涨������û��������������Ϊ�ӻ�����,����״̬����ΪSTATE_STOP
int g_nHeart_Lastcount = 0;                         //��¼�ϴν����������ļ���ֵ
enum ENUM_HEART_STAT eMotor_Heart = STATE_UNKNOW;
enum ENUM_ERROR_STATE eMotor_Error = ERROR_NONE;    // ���������ϼ�¼
union URcv_ERROR_Data uError;                       // �����������Ĺ���״̬

int g_nState_debug = -1;                            // ���������״̬����

/**********************************************************
* �������ܣ� stm32�ײ��canͨ��Э���ʼ����
* ������     ��
* ˵����     ��
**********************************************************/
void canDriverInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

#if CAN_RX0_INT_ENABLE 
	NVIC_InitTypeDef  		NVIC_InitStructure;
#endif

	//��Ӧ��GPIO�ں�ʱ�ӳ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //ʹ��CANʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //ʹ��AFIOʱ��

	//��Ҫ��CAN���ܸ��ã�ӳ�䵽PD0��PD1��
	GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);

	//��ʼ���õ���GPIO��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  //PD0��ΪRx 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //��������ģʽ
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  //PD1��ΪTx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������,ʹ�ø���CAN����
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//����ģʽ�������ʳ�ʼ��
	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM = DISABLE; //��ʱ�����ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM = DISABLE; //����Զ����߹���   
	CAN_InitStructure.CAN_AWUM = DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART = ENABLE;  //ʹ�ñ����Զ�����
	CAN_InitStructure.CAN_RFLM = DISABLE; //���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP = DISABLE; //���ȼ��ɱ��ı�ʶ������
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;    //ģʽ����
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;  //
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 12;
	CAN_Init(CAN1, &CAN_InitStructure);

	//����CANɸѡ��
	//���ù�����
	CAN_FilterInitStructure.CAN_FilterNumber = 0; //������0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //�����ڱ�ʶ������ģʽ��
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32λ
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;////32λID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //ʹ�ܹ�����0
	CAN_FilterInit(&CAN_FilterInitStructure);//��������ʼ��

	//����CAN�ж�
#if CAN_RX0_INT_ENABLE 

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);				//FIFO0��Ϣ�Һ��ж�����.	���յ����ݾͻ�����ж�    
	//CAN_ClearFlag(CAN1,CAN_IT_FMP0);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif	
}

/**********************************************************
* �������ܣ� can�жϷ�����
* ������     ��
* ˵����     ���жϺ����л�ȡ��ʵʱ�������ٶ�
**********************************************************/
#if CAN_RX0_INT_ENABLE	//ʹ��RX0�жϺ���
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;

	CAN_Receive(CAN1, 0, &RxMessage);
	/*- - - - - - - - - - - - -  - - - �ӻ����߻��߽��մӻ������������Ƿ���Ҫ���³�ʼ�� - - - - - - - - - - - - - - - */
		//��󷽵��
	if (RxMessage.StdId == 0x704)  //����������������ж�
	{
		g_nState_debug = RxMessage.Data[0];
		g_nHeart_count++;  //����������

		/*--------������״̬����---------------*/
		if (RxMessage.Data[0] == 0x04)
		{
			eMotor_Heart = STATE_STOP;
		}
		else if (RxMessage.Data[0] == 0x05)
		{
			g_nMotor_config = 1;
			eMotor_Heart = STATE_START;
		}
		else if (RxMessage.Data[0] == 0x7F)
		{
			eMotor_Heart = STATE_PRE_OPERATE;
		}

		/*--------������;���ֵ���Ҫ�����������³�ʼ�������---------------*/
		//���ư����ϵ磬������ϵ磬��Ҫ���³�ʼ��һ�ε��
		if (eMotor_Heart == STATE_UNKNOW && g_nMotor_config == -1)
		{
			g_nReInitMotor = 1;
		}
		//��;�������ϵ磬Ȼ���ٴ������ϵ�
		if (g_nMotor_config == 0)
		{
			//��������δ�ϵ磬�����������ϵ������
			g_nReInitMotor = 1;
		}
	}

	/*- - - - - - - - - - - - -  - - - TPDO�����ٶ�- - - - - - - - - - - - - - - */
	/*- - - - ��ȡ�����ٶ�˵������ȡ�����ٶȵõ�λ��0.1r/min,���Ƿ���������趨���෴ - - - -*/
		//������ٶȷ�����TPDO0���õ�
	else if (RxMessage.StdId == 0x184)
	{
		//ת��ʵʱ�����ٶȣ��ٶ�ֻ������ĵ���λ
		uVelLeft.ucData[0] = RxMessage.Data[0];
		uVelLeft.ucData[1] = RxMessage.Data[1];
		uVelLeft.ucData[2] = RxMessage.Data[2];
		uVelLeft.ucData[3] = RxMessage.Data[3];

	}
	//�Ҳ����ٶȷ�����TPDO0���õ�
	else if (RxMessage.StdId == 0x284)
	{
		//ת��ʵʱ�����ٶ�
		uVelRight.ucData[0] = RxMessage.Data[0];
		uVelRight.ucData[1] = RxMessage.Data[1];
		uVelRight.ucData[2] = RxMessage.Data[2];
		uVelRight.ucData[3] = RxMessage.Data[3];
	}


	/*- - - - - - - - - - - - -  - - - ���������Ϸ���(TPDO3) - - - - - - - - - - - - - - - */
  /*- - - - �����������Ĺ��ϣ����ñ�־λ��Ȼ���ں��������н��й������ - - - -*/
	/* �����У���ѹ��Ƿѹ�����������ء������������������ٶȳ���ο���ѹ����EEPROM��д������������ */
	else if (RxMessage.StdId == 0x484)  //TPDO3���õ�
	{
		uError.ucData[0] = RxMessage.Data[0];
		uError.ucData[1] = RxMessage.Data[1];
		if (uError.usError == 0x00)
		{
			eMotor_Error = ERROR_NONE;              // �޴���
		}
		else if (uError.usError == 0x01)
		{
			eMotor_Error = ERROR_OVER_VALUE;        // ��ѹ
		}
		else if (uError.usError == 0x02)
		{
			eMotor_Error = ERROR_LESS_VALUE;        // Ƿѹ
		}
		else if (uError.usError == 0x04)
		{
			eMotor_Error = ERROR_OVER_CURRENT;      // ����
		}
		else if (uError.usError == 0x08)
		{
			eMotor_Error = ERROR_OVER_LOAD;         // ����
		}
		else if (uError.usError == 0x10)
		{
			eMotor_Error = ERROR_OVERDIFF_CURRENT;  // ��������
		}
		else if (uError.usError == 0x20)
		{
			eMotor_Error = ERROR_OVERDIFF_ENCODER;  // ����������
		}
		else if (uError.usError == 0x40)
		{
			eMotor_Error = ERROR_OVERDIFF_VELOCITY; // �ٶȳ���
		}
		else if (uError.usError == 0x80)
		{
			eMotor_Error = ERROR_REF_VALUE;          // �ο���ѹ����
		}
		else if (uError.usError == 0x100)
		{
			eMotor_Error = ERROR_EEPROM;             // EEPROM��д����
		}
		else if (uError.usError == 0x200)
		{
			eMotor_Error = ERROR_HALL;               // ��������
		}
	}

	CAN_ClearFlag(CAN1, CAN_IT_FMP0);

}
#endif

