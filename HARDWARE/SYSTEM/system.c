#include "system.h"

void BSP_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	delay_init();					//��ʼ����ʱ����
	relayInit();
	ledInit();                     //��ʼ����LED���ӵ�Ӳ���ӿ�
	USART1_Init(115200);	        //���ڳ�ʼ��Ϊ��ͨ�Ĵ��ڣ���ӡ������Ϣ
	usart2Init(115200);            //����λ��ͨ�ų�ʼ��������2
	usart5Init(100000);            //����5��ʼ�������ں�ģ����
	Adc_Init();                     //�ɼ���ص�ѹADC���ų�ʼ��	
	canDriverInit();    		 	//�ײ�canЭ���ʼ��
	// Ultrasound_Init(0xffff, 72 - 1);

	while (g_nMotor_config != 1)
	{
		Motor_Init(MOTOR_CAN_ID);
	}
	g_nMotor_config = 1;
}
