#include "usartx.h"
#include "user_can.h"

unsigned char remote_flag = 0;              //��ģ������־λ
unsigned char ros_flag = 0;                 // ROS��λ�������־λ
/*--------����λ��ͨ�Ÿ�ʽ����usart2-----------*/
SEND_DATA send_data_;//�������ݵĽṹ��
RECEIVE_DATA receive_data;//�������ݵĽṹ��

/*--------��ģ���ݽ������ȫ�ֱ����Ķ���-----------*/
SBUS_CHANNEL_STRUCT sbus_channel_data_transformed_;
unsigned char usart5_recive_buffer_[MAX_BUFF_LEN] = { 0 };
int control_speed_level_ = 0;       //ǰ������ٶ�                   
int control_director_level_ = 0;       //ת��
unsigned char g_ucLightOnFlag = 0; //���ƿ�����־
float recive_robot_speed_x_ = 0.0;       // ���ڽ��յ����ٶ�����
float recive_robot_speed_y_ = 0.0;
float recive_robot_speed_z_ = 0.0;

/**************************************************************************
�������ܣ������շ��վ�������,����2�������ݵ���λ�����жϽ�������
��ڲ�������
�� �� ֵ����
**************************************************************************/
void communicationTask(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	const TickType_t xPeriod = pdMS_TO_TICKS(50);
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, xPeriod);//��������20Hz��Ƶ������
		parseRemoteData();//����ң��������
		getToSendData(); //��Ҫ���з��͵����ݽ��и�ֵ
		usart2SendData(); //����2(ROS)��������
	}
}
/**************************************************************************
�������ܣ�����2��ʼ������Ϊ����λ��ͨ��
��ڲ�����boundΪͨ�Ų�����
����PCB���еĵڶ���USB�ӿ�
�� �� ֵ����
**************************************************************************/
void usart2Init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);// ��Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��USART2ʱ��
	//USART_TX  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //A2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART_RX	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//A3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//UsartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;//��ռ���ȼ����ж����ȼ����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);     //��ʼ������3
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���3 
}

/**************************************************
* �������ܣ�����5��ʼ����������Ϊ��ģ�źŽ���
* ��    ����unBound����ͨ�Ų����ʣ���Ϊ100k
* �� �� ֵ����
**************************************************/
void usart5Init(unsigned int unBound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);	//ʹ��USART2��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

	//USART5_TX   GPIOC.12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //PC12
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC.12

	//USART5_RX	  GPIOD.2��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PD2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIOD.2  

	//USART2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate = unBound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//�շ�ģʽ

	USART_Init(UART5, &USART_InitStructure); //��ʼ������5
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	//USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	USART_Cmd(UART5, ENABLE);                //ʹ�ܴ���5 
}

/**************************************************
* �������ܣ��źŽ����жϴ�����
* ��ڲ�������
* �� �� ֵ����
**************************************************/
void UART5_IRQHandler()
{
	static int index_count = 0;

	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //�жϲ��� 
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE); //����жϱ�־
		usart5_recive_buffer_[index_count++] = USART_ReceiveData(UART5);     //���մ���5���ݵ�buff������
		if (usart5_recive_buffer_[index_count - 1] == 0x00 || index_count == MAX_BUFF_LEN)    //������յ�β��ʶ�ǻ��з������ߵ�������������������½��գ�
		{
			if (usart5_recive_buffer_[0] == 0x0f)                      //��⵽ͷ��ʶ��������Ҫ�� 
			{
				sbusTransform(usart5_recive_buffer_);
				index_count = 0;
			}
			else
			{
				index_count = 0;                                   //����������Ҫ�����ݻ��ߴﵽ����������ʼ���½���
			}
		}
	}
}

/**************************************************************************
�������ܣ�����2�����ж�,������λ�����͹���������
��ڲ�������
�� �� ֵ����
**************************************************************************/
void USART2_IRQHandler(void)
{
	static unsigned char count = 0;
	unsigned char usart2_receive;

	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�ж��Ƿ���յ�����
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE); //����жϱ�־
		ros_flag = 1;                        // ��ģ������ʱ�򣬸����������ֵΪ0
		if (remote_flag == 0 && ros_flag == 1)//TODO:����Ϊ||�������ڿ���ң������ͬʱ����Ϊ��λ������
		{
			robot_control_mode_ = CONTROL_MODE_ROS;
		}
		usart2_receive = USART_ReceiveData(USART2);//��ȡ����
		receive_data.buffer[count] = usart2_receive;
		if (usart2_receive == FRAME_HEADER || count > 0)
		{
			count++;
		}
		else
		{
			count = 0;
		}
		if (count == 11)	//��֤���ݰ��ĳ���
		{
			count = 0;//���¿�ʼ����
			if (receive_data.buffer[10] == FRAME_TAIL) //��֤���ݰ���β��У����Ϣ
			{
				if (receive_data.buffer[9] == checkSum(9, 0))	 //����У��λ���㣬ģʽ0�Ƿ�������У��
				{
					recive_robot_speed_x_ = parseTargetSpeed(receive_data.buffer[3], receive_data.buffer[4]);
					recive_robot_speed_y_ = parseTargetSpeed(receive_data.buffer[5], receive_data.buffer[6]);
					recive_robot_speed_z_ = parseTargetSpeed(receive_data.buffer[7], receive_data.buffer[8]);
				}
			}
		}
	}
}

/**************************************************************************
�������ܣ�����2(ROS)��������
��ڲ�������
����  ֵ����
**************************************************************************/
void usart2SendData(void)
{
	unsigned char i = 0;
	for (i = 0; i < 24; i++)
	{
		usart2SendByte(send_data_.buffer[i]);
	}
}

/**************************************************************************
�������ܣ�����2���͵��ֽ�
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart2SendByte(unsigned char data)
{
	USART2->DR = data;
	while ((USART2->SR & 0x40) == 0);
}

/**************************************************************************
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void getToSendData(void)
{
	send_data_.SENSOR_STR.frame_header = FRAME_HEADER; //֡ͷ
	send_data_.SENSOR_STR.frame_tail = FRAME_TAIL; //֡β

	//��ȡ�����ٶ�
	getMotorSpeed();

	// �˶�ѧ���� ͨ�������ٶȽ�������ٶ�
	send_data_.SENSOR_STR.car_x = ((motor_left_.line_speed_feedback + motor_right_.line_speed_feedback) / 2) * 1000; //С��x���ٶ�
	send_data_.SENSOR_STR.car_y = 0;
	send_data_.SENSOR_STR.car_z = ((motor_right_.line_speed_feedback - motor_left_.line_speed_feedback) / WHEELSPACING) * 1000;//С��z���ٶ�

	send_data_.SENSOR_STR.battery_voltage = Voltage * 1000; //��ص�ѹ(���ｫ�������Ŵ�һǧ�����䣬��Ӧ���ڽ��ն��ڽ��յ����ݺ�Ҳ����Сһǧ��)

	send_data_.buffer[0] = send_data_.SENSOR_STR.frame_header; //֡ͷ(�̶�ֵ)

	send_data_.buffer[2] = (send_data_.SENSOR_STR.car_x >> 8) & 0XFF; //С��x���ٶ�
	send_data_.buffer[3] = send_data_.SENSOR_STR.car_x & 0XFF;    //С��x���ٶ�
	send_data_.buffer[4] = (send_data_.SENSOR_STR.car_y >> 8) & 0XFF;  //С��y���ٶ�
	send_data_.buffer[5] = send_data_.SENSOR_STR.car_y & 0XFF;     //С��y���ٶ�
	send_data_.buffer[6] = (send_data_.SENSOR_STR.car_z >> 8) & 0XFF; //С��z���ٶ�
	send_data_.buffer[7] = send_data_.SENSOR_STR.car_z & 0XFF;    //С��z���ٶ�

	send_data_.buffer[20] = (send_data_.SENSOR_STR.battery_voltage >> 8) & 0XFF; //��ص�ѹ
	send_data_.buffer[21] = send_data_.SENSOR_STR.battery_voltage & 0XFF; //��ص�ѹ

	send_data_.buffer[22] = checkSum(22, 1); //����У��λ���㣬ģʽ1�Ƿ�������У��
	send_data_.buffer[23] = send_data_.SENSOR_STR.frame_tail;//֡β���̶�ֵ��
}

/**************************************************
* �������ܣ���sbus�ź�ת��Ϊͨ��ֵ
* ��    ����ucBufΪ���յ��Ĵ�������
* �� �� ֵ��0����ɹ���1����ʧ��
**************************************************/
unsigned char sbusTransform(unsigned char* ucBuf)
{
	if (ucBuf[23] == 0)
	{
		sbus_channel_data_transformed_.remote_receiver_connect_state = 1;
		sbus_channel_data_transformed_.channel_1 = ((int16_t)ucBuf[1] >> 0 | ((int16_t)ucBuf[2] << 8)) & 0x07FF;
		sbus_channel_data_transformed_.channel_2 = ((int16_t)ucBuf[2] >> 3 | ((int16_t)ucBuf[3] << 5)) & 0x07FF;
		sbus_channel_data_transformed_.channel_3 = ((int16_t)ucBuf[3] >> 6 | ((int16_t)ucBuf[4] << 2) | (int16_t)ucBuf[5] << 10) & 0x07FF;
		sbus_channel_data_transformed_.channel_4 = ((int16_t)ucBuf[5] >> 1 | ((int16_t)ucBuf[6] << 7)) & 0x07FF;
		sbus_channel_data_transformed_.channel_5 = ((int16_t)ucBuf[6] >> 4 | ((int16_t)ucBuf[7] << 4)) & 0x07FF;
		sbus_channel_data_transformed_.channel_6 = ((int16_t)ucBuf[7] >> 7 | ((int16_t)ucBuf[8] << 1) | (int16_t)ucBuf[9] << 9) & 0x07FF;
		sbus_channel_data_transformed_.channel_7 = ((int16_t)ucBuf[9] >> 2 | ((int16_t)ucBuf[10] << 6)) & 0x07FF;
		sbus_channel_data_transformed_.channel_8 = ((int16_t)ucBuf[10] >> 5 | ((int16_t)ucBuf[11] << 3)) & 0x07FF;
		sbus_channel_data_transformed_.channel_9 = ((int16_t)ucBuf[12] << 0 | ((int16_t)ucBuf[13] << 8)) & 0x07FF;
		sbus_channel_data_transformed_.channel_10 = ((int16_t)ucBuf[13] >> 3 | ((int16_t)ucBuf[14] << 5)) & 0x07FF;
		return 1;
	}
	else
	{
		sbus_channel_data_transformed_.remote_receiver_connect_state = 0;
		return 0;
	}

}
/**************************************************************************
�������ܣ����㷢�͵�����У��λ
��ڲ�����23λΪУ��λ�����������1-22�����������һ������Ϊ���ͻ��ǽ���У��λ
�� �� ֵ������λ
**************************************************************************/
u8 checkSum(unsigned char Count_Number, unsigned char Mode)
{
	unsigned char check_sum = 0, k;
	//�������ݵ�У��
	if (Mode == 1)
	{
		for (k = 0;k < Count_Number; k++)
		{
			check_sum = check_sum ^ send_data_.buffer[k];
		}
	}

	//�������ݵ�У��
	if (Mode == 0)
	{
		for (k = 0;k < Count_Number; k++)
		{
			check_sum = check_sum ^ receive_data.buffer[k];
		}
	}

	return check_sum;
}

/**************************************************************************
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float parseTargetSpeed(u8 High, u8 Low)
{
	short transition; //����ת�����м����
	transition = ((High << 8) + Low); //����8λ�͵�8λ���ϳ�һ��16λ��short������
	return transition / 1000 + (transition % 1000) * 0.001;    //���Ͷ˽����ݷ���ǰ����һ��*1000�ĵ�λ���㣬����������ݺ���Ҫ��ԭ��λ
}

#if 1
/**************************************************
* �������ܣ���ȡң�����趨���ٶȺͷ���ֵ�������ٶ�ʱ��Ҫ�õ�
* �����g_nVelocity��g_nDirector��ֵ
**************************************************/
void parseRemoteData()
{
	int nTemp = 0;  //��ʱ����
	int nSwitchSpeed = 0;

	//�ж�SWA�Ƿ��
	nTemp = sbus_channel_data_transformed_.channel_7;

	if (absInt(nTemp - TURN_OFF_REMOOT) < 10)
	{
		//���ش��ڹر�״̬
		remote_flag = 0;                   //��ģ�رձ�־λ	
		robot_control_mode_ = CONTROL_MODE_UNKNOW; //δ֪ģʽ
		control_speed_level_ = 0;
		return;
	}
	if (absInt(nTemp - TURN_ON_REMOOT) < 10)
	{
		//���ش��ڴ�״̬
		remote_flag = 1;                   // ��ģ������־λ
		ros_flag = 0;                      // ��ģ�������������Ȩ��
		robot_control_mode_ = CONTROL_MODE_REMOTE; //��ģң�ط�ʽѡ��

		nTemp = sbus_channel_data_transformed_.channel_6;

		//��ʱ�ж���ʹ�õ͡��С����ٶȵ�λ
		if (absInt(nTemp - SPEED_LEVEL1) < 10)
		{
			nSwitchSpeed = getTargetSpeed(sbus_channel_data_transformed_.channel_3);//��ȡǰ�����˷���
			//���ٵ�
			if (nSwitchSpeed > 0)
			{
				//ǰ��
				if (nSwitchSpeed == 1)
					control_speed_level_ = SPEED_LOW;
				else
					control_speed_level_ = SPEED_LOW * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else if (nSwitchSpeed < 0)
			{
				//����
				if (nSwitchSpeed == -1)
					control_speed_level_ = -SPEED_LOW;
				else
					control_speed_level_ = SPEED_LOW * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else
			{
				control_speed_level_ = 0;
			}
		}
		else if (absInt(nTemp - SPEED_LEVEL2) < 10)
		{
			//���ٵ�
			nSwitchSpeed = getTargetSpeed(sbus_channel_data_transformed_.channel_3);
			if (nSwitchSpeed > 0)
			{
				//�����ﵽ����ٶ�
				if (nSwitchSpeed == 1)
					control_speed_level_ = SPEED_MIDDLE;
				else
					control_speed_level_ = SPEED_MIDDLE * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else if (nSwitchSpeed < 0)
			{
				//��ת�ﵽ����ٶ�
				if (nSwitchSpeed == -1)
					control_speed_level_ = -SPEED_MIDDLE;
				else
					control_speed_level_ = SPEED_MIDDLE * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else
			{
				control_speed_level_ = 0;
			}
		}
		else if (absInt(nTemp - SPEED_LEVEL3) < 10)
		{
			//���ٵ�
			nSwitchSpeed = getTargetSpeed(sbus_channel_data_transformed_.channel_3);
			if (nSwitchSpeed > 0)
			{
				//�����ﵽ����ٶ�
				if (nSwitchSpeed == 1)
					control_speed_level_ = SPEED_HIGHT;
				else
					control_speed_level_ = SPEED_HIGHT * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else if (nSwitchSpeed < 0)
			{
				//��ת�ﵽ����ٶ�
				if (nSwitchSpeed == -1)
					control_speed_level_ = -SPEED_HIGHT;
				else
					control_speed_level_ = SPEED_HIGHT * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else
			{
				control_speed_level_ = 0;
			}
		}
	}

	// ת���ٶȻ�ȡ
	setDirector();
}
#endif

/**************************************************
* �������ܣ�����������ȡ����ֵ
**************************************************/
int absInt(int nValue)
{
	if (nValue < 0)
	{
		return (-nValue);
	}
	else
	{
		return nValue;
	}
}

/**************************************************
* �������ܣ���������ת���Ƿ�ת����
* �� �� ֵ�����������ֱ��������ת
**************************************************/
int getTargetSpeed(unsigned short usValue)
{
	int nValue = 0;

	//�Ƚ����ٶ��޷��趨������ȡ�ٶ�
	if (usValue > LIMIT_MAX_VAL)
	{
		nValue = 1;
	}
	else if (usValue < LIMIT_MIN_VAL)
	{
		nValue = -1;
	}
	else
	{
		nValue = ((int)usValue - VEL_BASE_VALUE);
		if (abs(nValue) < 10)
			nValue = 0;
	}
	return nValue;
}

/**************************************************
* �������ܣ���������ת���Ƿ�ת����
* �� �� ֵ�����������ֱ��������ת
**************************************************/
int getTargetDirect(unsigned short usValue)
{
	int nValue = 0;

	//�Ƚ��нǶȵ��޷��趨������ȡ�Ƕ�
	if (usValue > LIMIT_MAX_VAL)
	{
		nValue = 1;
	}
	else if (usValue < LIMIT_MIN_VAL)
	{
		nValue = -1;
	}
	else
	{
		nValue = ((int)usValue - DIR_BASE_VALUE);
		if (nValue < 10 && nValue > -10)
		{
			nValue = 0;
		}
	}
	return nValue;
}

#if 1
void setDirector()
{

	int nTemp = 0;  //��ʱ����
	int nSwitchDirect = 0;  //ת�䷽���ѡ��

	//�ж�SWA�Ƿ��
	nTemp = sbus_channel_data_transformed_.channel_7;
	if (absInt(nTemp - TURN_OFF_REMOOT) < 10)
	{
		//���ش��ڹر�״̬
		control_director_level_ = 0;
		return;
	}

	else if (absInt(nTemp - TURN_ON_REMOOT) < 10)
	{
		//���ش��ڴ�״̬
		nTemp = sbus_channel_data_transformed_.channel_6;

		//��ʱ�ж���ʹ�õ͡��С����ٶȵ�λ
		if (absInt(nTemp - SPEED_LEVEL1) < 10)
		{
			//���ٵ�
			nSwitchDirect = getTargetDirect(sbus_channel_data_transformed_.channel_1);
			if (nSwitchDirect > 0)
			{
				//��ת�����
				if (nSwitchDirect == 1)
					control_director_level_ = SPEED_DIR_LOW;
				else
					control_director_level_ = SPEED_DIR_LOW * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else if (nSwitchDirect < 0)
			{
				//��ת�����
				if (nSwitchDirect == -1)
					control_director_level_ = -SPEED_DIR_LOW;
				else
					control_director_level_ = SPEED_DIR_LOW * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else
			{
				control_director_level_ = 0;
			}
		}
		else if (absInt(nTemp - SPEED_LEVEL2) < 10)
		{
			//���ٵ�
			nSwitchDirect = getTargetDirect(sbus_channel_data_transformed_.channel_1);
			if (nSwitchDirect > 0)
			{
				//�Ҵ������
				if (nSwitchDirect == 1)
					control_director_level_ = SPEED_DIR_MIDDLE;
				else
					control_director_level_ = SPEED_DIR_MIDDLE * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else if (getTargetDirect(sbus_channel_data_transformed_.channel_1) < 0)
			{
				//��ת�����
				if (nSwitchDirect == -1)
					control_director_level_ = -SPEED_DIR_MIDDLE;
				else
					control_director_level_ = SPEED_DIR_MIDDLE * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else
			{
				control_director_level_ = 0;
			}
		}
		else if (absInt(nTemp - SPEED_LEVEL3) < 10)
		{
			//���ٵ�
			nSwitchDirect = getTargetDirect(sbus_channel_data_transformed_.channel_1);
			if (nSwitchDirect > 0)
			{
				//��ת�����
				if (nSwitchDirect == 1)
					control_director_level_ = SPEED_DIR_HIGHT;
				else
					control_director_level_ = SPEED_DIR_HIGHT * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else if (getTargetDirect(sbus_channel_data_transformed_.channel_1) < 0)
			{
				//��ת�����
				if (nSwitchDirect == -1)
					control_director_level_ = -SPEED_DIR_HIGHT;
				else
					control_director_level_ = SPEED_DIR_HIGHT * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else
			{
				control_director_level_ = 0;
			}
		}
		else
		{
			control_director_level_ = 0;
		}
	}
}
#endif


