#include "user_can.h"
#include "delay.h"
#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"
#include "usartx.h"


int g_nHeart_Time = 0;//����ʱ��û����������1000


/**********************************************************
* �������ܣ� CAN��׼֡���ͺ�����
* ������     ID����ڵ�ID��Data_size�������ݳ��ȣ�Data���ݡ�
* ˵����     �ޡ�
* ����ֵ��   0�����ͳɹ���1������ʧ��
**********************************************************/
uint8_t CAN_Send(uint32_t ID, uint32_t Data_Size, uint8_t* Data)
{
	CanTxMsg pTxMessage;
	int nIndex = 0;
	int nReturn = -1;
	int nCount = 0;

	pTxMessage.DLC = Data_Size;
	pTxMessage.IDE = CAN_Id_Standard;
	pTxMessage.RTR = CAN_RTR_Data;
	pTxMessage.StdId = ID;

	for (; nIndex < 8; nIndex++)
	{
		pTxMessage.Data[nIndex] = Data[nIndex];
	}
	nIndex = 0;

	nReturn = CAN_Transmit(CAN1, &pTxMessage);  //���ͳɹ�������0

	/*����Ҫ������ȴ���������Ȼ�ͻᵼ��canͨ�ŷ������ݲ��ɹ�*/
	while ((CAN_TransmitStatus(CAN1, nReturn) == CAN_TxStatus_Failed)
		&& (nCount < 0xFFFF))  //�ȴ����ͽ���
	{
		nCount++;
	}

	if (nCount > 0xFFFF)
	{
		//������
		printf("Send Msg faild! StdId is: %x\r\n", pTxMessage.StdId);
		return 1;  //����1���������ݲ��ɹ�
	}
	delay_ms(5);   //û�������ʱ�Ļ����ͻ�һֱ����
	//	//���Ե�ʱ���õ��Ĵ��ڴ�ӡ
	//	printf("Send Msg success! StdId is: %x\r\n",pTxMessage.StdId);
	return 0;
}

/**********************************************************
* �������ܣ� һ�϶������������ʼ����
* ������     IDΪ�ӻ���ַ��
* ˵����     �ޡ�
**********************************************************/
void Motor_Init(uint8_t ID)
{
	RPDO0_Config(ID);  //������ӳ��
	RPDO1_Config(ID);  //�����Ŀ���ٶ�
	RPDO2_Config(ID);  //�Ҳ���Ŀ���ٶ�
	TPDO0_Config(ID);  //�����ʵʱ�����ٶȻ�ȡ
	TPDO1_Config(ID);  //�Ҳ���ʵʱ�����ٶȻ�ȡ
	TPDO3_Config(ID);  //��ȡ����������

	Profile_Velocity_Init(ID);   //�ٶ�ģʽ��ʼ��
	NMT_Control(ID, 0x01, ID);  //����PDO1��������
	Driver_Enable(ID);
	Set_Heartbeat_Pack(ID);  //����������

	delay_ms(200);
}

/*�������������EEPROM*/
uint8_t Save_EEPROM(uint8_t ID)
{
	uint8_t Data[8] = { 0x2B, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00 };
	return CAN_Send(0x600 + ID, 0x08, Data);
}


/*�ָ���������*/
uint8_t Reset_Driver(uint8_t ID)
{
	uint8_t Data[8] = { 0x2B, 0x09, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00 };
	return CAN_Send(0x600 + ID, 0x08, Data);
}

/**********************************************************
* �������ܣ� RPDO0�¼�������
* ������     ID����ڵ��ַ��
* ˵����     RPDO0ӳ��0x6040�������֣���
*            RPDO0-COB-ID:0x200 + ID
**********************************************************/
uint8_t RPDO0_Config(uint8_t ID)
{
	uint8_t Data[8] = { 0x2F, 0x00, 0x14, 0x02, 0xFE, 0x00, 0x00, 0x00 };//RPDO0�¼�����
	CAN_Send(0x600 + ID, 0x08, Data); //0x600+ID������Ǵӻ��ĵ�ַ����������ַ����Ĭ����0x604
	delay_ms(5);                     //������������+16λ������+8λ������+�������

	//���ӳ��
	Data[0] = 0x2F;
	Data[1] = 0x00;
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);


	Data[0] = 0x23;
	Data[1] = 0x00;                 //����ӳ�䲿�ֵĲ���
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x10;                 //��һλӦ��û��ϵ
	Data[5] = 0x00;
	Data[6] = 0x40;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO0ӳ��0x6040,�����ֵĲ���
	delay_ms(5);  //���뼶��ʱ

	//����ӳ��
	Data[0] = 0x2F;
	Data[1] = 0x00;
	Data[2] = 0x16;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO0����1��ӳ�䣬ӳ�����
	delay_ms(5);

	return 0x00;
}


/**********************************************************
* �������ܣ� RPDO1�¼�������
* ������     ID����ڵ��ַ��
* ˵����     RPDO1ӳ��0x60FF 01������Ŀ���ٶȣ���
*            RPDO1-COB-ID:0x300 + ID
**********************************************************/
uint8_t RPDO1_Config(uint8_t ID)
{
	uint8_t Data[8];

	//���ӳ��
	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x01;
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x20;   // ����ҪΪ0x20����32λ
	Data[5] = 0x01;
	Data[6] = 0xFF;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO1ӳ��0x60FF 01������Ŀ���ٶ�
	delay_ms(5);

	//�¼�����
	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x14;
	Data[3] = 0x02;
	Data[4] = 0xFE;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO1�¼�����
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x16;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO1����1��ӳ��
	delay_ms(5);

	return 0x00;
}




/**********************************************************
* �������ܣ� RPDO2�¼�������
* ������     ID����ڵ��ַ��
* ˵����     RPDO2ӳ��0x60FF 02���ҵ��Ŀ���ٶȣ���
*            RPDO2-COB-ID:0x400 + ID
**********************************************************/
uint8_t RPDO2_Config(uint8_t ID)
{
	uint8_t Data[8];

	//���ӳ��
	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x02;
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x02;
	Data[6] = 0xFF;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2ӳ��0x60FF 02
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x14;
	Data[3] = 0x02;
	Data[4] = 0xFE;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2�¼�����
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x16;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2����1��ӳ��
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* �������ܣ� RPDO2�¼�������
* ������     ID����ڵ��ַ��
* ˵����     RPDO2ӳ��0x607A 00�����Ŀ��λ�ã���
*            RPDO2-COB-ID:0x400 + ID
**********************************************************/
uint8_t RPDO3_Config(uint8_t ID)
{
	uint8_t Data[8];

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x14;
	Data[3] = 0x02;
	Data[4] = 0xFE;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2�¼�����
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x02;
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x00;
	Data[6] = 0x7A;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2ӳ��0x607A 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x16;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2����1��ӳ��
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* �������ܣ� TPDO0��ʱ��������
* ������     ID����ڵ��ַ��
* ˵����     TPDO0ӳ��0x606C 01����෴���ٶȣ���
*            TPDO0��ʱ��100ms��
*            TPDO0-COB-ID:0x180 + ID
**********************************************************/
uint8_t TPDO0_Config(uint8_t ID)
{
	uint8_t Data[8];

	//���ӳ��
	Data[0] = 0x2F;
	Data[1] = 0x00;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x00;
	Data[2] = 0x18;
	Data[3] = 0x02;
	Data[4] = 0xFF;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO0��ʱ������
	delay_ms(5);

	Data[0] = 0x2B;
	Data[1] = 0x00;
	Data[2] = 0x18;
	Data[3] = 0x05;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO0��ʱ��200*0.5ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x00;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x01;
	Data[6] = 0x6C;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO0ӳ��0x606C 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x00;
	Data[2] = 0x1A;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO0����1��ӳ��
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* �������ܣ� TPDO1��ʱ��������
* ������     ID����ڵ��ַ��
* ˵����     TPDO1ӳ��0x606C 02���Ҳ෴���ٶȣ���
*            TPDO1��ʱ��100ms��
*            TPDO1-COB-ID:0x280 + ID
**********************************************************/
uint8_t TPDO1_Config(uint8_t ID)
{
	uint8_t Data[8];

	//���ӳ��
	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x18;
	Data[3] = 0x02;
	Data[4] = 0xFF;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO1��ʱ������
	delay_ms(5);

	Data[0] = 0x2B;
	Data[1] = 0x01;
	Data[2] = 0x18;
	Data[3] = 0x05;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDOA��ʱ��200*0.5ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x01;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x02;
	Data[6] = 0x6C;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO1ӳ��0x6077 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x1A;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO1����1��ӳ��
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* �������ܣ� TPDO2��ʱ��������
* ������     ID����ڵ��ַ��
* ˵����     TPDO2ӳ��0x6064 00���������λ�ã���
*            TPDO2��ʱ��ʱ��100ms��
*            TPDO2-COB-ID:0x380 + ID
**********************************************************/
uint8_t TPDO2_Config(uint8_t ID)
{
	uint8_t Data[8];

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x18;
	Data[3] = 0x02;
	Data[4] = 0xFF;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO2�¼�����
	delay_ms(5);

	Data[0] = 0x2B;
	Data[1] = 0x02;
	Data[2] = 0x18;
	Data[3] = 0x05;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO2��ʱ��200*0.5ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x02;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x00;
	Data[6] = 0x64;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO2ӳ��0x6064 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x1A;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO2����1��ӳ��
	delay_ms(5);

	return 0x00;
}

/**********************************************************
* �������ܣ� TPDO3�¼�������
* ������     ID����ڵ��ַ��
* ˵����     TPDO3ӳ��0x603F 00�������룩��
*            TPDO0��ֹʱ��500ms��
*            TPDO3-COB-ID:0x480 + ID
**********************************************************/
uint8_t TPDO3_Config(uint8_t ID)
{
	uint8_t Data[8];

	Data[0] = 0x2F;
	Data[1] = 0x03;
	Data[2] = 0x18;
	Data[3] = 0x02;
	Data[4] = 0xFE;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO3�¼�����
	delay_ms(5);

	Data[0] = 0x2B;
	Data[1] = 0x03;
	Data[2] = 0x18;
	Data[3] = 0x03;
	Data[4] = 0xE8;
	Data[5] = 0x03;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO3��ֹʱ��1000*0.5ms
	delay_ms(5);


	Data[0] = 0x23;
	Data[1] = 0x03;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x20;        // �ĸ��ֽڣ���Ӧ32λ
	Data[5] = 0x00;
	Data[6] = 0x3F;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO3ӳ��0x603F 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x03;
	Data[2] = 0x1A;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO3����1��ӳ��
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* �������ܣ� NMT�ڵ�״̬���ã��Խڵ�ʹ�ܡ�״̬�л���
* ������     ID����ڵ��ַ��data0Ϊ����ѡ��data1Ϊ�ڵ��ַ��
* ˵����     ��������PDO����
**********************************************************/
uint8_t NMT_Control(uint8_t ID, const uint8_t Data0, const uint8_t Data1)
{
	uint8_t Data[2] = { 0 };
	Data[0] = Data0;
	Data[1] = Data1;

	return CAN_Send(0x000, 0x02, Data);
}



/**********************************************************
* �������ܣ� ���õ��Ŀ���ٶȣ�����canopenЭ�鲽���ʼ��
* ������     ID����ڵ��ַ
* ˵����     0x200 + �ڵ�id�������PDO0�Ĺ����� + Node_id
**********************************************************/
uint8_t Driver_Enable(uint8_t ID)
{
	uint8_t Data_2Byte[2];
	//	//�������
	//	uint8_t Data[8] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
	//	CAN_Send(0x600 + ID, 0x08, Data); //0x600+ID������Ǵӻ��ĵ�ַ����������ַ����Ĭ����0x604


	//�ϵ����������ʼ������
	Data_2Byte[0] = 0x00;
	Data_2Byte[1] = 0x00;
	CAN_Send(0x200 + ID, 0x02, Data_2Byte);

	Data_2Byte[0] = 0x06;
	Data_2Byte[1] = 0x00;
	CAN_Send(0x200 + ID, 0x02, Data_2Byte);

	Data_2Byte[0] = 0x07;
	Data_2Byte[1] = 0x00;
	CAN_Send(0x200 + ID, 0x02, Data_2Byte);
	delay_ms(5);

	Data_2Byte[0] = 0x0F;
	Data_2Byte[1] = 0x00;
	CAN_Send(0x200 + ID, 0x02, Data_2Byte);

	Data_2Byte[0] = 0x0F;
	Data_2Byte[1] = 0x00;
	return CAN_Send(0x200 + ID, 0x02, Data_2Byte);

}

//
/**********************************************************
* �������ܣ� ����ٶ�ģʽ��ʼ����
* ������     ID����ӻ��ڵ��ַ��
* ˵����     0x600 �������SDOͨ�ţ�
* 		     ���õ���ļ��ٺͼ���ʱ�䡣
**********************************************************/
uint8_t Profile_Velocity_Init(uint8_t ID)
{
	uint8_t Data[8] = { 0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00 };//�����ٶ�ģʽ
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	//����Ϊ�첽��������
	Data[0] = 0x2B;
	Data[1] = 0x0F;
	Data[2] = 0x20;
	Data[3] = 0x00;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x0D;
	Data[2] = 0x20;
	Data[3] = 0x01;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//�����������ʼ�ٶ�
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x0D;
	Data[2] = 0x20;
	Data[3] = 0x02;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//�����Ҳ�����ʼ�ٶ�
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x83;
	Data[2] = 0x60;
	Data[3] = 0x01;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//�������������ʱ��200ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x83;
	Data[2] = 0x60;
	Data[3] = 0x02;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//�����Ҳ�������ʱ��200ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x84;
	Data[2] = 0x60;
	Data[3] = 0x01;
	Data[4] = 0x64;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//�������������ʱ��100ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x84;
	Data[2] = 0x60;
	Data[3] = 0x02;
	Data[4] = 0x64;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//�����Ҳ�������ʱ��100ms
	delay_ms(5);

	//

	return 0x00;
}


/**********************************************************
* �������ܣ� ���õ��Ŀ���ٶȣ�
* ������     ID����ڵ��ַ, nVelocity�������õ��ٶ�
* ˵����     0x300 + �ڵ�id�������PDO1�Ĺ����� + Node_id
**********************************************************/
void Profile_Velocity_Set(uint8_t ID, int nLeft_Velocity, int nRight_Velocity)
{
	uint8_t Data_4Byte[4];

	//�����͵�����ת��Ϊ4���ֽڵ�unsigned char��Ŀ���ٶȣ�Сβ�洢
	memcpy(Data_4Byte, &nLeft_Velocity, 4);

	CAN_Send(0x300 + ID, 0x04, Data_4Byte);//100rpm   �����
	delay_ms(5);
	memcpy(Data_4Byte, &nRight_Velocity, 4);
	CAN_Send(0x400 + ID, 0x04, Data_4Byte);//100rpm   �Ҳ���
}


/**********************************************************
* �������ܣ� С����ͣ����
* ������     ID����ڵ��ַ
* ˵����     0x800 + �ڵ�id�������SDO�Ĺ����� + Node_id
**********************************************************/
void Emergecy_Stop(uint8_t ID)
{
	//��ͣ�����ֹͣ����������״̬
	uint8_t Data[8] = { 0x2B, 0x40, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00 };//��ͣ���Ŀ���ٶ�
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);
}


/**********************************************************
* �������ܣ� �ٶ�ģʽ�����ͣ
* ������     ID����ڵ��ַ
* ˵����     0x800 + �ڵ�id�������SDO�Ĺ����� + Node_id
**********************************************************/
void Release_Stop_Vel(uint8_t ID)
{
	//����ʹ��
	uint8_t Data[8] = { 0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00 };//�����ٶ�ģʽ
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	//����Ŀ��ת��
	Profile_Velocity_Set(ID, 0, 0);
}


/**********************************************************
* �������ܣ� ���������ƺ���
* ������     ID����ڵ��ַ
* ˵����     ���ö����ֵ�1017h�д�վ����������ʱ�䣻���ö���
*            �ֵ�1016h����վ�����������ʱ�䡣���ڼ�����������
**********************************************************/
void Set_Heartbeat_Pack(uint8_t ID)
{
	uint8_t Data[8] = { 0x00 };//������������ʱ��

	Data[0] = 0x2B;
	Data[1] = 0x17;
	Data[2] = 0x10;
	Data[3] = 0x00;
	Data[4] = 0xE8;
	Data[5] = 0x03;
	Data[6] = 0x00;
	Data[7] = 0x00;

	// һ�϶�������COB-ID�� 0x700 + Node-ID
	CAN_Send(0x600 + ID, 0x08, Data);//��������������ʱ��1000
	delay_ms(5);
}


/**********************************************************
* �������ܣ� �ýڵ����ֹͣ״̬
* ������     ID����ڵ��ַ
* ˵����     �ڵ㶨�ڲ��ظ�������������ڵ��Ѿ�����
*            ͨ��NMT�����ýڵ����ֹͣ״̬��
**********************************************************/
void Release_ChildNode(uint8_t ID)
{
	//0x02����ֹͣ����
	NMT_Control(ID, 0x02, ID);

	//�����Ӧ�ı�־λ
	if (ID == MOTOR_CAN_ID)
	{
		g_nMotor_config = 0;
	}
}


/**********************************************************
* �������ܣ� �ýڵ�������״̬
* ������     ID����ڵ��ַ
* ˵����     ������״̬�ص�����״̬
*
**********************************************************/
void Recover_ToStart(uint8_t ID)
{
	//0x01������������
	NMT_Control(ID, 0x01, ID);
}

/**********************************************************
* �������ܣ� ���³�ʼ�����
* ˵    ���� �ڵ���ϵ�������ϵ�ʱ��Ҫ���³�ʼ�������
*            �ȸ����ư��ϵ磬Ȼ�������ӵ�ʱ����Ҫ���³�ʼ����
**********************************************************/
void ReInitalMotor(void)
{
	if (g_nReInitMotor == 1)
	{
		printf("Reinit LB����\r\n");
		while (g_nMotor_config != 1)
		{
			Motor_Init(MOTOR_CAN_ID);
		}
		g_nReInitMotor = 0;
		g_nMotor_config = 1;
		g_nHeart_Time = 0;
	}
}


/**********************************************************
* �������ܣ� ���������жϵ���Ƿ�����
* ˵    ���� �ӻ���һ��ʱ��δ�ظ����������������
**********************************************************/
void Is_Offline(void)
{
	//����ʱ��Ϊ150��show.cpp����ִ�е����ڣ�3s
	if (g_nHeart_Time > 150)
	{
		if (eMotor_Heart != STATE_STOP)
		{
			//��󷽵��
			if (g_nHeart_count < 3 || (g_nHeart_count == g_nHeart_Lastcount))
			{
				//10s��δ�յ�3�λذ�
				eMotor_Heart = STATE_STOP;
				g_nMotor_config = 0;
				g_nHeart_count = 0;
			}
		}

		g_nHeart_Time = 0;

		// ����ǰ�ļ���ֵ��ֵ����һ�μ���������Ϊ���ж��������Ƿ�����
		g_nHeart_Lastcount = g_nHeart_count;
	}
}

/**********************************************************
* �������ܣ� ��������������Ĺ��ϣ��м����ǲ�������ģ�
* ˵    ���� ֻҪ�������������ϣ����ж��м�������
			 �������е�������������ﵽ������ϵ�Ŀ��
**********************************************************/
void Clear_Error_SendData(uint8_t ID)
{
	//	//�������
	uint8_t Data[8] = { 0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00 };
	CAN_Send(0x600 + ID, 0x08, Data); //0x600+ID������Ǵӻ��ĵ�ַ����������ַ����Ĭ����0x604

}



void Clear_Error()
{
	/*if(eMotor_Error == ERROR_OVER_LOAD ||          eMotor_Error == ERROR_OVERDIFF_CURRENT
		|| eMotor_Error == ERROR_OVERDIFF_ENCODER || eMotor_Error == ERROR_OVERDIFF_VELOCITY
	  || eMotor_Error == ERROR_REF_VALUE ||        eMotor_Error == ERROR_EEPROM)*/
	if (eMotor_Error != ERROR_NONE)
	{
		printf("eMotor_Error's error is: %d\r\n", eMotor_Error);
		printf("clear error \r\n");

		// ��λ�����״̬
		g_nMotor_config = 0;
		eMotor_Heart = STATE_STOP;
		g_nReInitMotor = 1;

		Clear_Error_SendData(0x04);
		eMotor_Error = ERROR_NONE;
		// �������֮��Ҫ����ʹ�ܵ��
		ReInitalMotor();
	}
}


