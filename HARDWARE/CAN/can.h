#ifndef __can__h
#define __can__h

#include "stm32f10x.h"

#define CAN_RX0_INT_ENABLE 1   //ʹ���жϱ�ʶ��

extern int g_nState_debug;        // ���������״̬����
extern int g_nMotor_config;       // �������Ƿ�����
extern int g_nReInitMotor;        //�����������ϵ��������δ�ϵ磬���³�ʼ��������������				//Ϊ1������Ҫ���³�ʼ����Ϊ0�������Ҫ���³�ʼ��
extern int g_nHeart_count;        //���������������ڹ涨������û��������������Ϊ�ӻ�����			    //����״̬����ΪSTATE_STOP
extern int g_nHeart_Lastcount;   //��¼�ϴν����������ļ���ֵ

//�����ŷ����ٶȵĹ�����
union URcv_Vel_Data {
	int nVelcity;
	unsigned char ucData[4];
};

// �������������Ͻ��յĹ�����
union URcv_ERROR_Data {
	unsigned short usError;
	unsigned char ucData[2];
};

//����������״̬
enum ENUM_HEART_STAT
{
	STATE_UNKNOW = 0x00,   	//δ֪״̬
	STATE_STOP = 0x04,		//ֹͣ״̬
	STATE_START = 0x05,		//����״̬
	STATE_PRE_OPERATE = 0x7F		//Ԥ����״̬
};

// �����������Ĺ���״̬
enum ENUM_ERROR_STATE
{
	ERROR_NONE = 0x00,  // �޴���
	ERROR_OVER_VALUE = 0x01,  // ��ѹ
	ERROR_LESS_VALUE = 0x02,  // Ƿѹ
	ERROR_OVER_CURRENT = 0x04,  // ����
	ERROR_OVER_LOAD = 0x08,  // ����
	ERROR_OVERDIFF_CURRENT = 0x10,  // ��������
	ERROR_OVERDIFF_ENCODER = 0x20,  // ����������
	ERROR_OVERDIFF_VELOCITY = 0x40,  // �ٶȳ���
	ERROR_REF_VALUE = 0x80,  // �ο���ѹ����
	ERROR_EEPROM = 0x100, // EEPROM��д����
	ERROR_HALL = 0x200  // ��������
};

extern union URcv_Vel_Data uVelLeft;
extern union URcv_Vel_Data uVelRight; //�������ʵʱ�ٶȵı���
extern union URcv_ERROR_Data uError; // �����������Ĺ���״̬

extern enum ENUM_HEART_STAT eMotor_Heart;  //���ҵ��������
extern enum ENUM_ERROR_STATE eMotor_Error;   // �������������ȡ

void canDriverInit(void);
void Debug_PrintfFeedback_Velocity(void);            //���Դ�ӡ��������ٶ�

#endif

