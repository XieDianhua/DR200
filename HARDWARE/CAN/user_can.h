#ifndef __user_can__h
#define __user_can__h

#include "can.h"

#define MOTOR_CAN_ID   0x04

extern int g_nHeart_Time;        //�����೤ʱ��û������������Ϊ�ӻ�����


void CAN_ConfigFilter(void);
uint8_t RPDO0_Config(uint8_t ID);         //RPDO0����
uint8_t RPDO1_Config(uint8_t ID);         //RPDO1����
uint8_t RPDO2_Config(uint8_t ID);         //RPDO2����
uint8_t RPDO3_Config(uint8_t ID);         //RPDO3����
uint8_t TPDO0_Config(uint8_t ID);         //TPDO0����
uint8_t TPDO1_Config(uint8_t ID);         //TPDO1����
uint8_t TPDO2_Config(uint8_t ID);         //TPDO2����
uint8_t TPDO3_Config(uint8_t ID);         //TPDO3����

uint8_t NMT_Control(uint8_t ID, const uint8_t Data0, const uint8_t Data1);//NMT���ƹ���
uint8_t Driver_Enable(uint8_t ID);        //ʹ���л�


uint8_t Profile_Velocity_Init(uint8_t ID);//�ٶ�ģʽ��ʼ����
void Profile_Velocity_Set(uint8_t ID, int nLeft_Velocity, int nRight_Velocity);    //�ٶ�ģʽ����
void Motor_Init(uint8_t ID);                     //һ�϶������ʼ��
void ZLAC8015_Control(void);              //ZLAC8015���Գ���

void Emergecy_Stop(uint8_t ID);           //��ͣ���ã�����
void Release_Stop_Vel(uint8_t ID);        //�ٶ�ģʽ�½����ͣ
void Set_Heartbeat_Pack(uint8_t ID);      //���ô�վ���������ƣ���ʱ�����������жϴ�վ�Ƿ�����
void Release_ChildNode(uint8_t ID);       //�ýڵ����ֹͣ״̬
void ReInitalMotor(void);                 //�Ƿ������ʹ�ܵ��
void Recover_ToStart(uint8_t ID);         //���½������״̬
void Is_Offline(void);                     //�������жϵ���Ƿ�����
void Clear_Error_SendData(uint8_t ID);    // �����������Ҫ���͵�ָ��
void Clear_Error(void);                   // ������� 

//void ReEnableMotor(uint8_t ID);           //�Ƿ�����ʹ�ܵ��

#endif

