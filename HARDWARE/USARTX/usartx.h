/****************************************
Usart5�ǽ��պ�ģң�������͹��������ݣ�
Usart2������λ��ROSͨ�ŵġ�
****************************************/
#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"
#include "stm32f10x_conf.h"

/*-------�����ջ��С------*/
#define COMMUNICATION_STK_SIZE   512 
#define COMMUNICATION_TASK_PRIO  4   

/*��-------------��ģģ��--------------*/
#define MAX_BUFF_LEN 25     //һ֡���ݵĳ��ȣ�25���ֽ�
#define VEL_BASE_VALUE   1023    //ch3ͨ����׼ֵ���ٶ�
#define DIR_BASE_VALUE   1023    //ch1ͨ����׼ֵ������
#define LIMIT_MAX_VAL    1807    //�޷�ֵ��max
#define LIMIT_MIN_VAL    240     //�޷�ֵ��min
#define MOTOR_MULTI       3      //�ٶȱ���

#define SPEED_LEVEL1        240     //���������ٶȵ�λ�Ļ�׼ֵ,swb���ƣ�ch6ͨ��
#define SPEED_LEVEL2        1023
#define SPEED_LEVEL3        1807

#define TURN_OFF_REMOOT      240     //���庽ģ���أ�ch7ͨ��
#define TURN_ON_REMOOT       1807  

#define SPEED_LOW              20    //���������ٶȵȼ�
#define SPEED_MIDDLE           40
#define SPEED_HIGHT            100
#define SPEED_DIR_LOW          110    //��������ת���ٶȵȼ�
#define SPEED_DIR_MIDDLE       160
#define SPEED_DIR_HIGHT        220

/*��-------------����λ��ͨ��Э����غ궨��--------------*/
#define FRAME_HEADER 0X7B //�������ݵ�֡ͷ,"{"
#define FRAME_TAIL 0X7D //�������ݵ�֡β,"}"
#define SEND_DATA_SIZE 24
#define RECEIVE_DATA_SIZE 11 

/*��-------------����λ�����ڷ��ͺͽ������ݵĽṹ��-------------------*/
typedef struct _SEND_DATA_
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char frame_header;//1���ֽ�
		short car_x;	           //4���ֽ�
		short car_y;             //4���ֽ�
		short car_z;             //4���ֽ�
		short battery_voltage;       //4���ֽ�

		unsigned char frame_tail;  //1���ֽ�
	}SENSOR_STR;

}SEND_DATA;

typedef struct _RECEIVE_DATA_
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char frame_header;// 1
		float car_x;	          //4���ֽ�
		float car_y;            //4���ֽ�
		float car_z;            //4���ֽ�
		unsigned char frame_tail;     //1���ֽ�
	}CONTROL_STR;

}RECEIVE_DATA;

/*��-------------��ģģ��usart5�������ݽṹ��--------------*/
typedef struct
{
	unsigned short channel_1;//ͨ��1��ֵ   ���Ʒ���
	unsigned short channel_2;//ͨ��2��ֵ   �Ҳ෽�򣨼����ң�
	unsigned short channel_3;//ͨ��3��ֵ   �����ٶȵȼ�
	unsigned short channel_4;//ͨ��4��ֵ   ������ţ������ң�
	unsigned short channel_5;//ͨ��5��ֵ   
	unsigned short channel_6;//ͨ��6��ֵ    ��Ӧ����SWB
	unsigned short channel_7;//ͨ��7��ֵ    ��Ӧ����SWA
	unsigned short channel_8;//ͨ��8��ֵ    ��Ӧ����SWC
	unsigned short channel_9;//ͨ��9��ֵ    ��Ӧ����SWD
	unsigned short channel_10;//ͨ��10��ֵ����Ӧ����VRA�������Ƴ���
	unsigned char remote_receiver_connect_state;//ң���������������״̬ 0=δ���ӣ�1=��������
}SBUS_CHANNEL_STRUCT;

extern char remote_flag_;              //ң�������Ʊ�־λ
extern unsigned char ros_flag_;        // ROS��λ�������־λ

extern SBUS_CHANNEL_STRUCT sbus_channel_data_transformed_; //���ң����ת�����ͨ������
extern unsigned char usart5_recive_buffer_[MAX_BUFF_LEN];  //��Ž��յ�������

extern int control_speed_level_;            //ch3,������¿���С���ٶ�
extern int control_director_level_;         //ch1,�ұ����ҿ���С��ת��

extern float recive_robot_speed_x_;         // ���ڽ��յ����ٶ�����
extern float recive_robot_speed_y_;
extern float recive_robot_speed_z_;

void communicationTask(void* pvParameters);
void usart5Init(unsigned int unBound);              //����5��Ϊ��ģ���ݽ���
unsigned char sbusTransform(unsigned char* ucBuf);
void parseRemoteData(void);                         //���õ�����е��ٶ�
void setDirector(void);                             //���õ����ת��                          
int absInt(int nValue);
int getTargetSpeed(unsigned short usValue);
int getTargetDirect(unsigned short usValue);
void usart2Init(u32 bound);         //usart2��Ϊ����λ��ͨ��ģ��
void usart2SendByte(u8 data);
void usart2SendData(void);
void USART2_IRQHandler(void);       //�ж�����Ϊ���ݽ��մ���
unsigned char checkSum(unsigned char Count_Number, unsigned char Mode);
float parseTargetSpeed(u8 High, u8 Low);   //��λ�������������ٶ���Ϣ����
void getToSendData(void);         //����2���͸���λ������ת��

#endif

