/****************************************
Usart5是接收航模遥控器发送过来的数据；
Usart2是与上位机ROS通信的。
****************************************/
#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"
#include "stm32f10x_conf.h"

/*-------任务堆栈大小------*/
#define COMMUNICATION_STK_SIZE   512 
#define COMMUNICATION_TASK_PRIO  4   

/*—-------------航模模块--------------*/
#define MAX_BUFF_LEN 25     //一帧数据的长度，25个字节
#define VEL_BASE_VALUE   1023    //ch3通道基准值，速度
#define DIR_BASE_VALUE   1023    //ch1通道基准值，方向
#define LIMIT_MAX_VAL    1807    //限幅值，max
#define LIMIT_MIN_VAL    240     //限幅值，min
#define MOTOR_MULTI       3      //速度倍率

#define SPEED_LEVEL1        240     //定义三个速度挡位的基准值,swb控制，ch6通道
#define SPEED_LEVEL2        1023
#define SPEED_LEVEL3        1807

#define TURN_OFF_REMOOT      240     //定义航模开关，ch7通道
#define TURN_ON_REMOOT       1807  

#define SPEED_LOW              20    //定义三个速度等级
#define SPEED_MIDDLE           40
#define SPEED_HIGHT            100
#define SPEED_DIR_LOW          110    //定义三个转弯速度等级
#define SPEED_DIR_MIDDLE       160
#define SPEED_DIR_HIGHT        220

/*—-------------上下位机通信协议相关宏定义--------------*/
#define FRAME_HEADER 0X7B //发送数据的帧头,"{"
#define FRAME_TAIL 0X7D //发送数据的帧尾,"}"
#define SEND_DATA_SIZE 24
#define RECEIVE_DATA_SIZE 11 

/*—-------------上下位机串口发送和接收数据的结构体-------------------*/
typedef struct _SEND_DATA_
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char frame_header;//1个字节
		short car_x;	           //4个字节
		short car_y;             //4个字节
		short car_z;             //4个字节
		short battery_voltage;       //4个字节

		unsigned char frame_tail;  //1个字节
	}SENSOR_STR;

}SEND_DATA;

typedef struct _RECEIVE_DATA_
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char frame_header;// 1
		float car_x;	          //4个字节
		float car_y;            //4个字节
		float car_z;            //4个字节
		unsigned char frame_tail;     //1个字节
	}CONTROL_STR;

}RECEIVE_DATA;

/*—-------------航模模块usart5接收数据结构体--------------*/
typedef struct
{
	unsigned short channel_1;//通道1数值   控制方向
	unsigned short channel_2;//通道2数值   右侧方向（即左右）
	unsigned short channel_3;//通道3数值   控制速度等级
	unsigned short channel_4;//通道4数值   左侧油门（即左右）
	unsigned short channel_5;//通道5数值   
	unsigned short channel_6;//通道6数值    对应的是SWB
	unsigned short channel_7;//通道7数值    对应的是SWA
	unsigned short channel_8;//通道8数值    对应的是SWC
	unsigned short channel_9;//通道9数值    对应的是SWD
	unsigned short channel_10;//通道10数值，对应的是VRA，来控制车灯
	unsigned char remote_receiver_connect_state;//遥控器与接收器连接状态 0=未连接，1=正常连接
}SBUS_CHANNEL_STRUCT;

extern char remote_flag_;              //遥控器控制标志位
extern unsigned char ros_flag_;        // ROS上位机接入标志位

extern SBUS_CHANNEL_STRUCT sbus_channel_data_transformed_; //存放遥控器转换后的通道数据
extern unsigned char usart5_recive_buffer_[MAX_BUFF_LEN];  //存放接收到的数据

extern int control_speed_level_;            //ch3,左边上下控制小车速度
extern int control_director_level_;         //ch1,右边左右控制小车转向

extern float recive_robot_speed_x_;         // 串口接收到的速度数据
extern float recive_robot_speed_y_;
extern float recive_robot_speed_z_;

void communicationTask(void* pvParameters);
void usart5Init(unsigned int unBound);              //串口5作为航模数据接收
unsigned char sbusTransform(unsigned char* ucBuf);
void parseRemoteData(void);                         //设置电机运行的速度
void setDirector(void);                             //设置电机的转向                          
int absInt(int nValue);
int getTargetSpeed(unsigned short usValue);
int getTargetDirect(unsigned short usValue);
void usart2Init(u32 bound);         //usart2作为上下位机通信模块
void usart2SendByte(u8 data);
void usart2SendData(void);
void USART2_IRQHandler(void);       //中断中作为数据接收处理
unsigned char checkSum(unsigned char Count_Number, unsigned char Mode);
float parseTargetSpeed(u8 High, u8 Low);   //上位机传输下来的速度信息解析
void getToSendData(void);         //串口2发送给上位机数据转换

#endif

