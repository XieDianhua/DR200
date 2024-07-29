#ifndef __user_can__h
#define __user_can__h

#include "can.h"

#define MOTOR_CAN_ID   0x04

extern int g_nHeart_Time;        //超过多长时间没回心跳包，认为从机下线


void CAN_ConfigFilter(void);
uint8_t RPDO0_Config(uint8_t ID);         //RPDO0配置
uint8_t RPDO1_Config(uint8_t ID);         //RPDO1配置
uint8_t RPDO2_Config(uint8_t ID);         //RPDO2配置
uint8_t RPDO3_Config(uint8_t ID);         //RPDO3配置
uint8_t TPDO0_Config(uint8_t ID);         //TPDO0配置
uint8_t TPDO1_Config(uint8_t ID);         //TPDO1配置
uint8_t TPDO2_Config(uint8_t ID);         //TPDO2配置
uint8_t TPDO3_Config(uint8_t ID);         //TPDO3配置

uint8_t NMT_Control(uint8_t ID, const uint8_t Data0, const uint8_t Data1);//NMT控制管理
uint8_t Driver_Enable(uint8_t ID);        //使能切换


uint8_t Profile_Velocity_Init(uint8_t ID);//速度模式初始配置
void Profile_Velocity_Set(uint8_t ID, int nLeft_Velocity, int nRight_Velocity);    //速度模式测试
void Motor_Init(uint8_t ID);                     //一拖二电机初始化
void ZLAC8015_Control(void);              //ZLAC8015测试程序

void Emergecy_Stop(uint8_t ID);           //急停设置，锁轴
void Release_Stop_Vel(uint8_t ID);        //速度模式下解除急停
void Set_Heartbeat_Pack(uint8_t ID);      //设置从站的心跳机制，定时产生心跳来判断从站是否在线
void Release_ChildNode(uint8_t ID);       //让节点进入停止状态
void ReInitalMotor(void);                 //是否出重新使能电机
void Recover_ToStart(uint8_t ID);         //重新进入操作状态
void Is_Offline(void);                     //心跳包判断电机是否下线
void Clear_Error_SendData(uint8_t ID);    // 清除故障所需要发送的指令
void Clear_Error(void);                   // 清除故障 

//void ReEnableMotor(uint8_t ID);           //是否重新使能电机

#endif

