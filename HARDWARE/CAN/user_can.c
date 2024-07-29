#include "user_can.h"
#include "delay.h"
#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "string.h"
#include "usartx.h"


int g_nHeart_Time = 0;//超过时长没产生心跳包1000


/**********************************************************
* 函数功能： CAN标准帧发送函数。
* 参数：     ID代表节点ID，Data_size代表数据长度，Data数据。
* 说明：     无。
* 返回值：   0代表发送成功；1代表发送失败
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

	nReturn = CAN_Transmit(CAN1, &pTxMessage);  //发送成功，返回0

	/*必须要有这个等待函数，不然就会导致can通信发送数据不成功*/
	while ((CAN_TransmitStatus(CAN1, nReturn) == CAN_TxStatus_Failed)
		&& (nCount < 0xFFFF))  //等待发送结束
	{
		nCount++;
	}

	if (nCount > 0xFFFF)
	{
		//错误处理
		printf("Send Msg faild! StdId is: %x\r\n", pTxMessage.StdId);
		return 1;  //返回1代表发送数据不成功
	}
	delay_ms(5);   //没有这个延时的话，就会一直报错。
	//	//调试的时候用到的串口打印
	//	printf("Send Msg success! StdId is: %x\r\n",pTxMessage.StdId);
	return 0;
}

/**********************************************************
* 函数功能： 一拖二电机驱动器初始化。
* 参数：     ID为从机地址。
* 说明：     无。
**********************************************************/
void Motor_Init(uint8_t ID)
{
	RPDO0_Config(ID);  //控制字映射
	RPDO1_Config(ID);  //左侧电机目标速度
	RPDO2_Config(ID);  //右侧电机目标速度
	TPDO0_Config(ID);  //左侧电机实时反馈速度获取
	TPDO1_Config(ID);  //右侧电机实时反馈速度获取
	TPDO3_Config(ID);  //获取驱动器故障

	Profile_Velocity_Init(ID);   //速度模式初始化
	NMT_Control(ID, 0x01, ID);  //开启PDO1传输数据
	Driver_Enable(ID);
	Set_Heartbeat_Pack(ID);  //设置心跳包

	delay_ms(200);
}

/*保存参数配置至EEPROM*/
uint8_t Save_EEPROM(uint8_t ID)
{
	uint8_t Data[8] = { 0x2B, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00 };
	return CAN_Send(0x600 + ID, 0x08, Data);
}


/*恢复出厂配置*/
uint8_t Reset_Driver(uint8_t ID)
{
	uint8_t Data[8] = { 0x2B, 0x09, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00 };
	return CAN_Send(0x600 + ID, 0x08, Data);
}

/**********************************************************
* 函数功能： RPDO0事件触发。
* 参数：     ID代表节点地址。
* 说明：     RPDO0映射0x6040（控制字）；
*            RPDO0-COB-ID:0x200 + ID
**********************************************************/
uint8_t RPDO0_Config(uint8_t ID)
{
	uint8_t Data[8] = { 0x2F, 0x00, 0x14, 0x02, 0xFE, 0x00, 0x00, 0x00 };//RPDO0事件触发
	CAN_Send(0x600 + ID, 0x08, Data); //0x600+ID代表的是从机的地址（驱动器地址），默认是0x604
	delay_ms(5);                     //数据由命令字+16位所索引+8位子索引+数据组成

	//清除映射
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
	Data[1] = 0x00;                 //对象映射部分的参数
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x10;                 //这一位应该没关系
	Data[5] = 0x00;
	Data[6] = 0x40;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO0映射0x6040,控制字的操作
	delay_ms(5);  //毫秒级延时

	//启动映射
	Data[0] = 0x2F;
	Data[1] = 0x00;
	Data[2] = 0x16;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO0开启1个映射，映射个数
	delay_ms(5);

	return 0x00;
}


/**********************************************************
* 函数功能： RPDO1事件触发。
* 参数：     ID代表节点地址。
* 说明：     RPDO1映射0x60FF 01（左电机目标速度）；
*            RPDO1-COB-ID:0x300 + ID
**********************************************************/
uint8_t RPDO1_Config(uint8_t ID)
{
	uint8_t Data[8];

	//清除映射
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
	Data[4] = 0x20;   // 必须要为0x20，即32位
	Data[5] = 0x01;
	Data[6] = 0xFF;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO1映射0x60FF 01，左电机目标速度
	delay_ms(5);

	//事件触发
	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x14;
	Data[3] = 0x02;
	Data[4] = 0xFE;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO1事件触发
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x16;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO1开启1个映射
	delay_ms(5);

	return 0x00;
}




/**********************************************************
* 函数功能： RPDO2事件触发。
* 参数：     ID代表节点地址。
* 说明：     RPDO2映射0x60FF 02（右电机目标速度）；
*            RPDO2-COB-ID:0x400 + ID
**********************************************************/
uint8_t RPDO2_Config(uint8_t ID)
{
	uint8_t Data[8];

	//清除映射
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
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2映射0x60FF 02
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x14;
	Data[3] = 0x02;
	Data[4] = 0xFE;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2事件触发
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x16;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2开启1个映射
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* 函数功能： RPDO2事件触发。
* 参数：     ID代表节点地址。
* 说明：     RPDO2映射0x607A 00（电机目标位置）；
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
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2事件触发
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x02;
	Data[2] = 0x16;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x00;
	Data[6] = 0x7A;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2映射0x607A 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x16;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//RPDO2开启1个映射
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* 函数功能： TPDO0定时器触发。
* 参数：     ID代表节点地址。
* 说明：     TPDO0映射0x606C 01（左侧反馈速度）；
*            TPDO0定时器100ms；
*            TPDO0-COB-ID:0x180 + ID
**********************************************************/
uint8_t TPDO0_Config(uint8_t ID)
{
	uint8_t Data[8];

	//清除映射
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
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO0定时器触发
	delay_ms(5);

	Data[0] = 0x2B;
	Data[1] = 0x00;
	Data[2] = 0x18;
	Data[3] = 0x05;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO0定时器200*0.5ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x00;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x01;
	Data[6] = 0x6C;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO0映射0x606C 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x00;
	Data[2] = 0x1A;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO0开启1个映射
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* 函数功能： TPDO1定时器触发。
* 参数：     ID代表节点地址。
* 说明：     TPDO1映射0x606C 02（右侧反馈速度）；
*            TPDO1定时器100ms；
*            TPDO1-COB-ID:0x280 + ID
**********************************************************/
uint8_t TPDO1_Config(uint8_t ID)
{
	uint8_t Data[8];

	//清除映射
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
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO1定时器触发
	delay_ms(5);

	Data[0] = 0x2B;
	Data[1] = 0x01;
	Data[2] = 0x18;
	Data[3] = 0x05;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDOA定时器200*0.5ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x01;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x02;
	Data[6] = 0x6C;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO1映射0x6077 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x01;
	Data[2] = 0x1A;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO1开启1个映射
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* 函数功能： TPDO2定时器触发。
* 参数：     ID代表节点地址。
* 说明：     TPDO2映射0x6064 00（电机反馈位置）；
*            TPDO2定时器时间100ms；
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
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO2事件触发
	delay_ms(5);

	Data[0] = 0x2B;
	Data[1] = 0x02;
	Data[2] = 0x18;
	Data[3] = 0x05;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO2定时器200*0.5ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x02;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x20;
	Data[5] = 0x00;
	Data[6] = 0x64;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO2映射0x6064 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x02;
	Data[2] = 0x1A;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO2开启1个映射
	delay_ms(5);

	return 0x00;
}

/**********************************************************
* 函数功能： TPDO3事件触发。
* 参数：     ID代表节点地址。
* 说明：     TPDO3映射0x603F 00（故障码）；
*            TPDO0禁止时间500ms；
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
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO3事件触发
	delay_ms(5);

	Data[0] = 0x2B;
	Data[1] = 0x03;
	Data[2] = 0x18;
	Data[3] = 0x03;
	Data[4] = 0xE8;
	Data[5] = 0x03;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO3禁止时间1000*0.5ms
	delay_ms(5);


	Data[0] = 0x23;
	Data[1] = 0x03;
	Data[2] = 0x1A;
	Data[3] = 0x01;
	Data[4] = 0x20;        // 四个字节，对应32位
	Data[5] = 0x00;
	Data[6] = 0x3F;
	Data[7] = 0x60;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO3映射0x603F 00
	delay_ms(5);

	Data[0] = 0x2F;
	Data[1] = 0x03;
	Data[2] = 0x1A;
	Data[3] = 0x00;
	Data[4] = 0x01;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//TPDO3开启1个映射
	delay_ms(5);

	return 0x00;
}



/**********************************************************
* 函数功能： NMT节点状态设置，对节点使能、状态切换。
* 参数：     ID代表节点地址；data0为功能选择，data1为节点地址。
* 说明：     激活驱动PDO传输
**********************************************************/
uint8_t NMT_Control(uint8_t ID, const uint8_t Data0, const uint8_t Data1)
{
	uint8_t Data[2] = { 0 };
	Data[0] = Data0;
	Data[1] = Data1;

	return CAN_Send(0x000, 0x02, Data);
}



/**********************************************************
* 函数功能： 设置电机目标速度，根据canopen协议步骤初始化
* 参数：     ID代表节点地址
* 说明：     0x200 + 节点id代表的是PDO0的功能码 + Node_id
**********************************************************/
uint8_t Driver_Enable(uint8_t ID)
{
	uint8_t Data_2Byte[2];
	//	//清除故障
	//	uint8_t Data[8] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
	//	CAN_Send(0x600 + ID, 0x08, Data); //0x600+ID代表的是从机的地址（驱动器地址），默认是0x604


	//上电对驱动器初始化操作
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
* 函数功能： 电机速度模式初始化。
* 参数：     ID代表从机节点地址。
* 说明：     0x600 代表的是SDO通信，
* 		     设置电机的加速和减速时间。
**********************************************************/
uint8_t Profile_Velocity_Init(uint8_t ID)
{
	uint8_t Data[8] = { 0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00 };//设置速度模式
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	//设置为异步传输数据
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
	CAN_Send(0x600 + ID, 0x08, Data);//设置左侧电机起始速度
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x0D;
	Data[2] = 0x20;
	Data[3] = 0x02;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//设置右侧电机起始速度
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x83;
	Data[2] = 0x60;
	Data[3] = 0x01;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//设置左侧电机加速时间200ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x83;
	Data[2] = 0x60;
	Data[3] = 0x02;
	Data[4] = 0xC8;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//设置右侧电机加速时间200ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x84;
	Data[2] = 0x60;
	Data[3] = 0x01;
	Data[4] = 0x64;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//设置左侧电机减速时间100ms
	delay_ms(5);

	Data[0] = 0x23;
	Data[1] = 0x84;
	Data[2] = 0x60;
	Data[3] = 0x02;
	Data[4] = 0x64;
	Data[5] = 0x00;
	Data[6] = 0x00;
	Data[7] = 0x00;
	CAN_Send(0x600 + ID, 0x08, Data);//设置右侧电机减速时间100ms
	delay_ms(5);

	//

	return 0x00;
}


/**********************************************************
* 函数功能： 设置电机目标速度，
* 参数：     ID代表节点地址, nVelocity代表设置的速度
* 说明：     0x300 + 节点id代表的是PDO1的功能码 + Node_id
**********************************************************/
void Profile_Velocity_Set(uint8_t ID, int nLeft_Velocity, int nRight_Velocity)
{
	uint8_t Data_4Byte[4];

	//将整型的数据转换为4个字节的unsigned char的目标速度，小尾存储
	memcpy(Data_4Byte, &nLeft_Velocity, 4);

	CAN_Send(0x300 + ID, 0x04, Data_4Byte);//100rpm   左侧电机
	delay_ms(5);
	memcpy(Data_4Byte, &nRight_Velocity, 4);
	CAN_Send(0x400 + ID, 0x04, Data_4Byte);//100rpm   右侧电机
}


/**********************************************************
* 函数功能： 小车急停设置
* 参数：     ID代表节点地址
* 说明：     0x800 + 节点id代表的是SDO的功能码 + Node_id
**********************************************************/
void Emergecy_Stop(uint8_t ID)
{
	//急停，电机停止并保持锁轴状态
	uint8_t Data[8] = { 0x2B, 0x40, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00 };//急停清除目标速度
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);
}


/**********************************************************
* 函数功能： 速度模式解除急停
* 参数：     ID代表节点地址
* 说明：     0x800 + 节点id代表的是SDO的功能码 + Node_id
**********************************************************/
void Release_Stop_Vel(uint8_t ID)
{
	//驱动使能
	uint8_t Data[8] = { 0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00 };//设置速度模式
	CAN_Send(0x600 + ID, 0x08, Data);
	delay_ms(5);

	//设置目标转速
	Profile_Velocity_Set(ID, 0, 0);
}


/**********************************************************
* 函数功能： 心跳包机制函数
* 参数：     ID代表节点地址
* 说明：     设置对象字典1017h中从站产生心跳的时间；设置对象
*            字典1016h中主站检查心跳消费时间。定期检查产生心跳。
**********************************************************/
void Set_Heartbeat_Pack(uint8_t ID)
{
	uint8_t Data[8] = { 0x00 };//设置心跳产生时间

	Data[0] = 0x2B;
	Data[1] = 0x17;
	Data[2] = 0x10;
	Data[3] = 0x00;
	Data[4] = 0xE8;
	Data[5] = 0x03;
	Data[6] = 0x00;
	Data[7] = 0x00;

	// 一拖二心跳的COB-ID是 0x700 + Node-ID
	CAN_Send(0x600 + ID, 0x08, Data);//设置心跳产生的时间1000
	delay_ms(5);
}


/**********************************************************
* 函数功能： 让节点进入停止状态
* 参数：     ID代表节点地址
* 说明：     节点定期不回复心跳包，代表节点已经离线
*            通过NMT报文让节点进入停止状态。
**********************************************************/
void Release_ChildNode(uint8_t ID)
{
	//0x02代表停止命令
	NMT_Control(ID, 0x02, ID);

	//清除对应的标志位
	if (ID == MOTOR_CAN_ID)
	{
		g_nMotor_config = 0;
	}
}


/**********************************************************
* 函数功能： 让节点进入操作状态
* 参数：     ID代表节点地址
* 说明：     从其他状态回到操作状态
*
**********************************************************/
void Recover_ToStart(uint8_t ID)
{
	//0x01代表启动命令
	NMT_Control(ID, 0x01, ID);
}

/**********************************************************
* 函数功能： 重新初始化电机
* 说    明： 在电机断电后重新上电时需要重新初始化电机；
*            先给控制板上电，然后给电机接电时，需要重新初始化。
**********************************************************/
void ReInitalMotor(void)
{
	if (g_nReInitMotor == 1)
	{
		printf("Reinit LB……\r\n");
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
* 函数功能： 根据心跳判断电机是否下线
* 说    明： 从机隔一段时间未回复心跳，则代表下线
**********************************************************/
void Is_Offline(void)
{
	//检测的时长为150个show.cpp任务执行的周期，3s
	if (g_nHeart_Time > 150)
	{
		if (eMotor_Heart != STATE_STOP)
		{
			//左后方电机
			if (g_nHeart_count < 3 || (g_nHeart_count == g_nHeart_Lastcount))
			{
				//10s内未收到3次回包
				eMotor_Heart = STATE_STOP;
				g_nMotor_config = 0;
				g_nHeart_count = 0;
			}
		}

		g_nHeart_Time = 0;

		// 将当前的计数值赋值给上一次计数变量，为了判断驱动器是否离线
		g_nHeart_Lastcount = g_nHeart_count;
	}
}

/**********************************************************
* 函数功能： 清除驱动器产生的故障（有几类是不能清除的）
* 说    明： 只要驱动器发生故障，在中断中检测出来，
			 主函数中调用这个函数来达到清楚故障的目的
**********************************************************/
void Clear_Error_SendData(uint8_t ID)
{
	//	//清除故障
	uint8_t Data[8] = { 0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00 };
	CAN_Send(0x600 + ID, 0x08, Data); //0x600+ID代表的是从机的地址（驱动器地址），默认是0x604

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

		// 复位电机的状态
		g_nMotor_config = 0;
		eMotor_Heart = STATE_STOP;
		g_nReInitMotor = 1;

		Clear_Error_SendData(0x04);
		eMotor_Error = ERROR_NONE;
		// 清除故障之后，要重新使能电机
		ReInitalMotor();
	}
}


