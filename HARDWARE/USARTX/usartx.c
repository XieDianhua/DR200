#include "usartx.h"
#include "user_can.h"

unsigned char remote_flag = 0;              //航模开启标志位
unsigned char ros_flag = 0;                 // ROS上位机接入标志位
/*--------上下位机通信格式数据usart2-----------*/
SEND_DATA send_data_;//发送数据的结构体
RECEIVE_DATA receive_data;//接收数据的结构体

/*--------航模数据解析相关全局变量的定义-----------*/
SBUS_CHANNEL_STRUCT sbus_channel_data_transformed_;
unsigned char usart5_recive_buffer_[MAX_BUFF_LEN] = { 0 };
int control_speed_level_ = 0;       //前后加速速度                   
int control_director_level_ = 0;       //转向
unsigned char g_ucLightOnFlag = 0; //车灯开启标志
float recive_robot_speed_x_ = 0.0;       // 串口接收到的速度数据
float recive_robot_speed_y_ = 0.0;
float recive_robot_speed_z_ = 0.0;

/**************************************************************************
函数功能：串口收发收据任务函数,串口2发送数据到上位机，中断接收数据
入口参数：无
返 回 值：无
**************************************************************************/
void communicationTask(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	const TickType_t xPeriod = pdMS_TO_TICKS(50);
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, xPeriod);//此任务以20Hz的频率运行
		parseRemoteData();//解析遥控器数据
		getToSendData(); //对要进行发送的数据进行赋值
		usart2SendData(); //串口2(ROS)发送数据
	}
}
/**************************************************************************
函数功能：串口2初始化，作为上下位机通信
入口参数：bound为通信波特率
是在PCB板中的第二个USB接口
返 回 值：无
**************************************************************************/
void usart2Init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);// 需要使能AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART2时钟
	//USART_TX  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //A2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//USART_RX	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//A3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//UsartNVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;//抢占优先级，中断优先级最高
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure);     //初始化串口3
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART2, ENABLE);                    //使能串口3 
}

/**************************************************
* 函数功能：串口5初始化函数，作为航模信号接收
* 参    数：unBound代表通信波特率，需为100k
* 返 回 值：无
**************************************************/
void usart5Init(unsigned int unBound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);	//使能USART2，GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

	//USART5_TX   GPIOC.12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //PC12
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC.12

	//USART5_RX	  GPIOD.2初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PD2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOD.2  

	//USART2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = unBound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收发模式

	USART_Init(UART5, &USART_InitStructure); //初始化串口5
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启串口接受中断
	//USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	USART_Cmd(UART5, ENABLE);                //使能串口5 
}

/**************************************************
* 函数功能：信号接收中断处理函数
* 入口参数：无
* 返 回 值：无
**************************************************/
void UART5_IRQHandler()
{
	static int index_count = 0;

	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //中断产生 
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE); //清除中断标志
		usart5_recive_buffer_[index_count++] = USART_ReceiveData(UART5);     //接收串口5数据到buff缓冲区
		if (usart5_recive_buffer_[index_count - 1] == 0x00 || index_count == MAX_BUFF_LEN)    //如果接收到尾标识是换行符（或者等于最大接受数就清空重新接收）
		{
			if (usart5_recive_buffer_[0] == 0x0f)                      //检测到头标识是我们需要的 
			{
				sbusTransform(usart5_recive_buffer_);
				index_count = 0;
			}
			else
			{
				index_count = 0;                                   //不是我们需要的数据或者达到最大接收数则开始重新接收
			}
		}
	}
}

/**************************************************************************
函数功能：串口2接收中断,接收上位机发送过来的数据
入口参数：无
返 回 值：无
**************************************************************************/
void USART2_IRQHandler(void)
{
	static unsigned char count = 0;
	unsigned char usart2_receive;

	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //判断是否接收到数据
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE); //清除中断标志
		ros_flag = 1;                        // 航模开启的时候，给这个变量赋值为0
		if (remote_flag == 0 && ros_flag == 1)//TODO:更改为||，可以在开启遥控器的同时，变为上位机控制
		{
			robot_control_mode_ = CONTROL_MODE_ROS;
		}
		usart2_receive = USART_ReceiveData(USART2);//读取数据
		receive_data.buffer[count] = usart2_receive;
		if (usart2_receive == FRAME_HEADER || count > 0)
		{
			count++;
		}
		else
		{
			count = 0;
		}
		if (count == 11)	//验证数据包的长度
		{
			count = 0;//重新开始接收
			if (receive_data.buffer[10] == FRAME_TAIL) //验证数据包的尾部校验信息
			{
				if (receive_data.buffer[9] == checkSum(9, 0))	 //数据校验位计算，模式0是发送数据校验
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
函数功能：串口2(ROS)发送数据
入口参数：无
返回  值：无
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
函数功能：串口2发送单字节
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2SendByte(unsigned char data)
{
	USART2->DR = data;
	while ((USART2->SR & 0x40) == 0);
}

/**************************************************************************
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void getToSendData(void)
{
	send_data_.SENSOR_STR.frame_header = FRAME_HEADER; //帧头
	send_data_.SENSOR_STR.frame_tail = FRAME_TAIL; //帧尾

	//获取反馈速度
	getMotorSpeed();

	// 运动学正解 通过轮子速度解算底盘速度
	send_data_.SENSOR_STR.car_x = ((motor_left_.line_speed_feedback + motor_right_.line_speed_feedback) / 2) * 1000; //小车x轴速度
	send_data_.SENSOR_STR.car_y = 0;
	send_data_.SENSOR_STR.car_z = ((motor_right_.line_speed_feedback - motor_left_.line_speed_feedback) / WHEELSPACING) * 1000;//小车z轴速度

	send_data_.SENSOR_STR.battery_voltage = Voltage * 1000; //电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)

	send_data_.buffer[0] = send_data_.SENSOR_STR.frame_header; //帧头(固定值)

	send_data_.buffer[2] = (send_data_.SENSOR_STR.car_x >> 8) & 0XFF; //小车x轴速度
	send_data_.buffer[3] = send_data_.SENSOR_STR.car_x & 0XFF;    //小车x轴速度
	send_data_.buffer[4] = (send_data_.SENSOR_STR.car_y >> 8) & 0XFF;  //小车y轴速度
	send_data_.buffer[5] = send_data_.SENSOR_STR.car_y & 0XFF;     //小车y轴速度
	send_data_.buffer[6] = (send_data_.SENSOR_STR.car_z >> 8) & 0XFF; //小车z轴速度
	send_data_.buffer[7] = send_data_.SENSOR_STR.car_z & 0XFF;    //小车z轴速度

	send_data_.buffer[20] = (send_data_.SENSOR_STR.battery_voltage >> 8) & 0XFF; //电池电压
	send_data_.buffer[21] = send_data_.SENSOR_STR.battery_voltage & 0XFF; //电池电压

	send_data_.buffer[22] = checkSum(22, 1); //数据校验位计算，模式1是发送数据校验
	send_data_.buffer[23] = send_data_.SENSOR_STR.frame_tail;//帧尾（固定值）
}

/**************************************************
* 函数功能：将sbus信号转换为通道值
* 参    数：ucBuf为接收到的串口数据
* 返 回 值：0代表成功，1代表失败
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
函数功能：计算发送的数据校验位
入口参数：23位为校验位，结果是数组1-22的异或结果；后一个参数为发送或是接收校验位
返 回 值：检验位
**************************************************************************/
u8 checkSum(unsigned char Count_Number, unsigned char Mode)
{
	unsigned char check_sum = 0, k;
	//发送数据的校验
	if (Mode == 1)
	{
		for (k = 0;k < Count_Number; k++)
		{
			check_sum = check_sum ^ send_data_.buffer[k];
		}
	}

	//接收数据的校验
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
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float parseTargetSpeed(u8 High, u8 Low)
{
	short transition; //数据转换的中间变量
	transition = ((High << 8) + Low); //将高8位和低8位整合成一个16位的short型数据
	return transition / 1000 + (transition % 1000) * 0.001;    //发送端将数据发送前做了一个*1000的单位换算，这里接收数据后需要还原单位
}

#if 1
/**************************************************
* 函数功能：获取遥控器设定的速度和方向值，解算速度时需要用到
* 输出：g_nVelocity和g_nDirector的值
**************************************************/
void parseRemoteData()
{
	int nTemp = 0;  //临时变量
	int nSwitchSpeed = 0;

	//判断SWA是否打开
	nTemp = sbus_channel_data_transformed_.channel_7;

	if (absInt(nTemp - TURN_OFF_REMOOT) < 10)
	{
		//开关处于关闭状态
		remote_flag = 0;                   //航模关闭标志位	
		robot_control_mode_ = CONTROL_MODE_UNKNOW; //未知模式
		control_speed_level_ = 0;
		return;
	}
	if (absInt(nTemp - TURN_ON_REMOOT) < 10)
	{
		//开关处于打开状态
		remote_flag = 1;                   // 航模开启标志位
		ros_flag = 0;                      // 航模开启，给与最高权限
		robot_control_mode_ = CONTROL_MODE_REMOTE; //航模遥控方式选择

		nTemp = sbus_channel_data_transformed_.channel_6;

		//此时判断是使用低、中、高速度挡位
		if (absInt(nTemp - SPEED_LEVEL1) < 10)
		{
			nSwitchSpeed = getTargetSpeed(sbus_channel_data_transformed_.channel_3);//获取前进后退方向
			//低速档
			if (nSwitchSpeed > 0)
			{
				//前进
				if (nSwitchSpeed == 1)
					control_speed_level_ = SPEED_LOW;
				else
					control_speed_level_ = SPEED_LOW * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else if (nSwitchSpeed < 0)
			{
				//后退
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
			//中速挡
			nSwitchSpeed = getTargetSpeed(sbus_channel_data_transformed_.channel_3);
			if (nSwitchSpeed > 0)
			{
				//正传达到最大速度
				if (nSwitchSpeed == 1)
					control_speed_level_ = SPEED_MIDDLE;
				else
					control_speed_level_ = SPEED_MIDDLE * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else if (nSwitchSpeed < 0)
			{
				//反转达到最大速度
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
			//高速挡
			nSwitchSpeed = getTargetSpeed(sbus_channel_data_transformed_.channel_3);
			if (nSwitchSpeed > 0)
			{
				//正传达到最大速度
				if (nSwitchSpeed == 1)
					control_speed_level_ = SPEED_HIGHT;
				else
					control_speed_level_ = SPEED_HIGHT * (float)(sbus_channel_data_transformed_.channel_3 - VEL_BASE_VALUE) / 784;
			}
			else if (nSwitchSpeed < 0)
			{
				//反转达到最大速度
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

	// 转角速度获取
	setDirector();
}
#endif

/**************************************************
* 函数功能：对整型数据取绝对值
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
* 函数功能：处理电机正转还是反转函数
* 返 回 值：返回正负分别代表正反转
**************************************************/
int getTargetSpeed(unsigned short usValue)
{
	int nValue = 0;

	//先进行速度限幅设定，并获取速度
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
* 函数功能：处理电机正转还是反转函数
* 返 回 值：返回正负分别代表正反转
**************************************************/
int getTargetDirect(unsigned short usValue)
{
	int nValue = 0;

	//先进行角度的限幅设定，并获取角度
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

	int nTemp = 0;  //临时变量
	int nSwitchDirect = 0;  //转弯方向的选择

	//判断SWA是否打开
	nTemp = sbus_channel_data_transformed_.channel_7;
	if (absInt(nTemp - TURN_OFF_REMOOT) < 10)
	{
		//开关处于关闭状态
		control_director_level_ = 0;
		return;
	}

	else if (absInt(nTemp - TURN_ON_REMOOT) < 10)
	{
		//开关处于打开状态
		nTemp = sbus_channel_data_transformed_.channel_6;

		//此时判断是使用低、中、高速度挡位
		if (absInt(nTemp - SPEED_LEVEL1) < 10)
		{
			//低速档
			nSwitchDirect = getTargetDirect(sbus_channel_data_transformed_.channel_1);
			if (nSwitchDirect > 0)
			{
				//右转到最大
				if (nSwitchDirect == 1)
					control_director_level_ = SPEED_DIR_LOW;
				else
					control_director_level_ = SPEED_DIR_LOW * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else if (nSwitchDirect < 0)
			{
				//左转到最大
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
			//中速挡
			nSwitchDirect = getTargetDirect(sbus_channel_data_transformed_.channel_1);
			if (nSwitchDirect > 0)
			{
				//右传到最大
				if (nSwitchDirect == 1)
					control_director_level_ = SPEED_DIR_MIDDLE;
				else
					control_director_level_ = SPEED_DIR_MIDDLE * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else if (getTargetDirect(sbus_channel_data_transformed_.channel_1) < 0)
			{
				//左转到最大
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
			//高速挡
			nSwitchDirect = getTargetDirect(sbus_channel_data_transformed_.channel_1);
			if (nSwitchDirect > 0)
			{
				//右转到最大
				if (nSwitchDirect == 1)
					control_director_level_ = SPEED_DIR_HIGHT;
				else
					control_director_level_ = SPEED_DIR_HIGHT * (float)(sbus_channel_data_transformed_.channel_1 - DIR_BASE_VALUE) / 784;
			}
			else if (getTargetDirect(sbus_channel_data_transformed_.channel_1) < 0)
			{
				//左转到最大
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


