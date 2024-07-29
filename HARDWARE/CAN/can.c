#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "usartx.h"

int g_nMotor_config = -1;                           //驱动器是否配置
union URcv_Vel_Data uVelLeft;
union URcv_Vel_Data uVelRight;                      //存放左右实时速度的变量
int g_nReInitMotor = 0;                             //处理驱动器断电而开发板未断电，重新初始化驱动器的问题,为1代表需要重新初始化，为0则代表不需要重新初始化
int g_nHeart_count = 0;                             //设置了心跳包，在规定次数内没产生心跳，则认为从机下线,将其状态设置为STATE_STOP
int g_nHeart_Lastcount = 0;                         //记录上次进入心跳包的计数值
enum ENUM_HEART_STAT eMotor_Heart = STATE_UNKNOW;
enum ENUM_ERROR_STATE eMotor_Error = ERROR_NONE;    // 驱动器故障记录
union URcv_ERROR_Data uError;                       // 接收驱动器的故障状态

int g_nState_debug = -1;                            // 电机心跳包状态调试

/**********************************************************
* 函数功能： stm32底层的can通信协议初始化。
* 参数：     无
* 说明：     无
**********************************************************/
void canDriverInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

#if CAN_RX0_INT_ENABLE 
	NVIC_InitTypeDef  		NVIC_InitStructure;
#endif

	//对应的GPIO口和时钟初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //使能CAN时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //使能AFIO时钟

	//需要将CAN功能复用，映射到PD0和PD1上
	GPIO_PinRemapConfig(GPIO_Remap2_CAN1, ENABLE);

	//初始化用到的GPIO口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;  //PD0作为Rx 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入模式
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  //PD1作为Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出,使用复用CAN功能
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//工作模式、波特率初始化
	//CAN单元设置
	CAN_InitStructure.CAN_TTCM = DISABLE; //非时间出发通信模式  
	CAN_InitStructure.CAN_ABOM = DISABLE; //软件自动离线管理   
	CAN_InitStructure.CAN_AWUM = DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART = ENABLE;  //使用报文自动传送
	CAN_InitStructure.CAN_RFLM = DISABLE; //报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP = DISABLE; //优先级由报文标识符决定
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;    //模式设置
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;  //
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 12;
	CAN_Init(CAN1, &CAN_InitStructure);

	//设置CAN筛选器
	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber = 0; //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //工作在标识符屏蔽模式下
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32位
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;////32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //使能过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//过滤器初始化

	//开启CAN中断
#if CAN_RX0_INT_ENABLE 

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);				//FIFO0消息挂号中断允许.	接收到数据就会产生中断    
	//CAN_ClearFlag(CAN1,CAN_IT_FMP0);
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif	
}

/**********************************************************
* 函数功能： can中断服务函数
* 参数：     无
* 说明：     在中断函数中获取到实时反馈的速度
**********************************************************/
#if CAN_RX0_INT_ENABLE	//使能RX0中断函数
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;

	CAN_Receive(CAN1, 0, &RxMessage);
	/*- - - - - - - - - - - - -  - - - 从机上线或者接收从机心跳，或者是否需要重新初始化 - - - - - - - - - - - - - - - */
		//左后方电机
	if (RxMessage.StdId == 0x704)  //左侧电机心跳包接收中断
	{
		g_nState_debug = RxMessage.Data[0];
		g_nHeart_count++;  //心跳包计数

		/*--------心跳包状态反馈---------------*/
		if (RxMessage.Data[0] == 0x04)
		{
			eMotor_Heart = STATE_STOP;
		}
		else if (RxMessage.Data[0] == 0x05)
		{
			g_nMotor_config = 1;
			eMotor_Heart = STATE_START;
		}
		else if (RxMessage.Data[0] == 0x7F)
		{
			eMotor_Heart = STATE_PRE_OPERATE;
		}

		/*--------处理中途出现的需要对驱动器重新初始化的情况---------------*/
		//控制板先上电，电机后上电，需要重新初始化一次电机
		if (eMotor_Heart == STATE_UNKNOW && g_nMotor_config == -1)
		{
			g_nReInitMotor = 1;
		}
		//中途驱动器断电，然后再次重新上电
		if (g_nMotor_config == 0)
		{
			//处理开发板未断电，驱动器重新上电的问题
			g_nReInitMotor = 1;
		}
	}

	/*- - - - - - - - - - - - -  - - - TPDO反馈速度- - - - - - - - - - - - - - - */
	/*- - - - 读取到得速度说明：读取到的速度得单位是0.1r/min,但是方向跟我们设定的相反 - - - -*/
		//左侧电机速度反馈，TPDO0设置的
	else if (RxMessage.StdId == 0x184)
	{
		//转换实时反馈速度，速度只有数组的低四位
		uVelLeft.ucData[0] = RxMessage.Data[0];
		uVelLeft.ucData[1] = RxMessage.Data[1];
		uVelLeft.ucData[2] = RxMessage.Data[2];
		uVelLeft.ucData[3] = RxMessage.Data[3];

	}
	//右侧电机速度反馈，TPDO0设置的
	else if (RxMessage.StdId == 0x284)
	{
		//转换实时反馈速度
		uVelRight.ucData[0] = RxMessage.Data[0];
		uVelRight.ucData[1] = RxMessage.Data[1];
		uVelRight.ucData[2] = RxMessage.Data[2];
		uVelRight.ucData[3] = RxMessage.Data[3];
	}


	/*- - - - - - - - - - - - -  - - - 驱动器故障反馈(TPDO3) - - - - - - - - - - - - - - - */
  /*- - - - 根据驱动器的故障，设置标志位，然后在后续程序中进行故障清除 - - - -*/
	/* 故障有：过压、欠压、过流、过载、电流超差、编码器超差、速度超差、参考电压出错、EEPROM读写出错、霍尔出错 */
	else if (RxMessage.StdId == 0x484)  //TPDO3设置的
	{
		uError.ucData[0] = RxMessage.Data[0];
		uError.ucData[1] = RxMessage.Data[1];
		if (uError.usError == 0x00)
		{
			eMotor_Error = ERROR_NONE;              // 无错误
		}
		else if (uError.usError == 0x01)
		{
			eMotor_Error = ERROR_OVER_VALUE;        // 过压
		}
		else if (uError.usError == 0x02)
		{
			eMotor_Error = ERROR_LESS_VALUE;        // 欠压
		}
		else if (uError.usError == 0x04)
		{
			eMotor_Error = ERROR_OVER_CURRENT;      // 过流
		}
		else if (uError.usError == 0x08)
		{
			eMotor_Error = ERROR_OVER_LOAD;         // 过载
		}
		else if (uError.usError == 0x10)
		{
			eMotor_Error = ERROR_OVERDIFF_CURRENT;  // 电流超差
		}
		else if (uError.usError == 0x20)
		{
			eMotor_Error = ERROR_OVERDIFF_ENCODER;  // 编码器超差
		}
		else if (uError.usError == 0x40)
		{
			eMotor_Error = ERROR_OVERDIFF_VELOCITY; // 速度超差
		}
		else if (uError.usError == 0x80)
		{
			eMotor_Error = ERROR_REF_VALUE;          // 参考电压出错
		}
		else if (uError.usError == 0x100)
		{
			eMotor_Error = ERROR_EEPROM;             // EEPROM读写错误
		}
		else if (uError.usError == 0x200)
		{
			eMotor_Error = ERROR_HALL;               // 霍尔出错
		}
	}

	CAN_ClearFlag(CAN1, CAN_IT_FMP0);

}
#endif

