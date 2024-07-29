#include "show.h"
#include "user_can.h"

int Voltage_Show = 0;          //电压显示全局变量

/**************************************************************************
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void showTask(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	//static int nFlag = 0;                //启动标志位，上电后10s才会执行后面的控制任务程序
	const TickType_t xPeriod = pdMS_TO_TICKS(20);
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, xPeriod);//此任务以50Hz的频率运行

		g_nHeart_Time++;

		Voltage_All += Get_battery_volt();     //多次采集电压数据累积
		if (++Voltage_Count == 20)
		{
			// 采集20次电池电压数据,多次采集求平均值
			Voltage = Voltage_All / 20;
			Voltage_All = 0;
			Voltage_Count = 0;
			// 计算出电池电压的占比
			g_fltProprity_Voltage = Voltage / Battery_Max;
			g_nVol_get_Flag = 1;             // 进行控制的标志位
		}
	}
}
