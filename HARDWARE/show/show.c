#include "show.h"
#include "user_can.h"

int Voltage_Show = 0;          //��ѹ��ʾȫ�ֱ���

/**************************************************************************
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void showTask(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	//static int nFlag = 0;                //������־λ���ϵ��10s�Ż�ִ�к���Ŀ����������
	const TickType_t xPeriod = pdMS_TO_TICKS(20);
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, xPeriod);//��������50Hz��Ƶ������

		g_nHeart_Time++;

		Voltage_All += Get_battery_volt();     //��βɼ���ѹ�����ۻ�
		if (++Voltage_Count == 20)
		{
			// �ɼ�20�ε�ص�ѹ����,��βɼ���ƽ��ֵ
			Voltage = Voltage_All / 20;
			Voltage_All = 0;
			Voltage_Count = 0;
			// �������ص�ѹ��ռ��
			g_fltProprity_Voltage = Voltage / Battery_Max;
			g_nVol_get_Flag = 1;             // ���п��Ƶı�־λ
		}
	}
}
