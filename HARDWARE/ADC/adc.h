#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#include "system.h"


#define Battery_Ch    14       // ADC��ͨ��14
#define Battery_Max   54.0f    // ���õ�ص�����״̬
#define Sample_time   10       // ���õ�ص����Ĳ�������
#define Potentiometer 8        // ��λ��ADCͨ��8

#define R16           47.0f    //470K
#define R17						2.0f     //20k    �ֱ���2����ѹ����


extern float Voltage, Voltage_All,Voltage_Count;    // �洢��ȡ���ĵ�ѹֵ 	
extern float g_fltProprity_Voltage;  // ��ص����ı���

extern int g_nVol_get_Flag;          // �Ƿ��Ѿ������õ���ѹ�ı�־λ

void Adc_Init(void);
u16 Get_Adc(u8 ch);
float Get_battery_volt(void) ;
u16 Get_adc_Average(u8 chn, u8 times);


#endif 


