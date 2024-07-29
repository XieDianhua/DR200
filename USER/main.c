#include "system.h"

#define START_TASK_PRIO			4
#define START_STK_SIZE 			256  

TaskHandle_t StartTask_Handler;

void start_task(void* pvParameters);

int main(void)
{
  BSP_Init();//Ӳ����ʼ��

  xTaskCreate((TaskFunction_t)start_task,//������
    (const char*)"start_task",//��������
    (uint16_t)START_STK_SIZE,//����ջ��С
    (void*)NULL,//���ݸ��������Ĳ���
    (UBaseType_t)START_TASK_PRIO,//�������ȼ�
    (TaskHandle_t*)&StartTask_Handler);//������

  vTaskStartScheduler();
}

void start_task(void* pvParameters)
{
  taskENTER_CRITICAL();//�����ٽ���

  xTaskCreate(motionControlTask, "motionControlTask", BALANCE_STK_SIZE, NULL, MOTION_CONTROL_TASK_PRIO, NULL);
  xTaskCreate(showTask, "showTask", SHOW_STK_SIZE, NULL, SHOW_TASK_PRIO, NULL);
  xTaskCreate(ledTask, "ledTask", LED_STK_SIZE, NULL, LED_TASK_PRIO, NULL);
  xTaskCreate(communicationTask, "communicationTask", COMMUNICATION_STK_SIZE, NULL, COMMUNICATION_TASK_PRIO, NULL);

  vTaskDelete(StartTask_Handler);
  taskEXIT_CRITICAL();//�˳��ٽ���
}



