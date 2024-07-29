#include "system.h"

#define START_TASK_PRIO			4
#define START_STK_SIZE 			256  

TaskHandle_t StartTask_Handler;

void start_task(void* pvParameters);

int main(void)
{
  BSP_Init();//硬件初始化

  xTaskCreate((TaskFunction_t)start_task,//任务函数
    (const char*)"start_task",//任务名称
    (uint16_t)START_STK_SIZE,//任务栈大小
    (void*)NULL,//传递给任务函数的参数
    (UBaseType_t)START_TASK_PRIO,//任务优先级
    (TaskHandle_t*)&StartTask_Handler);//任务句柄

  vTaskStartScheduler();
}

void start_task(void* pvParameters)
{
  taskENTER_CRITICAL();//进入临界区

  xTaskCreate(motionControlTask, "motionControlTask", BALANCE_STK_SIZE, NULL, MOTION_CONTROL_TASK_PRIO, NULL);
  xTaskCreate(showTask, "showTask", SHOW_STK_SIZE, NULL, SHOW_TASK_PRIO, NULL);
  xTaskCreate(ledTask, "ledTask", LED_STK_SIZE, NULL, LED_TASK_PRIO, NULL);
  xTaskCreate(communicationTask, "communicationTask", COMMUNICATION_STK_SIZE, NULL, COMMUNICATION_TASK_PRIO, NULL);

  vTaskDelete(StartTask_Handler);
  taskEXIT_CRITICAL();//退出临界区
}



