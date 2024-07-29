#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define MOTION_CONTROL_TASK_PRIO 4 //任务优先级
#define BALANCE_STK_SIZE 		512   //任务堆栈大小

#define VELOCITY_MULTI      3     //速度值倍率
#define DIRECTOR_BASE       500   //Z轴角速度的基准


struct MOTOR_PARAMETER
{
    int rotate_speed_target;        //单位r/min,为了便于做电机控制写入驱动器
    int rotate_speed_feedback;

    float line_speed_target;    //做上下位机通信，传输的是轮子的速度m/s
    float line_speed_feedback;
};

// 枚举定义小车的控制方式
enum ROBOT_CONTROL_MODE
{
    CONTROL_MODE_UNKNOW = 0,    // 未知控制模式
    CONTROL_MODE_REMOTE = 1,    // 航模控制方式
    CONTROL_MODE_ROS = 2,    // 上位机控制模式
};

extern struct MOTOR_PARAMETER  motor_left_, motor_right_;//小车速度结构体
extern float robot_x_, robot_y_, robot_z_;   //小车各个轴的速度
extern enum ROBOT_CONTROL_MODE robot_control_mode_;  //机器人的控制方式

/*----------------------核心控制函数--------------------------*/
void motionControlTask(void* pvParameters);           //任务函数
void setRobotSpeed(void);
void inverseSolutionKinematics(float car_x, float car_y, float car_z);
void setMotorSpeed(int nID, int nMotorLeft_Speed, int nMotorRight_Speed);//设置2个电机速度
void getMotorSpeed(void);
unsigned char turnOff(int voltage);
float rotateToLine(int nRotateSpeed);    //电机转速转换为轮子线速度，m/s
int lineToRotate(float fltSpeed);      //轮子速度转换为电机转速r/min
int speedLimit(int rotateData);              // 转速的限幅，控制在200 r/min

#endif  

