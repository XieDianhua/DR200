#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define MOTION_CONTROL_TASK_PRIO 4 //�������ȼ�
#define BALANCE_STK_SIZE 		512   //�����ջ��С

#define VELOCITY_MULTI      3     //�ٶ�ֵ����
#define DIRECTOR_BASE       500   //Z����ٶȵĻ�׼


struct MOTOR_PARAMETER
{
    int rotate_speed_target;        //��λr/min,Ϊ�˱������������д��������
    int rotate_speed_feedback;

    float line_speed_target;    //������λ��ͨ�ţ�����������ӵ��ٶ�m/s
    float line_speed_feedback;
};

// ö�ٶ���С���Ŀ��Ʒ�ʽ
enum ROBOT_CONTROL_MODE
{
    CONTROL_MODE_UNKNOW = 0,    // δ֪����ģʽ
    CONTROL_MODE_REMOTE = 1,    // ��ģ���Ʒ�ʽ
    CONTROL_MODE_ROS = 2,    // ��λ������ģʽ
};

extern struct MOTOR_PARAMETER  motor_left_, motor_right_;//С���ٶȽṹ��
extern float robot_x_, robot_y_, robot_z_;   //С����������ٶ�
extern enum ROBOT_CONTROL_MODE robot_control_mode_;  //�����˵Ŀ��Ʒ�ʽ

/*----------------------���Ŀ��ƺ���--------------------------*/
void motionControlTask(void* pvParameters);           //������
void setRobotSpeed(void);
void inverseSolutionKinematics(float car_x, float car_y, float car_z);
void setMotorSpeed(int nID, int nMotorLeft_Speed, int nMotorRight_Speed);//����2������ٶ�
void getMotorSpeed(void);
unsigned char turnOff(int voltage);
float rotateToLine(int nRotateSpeed);    //���ת��ת��Ϊ�������ٶȣ�m/s
int lineToRotate(float fltSpeed);      //�����ٶ�ת��Ϊ���ת��r/min
int speedLimit(int rotateData);              // ת�ٵ��޷���������200 r/min

#endif  

