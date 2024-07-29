#include "MotionControl.h"
#include "user_can.h"
#include "led.h"

struct MOTOR_PARAMETER  motor_left_, motor_right_;
float robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;
enum ROBOT_CONTROL_MODE robot_control_mode_ = CONTROL_MODE_UNKNOW;  // �����˵Ŀ��Ʒ�ʽ
/**************************************************************************
�������ܣ����Ŀ������
��ڲ�����
����  ֵ��
**************************************************************************/
void motionControlTask(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt(); //��ȡ��ϵͳ�ϴ�ִ��ʱ��
	const TickType_t xPeriod = pdMS_TO_TICKS(10);
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, xPeriod); //��������100Hz��Ƶ�����У�10ms����һ�Σ�
		setRobotSpeed();//���ø�ģʽ�»������ٶ�
		inverseSolutionKinematics(robot_x_, robot_y_, robot_z_);//���ݻ������ٶȵõ�������ٶ�   
		setMotorSpeed(MOTOR_CAN_ID, -motor_left_.rotate_speed_target, motor_right_.rotate_speed_target);
	}
}

void setRobotSpeed(void)
{
	//ʹ����λ�����Ƶ�ʱ�򣬻���usart�ж��йر�Remote_ON_Flag��־λ
	if (robot_control_mode_ == CONTROL_MODE_REMOTE)
	{
		//��ģ��ȡ���趨���ٶ�
		//�Ժ�ģ����������ֵת�������ٶȺͽ��ٶ�
		robot_x_ = ((float)control_speed_level_ * 2 * PI * RADIAUS) / 60 / REDUCTION_RATE;    //��λΪm/s
		robot_y_ = 0;
		robot_z_ = -((float)control_director_level_) / DIRECTOR_BASE * (PI / 4);
	}
	else if (robot_control_mode_ == CONTROL_MODE_ROS)
	{
		//��λ�����Ƶ�ʱ�򣬴�ʱZ����ٶ�Ҫ����һ�����򣬲�Ȼ����ת���ʱ��������
		robot_x_ = recive_robot_speed_x_;
		robot_y_ = recive_robot_speed_y_;
		robot_z_ = recive_robot_speed_z_;    // ��Ҫ����Z�����ݣ������з��ֵ�
	}
	else
	{
		robot_x_ = 0;
		robot_y_ = 0;
		robot_z_ = 0;
	}
}
/**************************************************************************
�������ܣ��Խ��յ������˶��ٶ����õ������ٶ�
��ڲ�����X��Y Z�᷽����˶��ٶ�
�����Ŀ���ٶ�
**************************************************************************/
void inverseSolutionKinematics(float robot_x, float robot_y, float robot_z)
{
	motor_left_.line_speed_target = robot_x - robot_z * WHEELSPACING / 2; //���������Ŀ���ٶ�
	motor_right_.line_speed_target = robot_x + robot_z * WHEELSPACING / 2; //���������Ŀ���ٶ�

	// �ٶȵ��޷��ȣ������챵�����ٶȲ��ܳ���200 r/min
	motor_left_.rotate_speed_target = speedLimit(lineToRotate(motor_left_.line_speed_target));
	motor_right_.rotate_speed_target = speedLimit(lineToRotate(motor_right_.line_speed_target));
}

/**************************************************************************
�������ܣ���ȡ2������ٶȡ�����can�жϣ���ȡ�õ�2��������ٶȣ�����ڶ�Ӧ��Motor�ṹ���С�r/min
����������ٶ�
**************************************************************************/
void getMotorSpeed()
{
	//����ɼ����������������������������ݣ�Ҫ������Ӧת������
	motor_left_.rotate_speed_feedback = -uVelLeft.nVelcity * 0.1;//�������ǰΪ�������ԭʼ���ٶȵ�λ0.1r/min
	motor_right_.rotate_speed_feedback = uVelRight.nVelcity * 0.1;

	// ת��ת�����ٶ� m/s
	motor_left_.line_speed_feedback = rotateToLine(motor_left_.rotate_speed_feedback);
	motor_right_.line_speed_feedback = rotateToLine(motor_right_.rotate_speed_feedback);
}

/**************************************************************************
�������ܣ�ת�ٵ��޷�
��ڲ�����Բ�̵���챵�����ٶ�������200 r/min����
����  ֵ���޷��Ⱥ���ٶ�
**************************************************************************/
int speedLimit(int rotateData)
{
	if (rotateData >= 200)
	{
		return 200;
	}
	else if (rotateData <= -200)
	{
		return -200;
	}
	else
	{
		return rotateData;
	}
}
/**************************************************************************
�������ܣ��������ת�٣�r/minת�����������ٶ� m/s��
					������ٶȽṹ���С�
					1 r/min = ��2�� * R�� / 60  m/s
��ڲ�����nRotateSpeed����ת�٣�r/min
�� �� ֵ������ת��
**************************************************************************/
float rotateToLine(int nRotateSpeed)
{
	float fltReturn = 0.0f;
	fltReturn = nRotateSpeed * 2 * PI * RADIAUS / 60;  //ת��Ϊ��̥�ٶ�

	return fltReturn;
}

/**************************************************************************
�������ܣ������ӵ��ٶȣ�m/sת���ɵ����ת�� r/min��
					������ٶȽṹ���С�
					1 m/s = 60 /��2 * PI * R��
��ڲ�����fltSpeed��������ת�٣�m/s
�� �� ֵ��������ת��
**************************************************************************/
int lineToRotate(float fltSpeed)
{
	int nReturn = 0;
	nReturn = (60 * fltSpeed / (2 * PI * RADIAUS)) * REDUCTION_RATE;

	return nReturn;
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
unsigned char turnOff(int voltage)
{
	unsigned char ucTemp;
	if (voltage < 10)//��ص�ѹ���͹رյ��
	{
		//���õ��Ŀ���ٶ�Ϊ0����ʧ�ܵ��
		ucTemp = 1;
		setMotorSpeed(MOTOR_CAN_ID, 0, 0);   //����Ŀ�������ٶ�
	}
	else
	{
		ucTemp = 0;
	}
	return ucTemp;
}

/**************************************************************************
�������ܣ�����2������ٶ�
��ڲ����������ֱ��������������ҵ����Ӧ��can�ӻ�id�͵���ٶ�
�� �� ֵ����
**************************************************************************/
void setMotorSpeed(int nID, int nMotorLeft_Speed, int nMotorRight_Speed)
{
	Clear_Error();            //�й��ϵĻ������
	ReInitalMotor();          //���Ƿ�����Ҫ���³�ʼ���ĵ��
	Is_Offline();
	if (g_nMotor_config == 1 && eMotor_Heart == STATE_START)
	{
		Profile_Velocity_Set(nID, nMotorLeft_Speed, nMotorRight_Speed);
	}
	else if (eMotor_Heart == STATE_STOP)
	{
		Release_ChildNode(nID);
	}
	else if (g_nMotor_config == 1 && eMotor_Heart == STATE_PRE_OPERATE)
	{
		Recover_ToStart(nID);
	}
}


