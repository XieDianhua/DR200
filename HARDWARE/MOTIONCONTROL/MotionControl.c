#include "MotionControl.h"
#include "user_can.h"
#include "led.h"

struct MOTOR_PARAMETER  motor_left_, motor_right_;
float robot_x_ = 0, robot_y_ = 0, robot_z_ = 0;
enum ROBOT_CONTROL_MODE robot_control_mode_ = CONTROL_MODE_UNKNOW;  // 机器人的控制方式
/**************************************************************************
函数功能：核心控制相关
入口参数：
返回  值：
**************************************************************************/
void motionControlTask(void* pvParameters)
{
	u32 lastWakeTime = getSysTickCnt(); //获取到系统上次执行时间
	const TickType_t xPeriod = pdMS_TO_TICKS(10);
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, xPeriod); //此任务以100Hz的频率运行（10ms控制一次）
		setRobotSpeed();//设置各模式下机器人速度
		inverseSolutionKinematics(robot_x_, robot_y_, robot_z_);//根据机器人速度得到各电机速度   
		setMotorSpeed(MOTOR_CAN_ID, -motor_left_.rotate_speed_target, motor_right_.rotate_speed_target);
	}
}

void setRobotSpeed(void)
{
	//使用上位机控制的时候，会在usart中断中关闭Remote_ON_Flag标志位
	if (robot_control_mode_ == CONTROL_MODE_REMOTE)
	{
		//航模获取到设定的速度
		//对航模解析到的数值转换成线速度和角速度
		robot_x_ = ((float)control_speed_level_ * 2 * PI * RADIAUS) / 60 / REDUCTION_RATE;    //单位为m/s
		robot_y_ = 0;
		robot_z_ = -((float)control_director_level_) / DIRECTOR_BASE * (PI / 4);
	}
	else if (robot_control_mode_ == CONTROL_MODE_ROS)
	{
		//上位机控制的时候，此时Z轴的速度要进行一个反向，不然左右转弯的时候有问题
		robot_x_ = recive_robot_speed_x_;
		robot_y_ = recive_robot_speed_y_;
		robot_z_ = recive_robot_speed_z_;    // 需要反向Z轴数据，调试中发现的
	}
	else
	{
		robot_x_ = 0;
		robot_y_ = 0;
		robot_z_ = 0;
	}
}
/**************************************************************************
函数功能：对接收到底盘运动速度逆解得到轮子速度
入口参数：X和Y Z轴方向的运动速度
输出：目标速度
**************************************************************************/
void inverseSolutionKinematics(float robot_x, float robot_y, float robot_z)
{
	motor_left_.line_speed_target = robot_x - robot_z * WHEELSPACING / 2; //计算出左轮目标速度
	motor_right_.line_speed_target = robot_x + robot_z * WHEELSPACING / 2; //计算出右轮目标速度

	// 速度的限幅度，这个轮毂电机的速度不能超过200 r/min
	motor_left_.rotate_speed_target = speedLimit(lineToRotate(motor_left_.line_speed_target));
	motor_right_.rotate_speed_target = speedLimit(lineToRotate(motor_right_.line_speed_target));
}

/**************************************************************************
函数功能：获取2个电机速度。根据can中断，获取得到2个电机的速度，存放在对应的Motor结构体中。r/min
输出：反馈速度
**************************************************************************/
void getMotorSpeed()
{
	//这里采集回来的是驱动器反馈回来的数据，要经过相应转换才行
	motor_left_.rotate_speed_feedback = -uVelLeft.nVelcity * 0.1;//左侧电机向前为负，电机原始的速度单位0.1r/min
	motor_right_.rotate_speed_feedback = uVelRight.nVelcity * 0.1;

	// 转速转换成速度 m/s
	motor_left_.line_speed_feedback = rotateToLine(motor_left_.rotate_speed_feedback);
	motor_right_.line_speed_feedback = rotateToLine(motor_right_.rotate_speed_feedback);
}

/**************************************************************************
函数功能：转速的限幅
入口参数：圆盘的轮毂电机，速度限制在200 r/min以下
返回  值：限幅度后的速度
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
函数功能：将电机的转速，r/min转换成轮子线速度 m/s，
					存放在速度结构体中。
					1 r/min = （2Π * R） / 60  m/s
入口参数：nRotateSpeed代表转速，r/min
返 回 值：轮子转速
**************************************************************************/
float rotateToLine(int nRotateSpeed)
{
	float fltReturn = 0.0f;
	fltReturn = nRotateSpeed * 2 * PI * RADIAUS / 60;  //转换为轮胎速度

	return fltReturn;
}

/**************************************************************************
函数功能：将轮子的速度，m/s转换成电机的转速 r/min，
					存放在速度结构体中。
					1 m/s = 60 /（2 * PI * R）
入口参数：fltSpeed代表轮子转速，m/s
返 回 值：代表电机转速
**************************************************************************/
int lineToRotate(float fltSpeed)
{
	int nReturn = 0;
	nReturn = (60 * fltSpeed / (2 * PI * RADIAUS)) * REDUCTION_RATE;

	return nReturn;
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
unsigned char turnOff(int voltage)
{
	unsigned char ucTemp;
	if (voltage < 10)//电池电压过低关闭电机
	{
		//设置电机目标速度为0，且失能电机
		ucTemp = 1;
		setMotorSpeed(MOTOR_CAN_ID, 0, 0);   //设置目标电机的速度
	}
	else
	{
		ucTemp = 0;
	}
	return ucTemp;
}

/**************************************************************************
函数功能：设置2个电机速度
入口参数：参数分别代表的是左电机和右电机对应的can从机id和电机速度
返 回 值：无
**************************************************************************/
void setMotorSpeed(int nID, int nMotorLeft_Speed, int nMotorRight_Speed)
{
	Clear_Error();            //有故障的话先清除
	ReInitalMotor();          //看是否有需要重新初始化的电机
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


