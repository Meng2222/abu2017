/**
  ******************************************************************************
  * @file      movebase.h
  * @author    ST42 & Meng22
  * @version   V2.0.0
  * @date      2017/02/07
  * @brief     Robocon2017运动控制
  ******************************************************************************
  * @attention
  *            None
  ******************************************************************************
  */
#ifndef __MOVEBASE_H
#define __MOVEBASE_H

/* Exported define ------------------------------------------------------------*/
#define LEFT_WHEEL_ID 3
#define FORWARD_WHEEL_ID 2
#define BACKWARD_WHEEL_ID 1
#define MOVEBASE_BROADCAST_ID 0


/*
============================================================
                三全向轮底盘相关参数宏定义           
============================================================
*/
 
//车轮半径 单位:mm
#define WHEELRADIUS 76.0f
//姿态修正时绕轮旋转半径 单位:mm
#define ROTATERAD   815.0f

//减速比
#define REDUCTION (299.0f/14.0f)

//每圈脉冲数
#define STDPULSE 2000.0f

//电机最大速度   单位:脉冲/s
#define INSANEVEL 400000.0f
//电机最大加速度  单位:脉冲/s^2
#define MAXACC 3000000.0f

//机器人特殊情况制动速度   单位:mm/s
#define MAXSPEED 4000.0f

//运动完成时停车速度    单位:mm/s
#define ENDSPEED 200.0f



/*
============================================================
                        其他宏定义           
============================================================
*/
 
#define PI  3.141592653579f


/*
============================================================
                         单位转换          
============================================================
*/

//弧度制和角度制相互转换
#define ANGTORAD(x) (float)((x) / 180.0f * 3.141592653579f)
#define RADTOANG(x) (float)((x) / 3.141592653579f * 180.0f)


/* Exported types ------------------------------------------------------------*/

//加速度的结构体
typedef struct
{
	float wheel1;
	float wheel2;
	float wheel3;
}motorAcc_t;

//三个轮速度的结构体  单位:脉冲/s
typedef struct
{
	float leftWheelSpeed;
	float forwardWheelSpeed;
	float backwardWheelSpeed;
}wheelSpeed_t;

typedef struct
{
	float leftWheelCurrent;
	float forwardWheelCurrent;
	float backwardWheelCurrent;
}wheelCurrent_t;

//电机定子表面温度，单位摄氏度
typedef struct
{
	float leftWheelMotorTemperature;
	float forwardWheelMotorTemperature;
	float backwardWheelMotorTemperature;
}motorTemperature_t;

//驱动器内部mosfet温度，单位摄氏度
typedef struct
{
	float leftWheelDriverTemperature;
	float forwardWheelDrvierTemperature;
	float backwardWheelDriverTemperature;
}driverTemperature_t;


typedef struct
{
	int leftWheelDriverFlag;
	int forwardWheelDriverFlag;
	int backwardWheelDriverFlag;
}driverCurrentLimitFlag_t;


typedef struct
{
	int leftDriverCommandVelocity;
	int forwardDriverCommandVelocity;
	int backwardDriverCommandVelocity;
}driverCommandVelocity_t;

typedef struct
{
	int leftDriverJoggingVelocity;
	int forwardDriverJoggingVelocity;
	int backwardDriverJoggingVelocity;
}driverJoggingVelocity_t;


typedef struct
{
	//机器人走行轮子目标线速度，范围【-30，30】，单位0.1m/s
	wheelSpeed_t targetSpeed;
	//机器人走行轮子实际线速度，范围【-30，30】，单位0.1m/s
	wheelSpeed_t actualSpeed;
	//机器人走行轮子目标线加速度速度，范围【-30，30】，单位0.1m/s2
	motorAcc_t targetAcc;
	//机器人走行轮子实际线加速度速度，范围【-30，30】，单位0.1m/s2
	motorAcc_t actualAcc;
	//机器人走行轮子目标电机电流，范围【-200，200】，单位0.1A
	wheelCurrent_t targetCurrent;
	//机器人走行轮子实际电机电流，范围【-200，200】，单位0.1A， fix me
	wheelCurrent_t acturalCurrent;
	
	
	//电机定子温度，目前只写了走行的3个电机温度，fix me
	motorTemperature_t motorTemperature;
	//驱动器温度，目前只写了走行的3个驱动器温度，fix me
	driverTemperature_t driverTemperature;
	
	driverCurrentLimitFlag_t driverCurrentLimitFlag;
	
	driverCommandVelocity_t driverCommandVelocity;
	
	driverJoggingVelocity_t driverJoggingVelocity;

	//机器人目标停止位置，范围1、2、3，对应靠近发射区，中点和靠近出发区点
	unsigned char targetPoint;
	//机器人目标角度，逆时针为正，范围【-180，180】，单位度
	float targetAngle;
	//机器人实际角度，逆时针为正，范围【-180，180】，单位度
	float actualAngle;
	//机器人X方向目标位置，出发前进方向位X正方向，单位毫米
	float targetXPos;
	//机器人X方向实际位置，出发前进方向位X正方向，单位毫米
	float actualXPos;
	//机器人Y方向目标位置，出发前进右侧方向位Y正方向，单位毫米
	float targetYPos;
	//机器人Y方向实际位置，出发前进右侧方向位Y正方向，单位毫米
	float actualYPos;
}movebase_t;
/* Exported functions ------------------------------------------------------- */

/*
============================================================
         车速（mm/s）与电机转速（脉冲/s）的转换             
============================================================
*/

//脉冲速度转化为标准单位速度
float Pulse2Vel(float pulse);

//标准单位速度转化为脉冲速度
float Vel2Pulse(float vel);


/*
*名称：MOVEBASE_Init
*功能：机器人底盘初始化，初始化走行电机、定位系统，底盘传感器
*初始化后，需要将机器人状态切换为已初始化
*参数：none
*注意：此函数可以等待定位系统初始化完成，也可不等待，如果不等待，需要
*在定位系统接收到有效数据后，更新机器人状态为初始化。这里的逻辑需要注意
*/
void MOVEBASE_Init(void);

/*
*MOVEBASE_Run
*功能：此函数完成整个比赛过程中机器人的走行
*根据机器人状态的决定走行方式，同时也会更新机器人的状态
*参数：none
*注意：此函数每个控制周期调用一次
*/
void MOVEBASE_Run(void);

/*******************旧***************************************/



/*
============================================================
                     闭环用到的宏定义           
============================================================
*/

//姿态修正PID
#define PPOSE 5.0f
//速度闭环PID
#define PVEL 6.0f


/*
============================================================
                        其他宏定义           
============================================================
*/
 
#define PI  3.141592653579f


/*
============================================================
                         单位转换          
============================================================
*/

//弧度制和角度制相互转换
#define ANGTORAD(x) (float)((x) / 180.0f * 3.141592653579f)
#define RADTOANG(x) (float)((x) / 3.141592653579f * 180.0f)


/* Exported types ------------------------------------------------------------*/

//电机失效原因
typedef struct
{
	float state1;
	float state2;
	float state3;
}motorFailureState_t;

//电流限制标志
typedef struct
{
	float flag1;
	float flag2;
	float flag3;
}currentLimitFlag_t;

//速度规划的理论值结构体
typedef struct
{
	float dist;
	float speed;
	float pos;
}expData_t;


/* Exported functions ------------------------------------------------------- */

/*
============================================================
                       电机配置
============================================================
*/

//计算各电机加速度
motorAcc_t CalcMotorAcc(float carAcc,float angle);

//配置电机加速度
void SetMotorAcc(motorAcc_t motorAcc);

//在三个轮子上输出电机速度
void ThreeWheelVelControl(wheelSpeed_t speed);

/*
============================================================
         车速（mm/s）与电机转速（脉冲/s）的转换             
============================================================
*/

//脉冲速度转化为标准单位速度
float Pulse2Vel(float pulse);

//标准单位速度转化为脉冲速度
float Vel2Pulse(float vel);

/*
============================================================
                      过程量控制与调节            
============================================================
*/

//轨迹理论值计算函数
void CalcPath(expData_t *pExpData, float velX, float startPos, float targetPos, float accX);

//速度调节函数
void SpeedAmend(wheelSpeed_t *pSpeedOut, expData_t *pExpData, float velX);

/*
============================================================
                      机器人运动部分            
============================================================
*/

//电机制动抱死
void LockWheel(void);

//x方向定速移动函数
void MoveX(float velX);

//运动函数
void MoveTo(float targetPos, float velX, float accX);

float GetPosX(void);
float GetAngle(void);

#endif

