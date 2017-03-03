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
#define ENDSPEED 120.0f


/*
============================================================
                     闭环用到的宏定义           
============================================================
*/

//姿态修正PID
#define PPOSE 5.0f
//速度闭环PID
#define PVEL 8.0f


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
	float v1;
	float v2;
	float v3;
}wheelSpeed_t;

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

#endif

