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
#define ROTATERAD   500.0f

//减速比
#define REDUCTION (299.0f/14.0f)

//每圈脉冲数
#define STDPULSE 2000.0f

//电机最大速度   单位:脉冲/s
#define MAXVEL 250000.0f
//电机最大加速度 单位:mm/s^2
#define MAXACC 5000.0f
//正常运动最大距离误差   单位:mm
#define MAXPOSERR 50.0f
//机器人特殊情况制动速度   单位:mm/s
#define INSANESPEED 3000.0f


/*
============================================================
                关于操作指令反馈的宏定义           
============================================================
*/

//函数正常运行
#define RETURNOK 		 1
//无效变量
#define INVALIDPARM      0
//有效变量
#define VALIDPARM 		 1
//变量超出范围
#define OUTOFRANGE 	    -2


/*
============================================================
                     闭环用到的宏定义           
============================================================
*/

//姿态修正PID
#define PPOSE 1.0f

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


/* Exported functions ------------------------------------------------------- */

/*
============================================================
                       电机配置
============================================================
*/

/**
  * @brief  计算各电机加速度
  * @param  carAcc:机器人的合加速度
  * @param  angle:机器人平移方向角
  * @retval mototAcc:四轮电机加速度
  */
motorAcc_t CalcMotorAcc(float carAcc,float angle);

/**
  * @brief  配置电机加速度
  * @param  motorAcc:四轮电机加速度结构体
  * @retval None
  */
void SetMotorAcc(motorAcc_t motorAcc);

/**
  * @brief  在三个轮子上输出电机速度
  * @param  speed:三轮电机速度结构体
  * @retval None
  */
void ThreeWheelVelControl(wheelSpeed_t speed);

/*
============================================================
         车速（mm/s）与电机转速（脉冲/s）的转换             
============================================================
*/

/**
  * @brief  脉冲速度转化为标准单位速度
  * @param  pulse:速度 脉冲/s
  * @retval vel:速度   mm/s
  */
float Pulse2Vel(float pulse);

/**
  * @brief  标准单位速度转化为脉冲速度
  * @param  vel:速度   mm/s
  * @retval pulse:速度 脉冲/s
  */
float Vel2Pulse(float vel);

/*
============================================================
                      机器人运动部分            
============================================================
*/

/**
  * @brief  电机制动抱死
  * @param  None
  * @retval None
  */
void LockWheel(void);

/**
  * @brief  匀加减速运动控制函数
  * @param  velX:x方向速度     mm/s
  * @param  startPos:起始位置  mm
  * @param  targetPos:目标位置 mm
  * @param  accX:x方向加速度   mm/s^2
  * @retval RETURNOK:状态宏定义
  * @attention
  *         此函数没有停车语句
  */
int Move(float velX, float startPos, float targetPos, float accX);

#endif

