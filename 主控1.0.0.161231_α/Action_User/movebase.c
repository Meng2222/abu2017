/**
  ******************************************************************************
  * @file      movebase.c
  * @author    沈阳艾克申机器人有限公司
  * @version   V1.0.1
  * @date      2017/01/16
  * @brief     四轮轮全向移动底盘位置闭环程序
  ******************************************************************************
  * @attention
  *
  *
  *
  *
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "math.h"
#include "gpio.h"
#include "usart.h"
#include "stdlib.h"
#include "elmo.h"
#include "timer.h"
#include "movebase.h"
#include "stm32f4xx_usart.h"


/* Private  variables ---------------------------------------------------------*/
//static wheelSpeed_t WheelVel;

/*
 *电机配置
 */

/**
* @brief  计算电机加速度
* @param  carAcc : 机器人的合加速度
* @param  angle : 机器人平移方向角
* @retval mototAcc ：四轮电机加速度
* @author ACTION
*/
motorAcc_t CalcMotorAcc(float carAcc , float angle)
{
  motorAcc_t motorAcc;
	float ward = 0.0f;
	if(angle>=0.0f)
	{
		ward = RADTOANG(atan2f(VELHORIZONTAL,VELVERTICAL));
	}
  if(angle<0.0f)
	{
		ward = RADTOANG(atan2f(-VELHORIZONTAL,VELVERTICAL));
	}
	motorAcc.wheel3 = carAcc * fabs(cosf(ANGTORAD(45+ward)));
	motorAcc.wheel2 = carAcc * fabs(cosf(ANGTORAD(45-ward)));
	motorAcc.wheel1 = carAcc * fabs(cosf(ANGTORAD(135-ward)));
	motorAcc.wheel4 = carAcc * fabs(cosf(ANGTORAD(135+ward)));
	return motorAcc;
}

/**
* @brief  配置电机加速度
* @param  motorAcc ： 四轮电机加速度结构体
* @retval 无
* @author ACTION
*/
void SetMotorAcc(motorAcc_t motorAcc)
{
	Vel_cfg(1,motorAcc.wheel1,motorAcc.wheel1);
    Vel_cfg(2,motorAcc.wheel2,motorAcc.wheel2);
	Vel_cfg(3,motorAcc.wheel3,motorAcc.wheel3);
	Vel_cfg(4,motorAcc.wheel4,motorAcc.wheel4);
}


/**
* @brief  配置电机速度
* @param  speed ： 四轮电机速度结构体
* @retval 无
* @author ACTION
*/
void FourWheelVelControl(wheelSpeed_t speed)
{
    VelCrl(1,speed.v1);
    VelCrl(2,speed.v2);
    VelCrl(3,speed.v3);
    VelCrl(4,speed.v4);
}
/**
* @brief   电机制动抱死
* @param  无
* @retval 无
* @author ACTION
*/
void StopMove(void)
{
		VelCrl(1,0);
		VelCrl(2,0);
		VelCrl(3,0);
		VelCrl(4,0);
}

/**
* @brief   脉冲速度转化为标准单位速度
* @param  velPulse ： 速度 脉冲/秒
* @retval velStandard ： 速度 米/秒
* @author ACTION
*/

float VelPulse2Standard(float velPulse)
{
	float velStandard = 0.0f;
	velStandard = velPulse/(REDUCTION)/2000.0f*2.0f*PI*WHEELRADIUS;
	return velStandard;
}
/**
* @brief   标准单位速度转化为脉冲速度
* @param  velStandard ： 速度 米/秒
* @retval velPulse ： 速度 脉冲/秒
* @author ACTION
*/
float VelStandard2Pulse(float velStandard)
{
	float velPulse = 0.0f;
	velPulse = velStandard/2.0f/PI/WHEELRADIUS*2000.0f*(REDUCTION);
	return velPulse;
}

//X运动函数
int MoveX(float velX)
{
	static wheelSpeed_t speedOut = {0.0,0.0,0.0,0.0};
	static float p = 14.0f;
	float velY = fabs(0.07f * velX) + 0.1f;
	
	speedOut.v1 =  VelStandard2Pulse( velX * 0.707107f + velY  * 0.707107f);
	speedOut.v2 =  VelStandard2Pulse( velX * 0.707107f - velY  * 0.707107f);
	speedOut.v3 =  VelStandard2Pulse(-velX * 0.707107f - velY  * 0.707107f);
	speedOut.v4 =  VelStandard2Pulse(-velX * 0.707107f + velY  * 0.707107f);

	//姿态修正
	if(getAngle() > 0)
	{
		speedOut.v1 +=  VelStandard2Pulse(0.4794f * ANGTORAD(-p * getAngle()));
		speedOut.v2 +=  VelStandard2Pulse(1.0950f * ANGTORAD(-p * getAngle()));
		speedOut.v3 +=  VelStandard2Pulse(0.6166f * ANGTORAD(-p * getAngle())); 
		speedOut.v4 +=  VelStandard2Pulse(0.0f);
	}
	else if(getAngle() < 0)
	{
		speedOut.v1 +=  VelStandard2Pulse(1.0950f * ANGTORAD(-p * getAngle()));
		speedOut.v2 +=  VelStandard2Pulse(0.4794f * ANGTORAD(-p * getAngle()));
		speedOut.v3 +=  VelStandard2Pulse(0.0f); 
		speedOut.v4 +=  VelStandard2Pulse(0.6166f * ANGTORAD(-p * getAngle()));
	}
	
	if(velX < 2.0f && velX > -2.0f)
	{
		FourWheelVelControl(speedOut);
	}
	if(velX >= 2.0f || velX <= -2.0f)
	{
		StopMove();
	}
	
	return RETURNOK;
}

//Y运动函数
int MoveY(float velY)
{
	static wheelSpeed_t speedOut = {0.0,0.0,0.0,0.0};
	
	speedOut.v1 =  VelStandard2Pulse( velY  * 0.707107f);
	speedOut.v2 =  VelStandard2Pulse(-velY  * 0.707107f);
	speedOut.v3 =  VelStandard2Pulse(-velY  * 0.707107f);
	speedOut.v4 =  VelStandard2Pulse( velY  * 0.707107f);
	
	FourWheelVelControl(speedOut);
	
	return RETURNOK;
}
//姿态修正函数
wheelSpeed_t RotateRoundAPoint(int wheelNum , float w)
{
	wheelSpeed_t speedOut = {0.0f,0.0f,0.0f,0.0f};
	if(wheelNum == 4)
	{
		speedOut.v1 =  VelStandard2Pulse(0.4794f * ANGTORAD(w));
		speedOut.v2 =  VelStandard2Pulse(1.1040f * ANGTORAD(w));
		speedOut.v3 =  VelStandard2Pulse(0.6166f * ANGTORAD(w)); 
		speedOut.v4 =  VelStandard2Pulse(0.0f * ANGTORAD(w));
	}
	else if(wheelNum == 3)
	{
		speedOut.v1 =  VelStandard2Pulse(1.1040f * ANGTORAD(w));
		speedOut.v2 =  VelStandard2Pulse(0.4794f * ANGTORAD(w));
		speedOut.v3 =  VelStandard2Pulse(0.0f); 
		speedOut.v4 =  VelStandard2Pulse(0.6166f * ANGTORAD(w));
	}
	return speedOut;
}

//减速x方向速度函数
float XSpeedDown(float posX, float dstX, float speedBegin)
{
	float speed = 0.0f;
	speed = (GetPosX() - dstX) / (posX - dstX) * (speedBegin - 0.7f * (speedBegin / fabs(speedBegin)))
        	+ 0.7f * (speedBegin / fabs(speedBegin));;
	return speed;
}

//加速x方向速度函数
float XSpeedUp(float posX, float dstX, float speedEnd)
{
	float speed = 0.0f;
	speed = (GetPosX() - posX) / (dstX - posX) * (speedEnd - 1.0f * (speedEnd / fabs(speedEnd)))
        	+ 1.0f * (speedEnd / fabs(speedEnd));
	return speed;
}
