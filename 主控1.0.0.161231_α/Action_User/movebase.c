/**
  ******************************************************************************
  * @file      movebase.c
  * @author    ST42 & Meng22
  * @version   V2.0.0
  * @date      2017/02/07
  * @brief     Robocon2017运动控制
  ******************************************************************************
  * @attention
  *            None
  ******************************************************************************
  */

/* Includes -------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "math.h"
#include "gpio.h"
#include "usart.h"
#include "stdlib.h"
#include "elmo.h"
#include "timer.h"
#include "movebase.h"
#include "stm32f4xx_usart.h"
#include "GET_SET.h"

/* Exported functions ---------------------------------------------------------*/

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
motorAcc_t CalcMotorAcc(float carAcc, float angle)
{
    motorAcc_t motorAcc;

	motorAcc.wheel3 = carAcc * fabs(cosf(ANGTORAD(150 - RADTOANG(angle))));
	motorAcc.wheel2 = carAcc * fabs(cosf(ANGTORAD(90  - RADTOANG(angle))));
	motorAcc.wheel1 = carAcc * fabs(cosf(ANGTORAD(30  - RADTOANG(angle))));
	
	return motorAcc;
}

/**
  * @brief  配置电机加速度
  * @param  motorAcc:四轮电机加速度结构体
  * @retval None
  */
void SetMotorAcc(motorAcc_t motorAcc)
{
	Vel_cfg(1,motorAcc.wheel1,motorAcc.wheel1);
    Vel_cfg(2,motorAcc.wheel2,motorAcc.wheel2);
	Vel_cfg(3,motorAcc.wheel3,motorAcc.wheel3);
}

/**
  * @brief  在三个轮子上输出电机速度
  * @param  speed:三轮电机速度结构体
  * @retval None
  */
void ThreeWheelVelControl(wheelSpeed_t speed)
{
    VelCrl(1, speed.v1);
    VelCrl(2, speed.v2);
    VelCrl(3, speed.v3);
}

/*
============================================================
         车速（mm/s）与电机转速（脉冲/s）的转换             
============================================================
*/

/**
  * @brief  脉冲速度转化为标准单位速度
  * @param  pulse:速度 脉冲/s
  * @retval vel:速度 mm/s
  */
float Pulse2Vel(float pulse)
{
	float vel = 0.0f;
	vel = pulse * (2.0f * PI * WHEELRADIUS) / REDUCTION / STDPULSE;
	return vel;
}

/**
  * @brief  标准单位速度转化为脉冲速度
  * @param  vel:速度 mm/s
  * @retval velpulse:速度 脉冲/s
  */
float Vel2Pulse(float vel)
{
	float pulse = 0.0f;
	pulse = vel / (2.0f * PI * WHEELRADIUS) * STDPULSE * REDUCTION;
	return pulse;
}

/*
============================================================
                      机器人运动部分            
============================================================
*/

/**
  * @brief   电机制动
  * @param   None
  * @retval  None
  */
void LockWheel(void)
{
	VelCrl(1, 0);
	VelCrl(2, 0);
	VelCrl(3, 0);
}

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
int Move(float velX, float startPos, float targetPos, float accX)
{
	//速度控制需要的过程变量
	float targetDist = 0.0f;                                //targetDist:目标距离
	float timeAcc = 0.0f, timeConst = 0.0f;                 //timeAcc:加速时间     timeConst:匀速时间
	float distAcc = 0.0f, distConst = 0.0f;                 //distAcc:加速距离     distAcc:匀速距离
	float expDist = 0.0f, expSpeed = 0.0f, expPos = 0.0f;   //expDist:理论距离     expSpeed:理论速度   expPos:理论位置
	float posErr = 0.0f;                                    //posErr:位置误差
	float outputSpeed = 0.0f;                               //outputSpeed:输出速度
	static float timer = 0.0f, formerStartPos = 23333.0f;   //timer:时间因子       formerStartPos:判断起始位置改变
	static float startPosAct = 0.0f;                        //startPosAct:实际起始点，用于路径重新规划              
	
	
	//实际运动需要的过程变量
	static wheelSpeed_t speedOut = {0.0, 0.0, 0.0};
	float velY = 0.0f;
	
	
	//梯形速度控制部分	
	/*新运动过程初始化*/
	if(formerStartPos != startPos)
	{
		formerStartPos = startPos;
		startPosAct = startPos;
		if (velX >= 0)
		{
			SetMotorAcc(CalcMotorAcc(1.5f * Vel2Pulse(accX), atan2f(-1000.0f, 70.0f)/* velY 约等于  0.07*velX */));
		}
		else
		{
			SetMotorAcc(CalcMotorAcc(1.5f * Vel2Pulse(accX), atan2f(1000.0f, 70.0f)/* velY 约等于 -0.07*velX */));
		}
		timer = 0.0f;
	}
	
	/*保证加速度安全*/
	if(accX > MAXACC)
	{
		accX = MAXACC;
	}
	
	/*计算速度和距离的理论值*/
	targetDist = fabs(targetPos - startPosAct);
	timeAcc = fabs(velX) / accX;
	distAcc = 0.5f * accX * pow(timeAcc, 2);
	/*梯形速度规划部分*/
	if(2.0f * distAcc < targetDist)
	{
		distConst = targetDist - 2.0f * distAcc;
		timeConst = distConst / fabs(velX);
		if(timer <= timeAcc)    /*加速段*/
		{
			expDist = targetDist - 0.5f * accX * pow(timer, 2);
			expSpeed = accX * (timer + 0.01f);
		}
		else if(timer > timeAcc && timer <= (timeAcc + timeConst))    /*匀速段*/
		{
			expDist = targetDist - distAcc - fabs(velX) * (timer - timeAcc);
			expSpeed = fabs(velX);
		}
		else if(timer > (timeAcc + timeConst) && timer <= (2.0f * timeAcc + timeConst))    /*减速段*/
		{
			expDist = 0.5f * accX * pow(2.0f * timeAcc + timeConst - timer, 2);
			expSpeed = accX * (2.0f * timeAcc + timeConst - timer - 0.01f);
		}
		else if(timer > (2 * timeAcc + timeConst))    /*超出预定轨迹*/
		{
			expDist = 0.0f;
			expSpeed = 0.0f;
		}
	}
	/*三角形速度规划部分*/
	else if(2.0f * distAcc >= targetDist)
	{
		if(timer <= timeAcc)    /*加速段*/
		{
			expDist = targetDist - 0.5f * accX * pow(timer, 2);
			expSpeed = accX * (timer + 0.01f);
		}
		else if(timer > timeAcc && timer <= 2.0f * timeAcc)    /*减速段*/
		{
			expDist = 0.5f * accX * pow(2.0f * timeAcc - timer, 2);
			expSpeed = accX * (2.0f * timeAcc - timer - 0.01f);
		}
		else if(timer > 2.0f * timeAcc)    /*超出预定轨迹*/
		{
			expDist = 0.0f;
			expSpeed = 0.0f;
		}		
	}
	timer += 0.01f;
	
	/*计算理论可以达到的位置*/
	if(velX < 0.0f)
	{
		expPos = startPosAct - targetDist + expDist;
		expSpeed = -expSpeed;
	}
	else 
	{
		expPos = startPosAct + targetDist - expDist;
	}
	
	/*存在距离差用PID调速*/
	posErr = expPos - GetPosX();
	outputSpeed = expSpeed + posErr * PVEL;	
	/*距离差过大时重新规划*/
	if(fabs(posErr) > MAXPOSERR)
	{
		timer = fabs(GetVel()) / accX;
		if (velX >= 0.0f)
		{
			startPosAct = GetPosX() - pow(GetVel(), 2) / (2 * accX);
		}
		else
		{
			startPosAct = GetPosX() + pow(GetVel(), 2) / (2 * accX);
		}
		outputSpeed = GetVel();
	}
	/*特殊情况导致速度极不安全时紧急制动*/
	if(fabs(outputSpeed) > INSANESPEED)
	{
		while(1)
		{
			LockWheel();
		}
	}
	
	/*减至某较低速度匀速*/
	if((outputSpeed <= 100.0f && outputSpeed >= 0.0f) && expDist <= targetDist / 2)
	{
		outputSpeed = 100.0f;
	}
	else if((outputSpeed >= -100.0f && outputSpeed < 0.0f) && expDist <= targetDist / 2)
	{
		outputSpeed = -100.0f;
	}
	
	velX = outputSpeed;
	velY = fabs(0.07f * velX) + 100.0f;
	
	
	//速度分配至各轮
	speedOut.v1 = Vel2Pulse( velX * 0.5f/*cos60*/ - velY * 0.8660254f/*cos30*/);
	speedOut.v2 = Vel2Pulse(-velX                                             );
	speedOut.v3 = Vel2Pulse( velX * 0.5f/*cos60*/ + velY * 0.8660254f/*cos30*/);

	
	//姿态修正
	if(GetAngle() > 0)
	{
		speedOut.v1 += Vel2Pulse(0.0f      * ANGTORAD(PPOSE * GetAngle()));
		speedOut.v2 += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		speedOut.v3 += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle())); 
	}
	else if(GetAngle() < 0)
	{
		speedOut.v1 += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		speedOut.v2 += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		speedOut.v3 += Vel2Pulse(0.0f      * ANGTORAD(PPOSE * GetAngle())); 
	}
	
	
	//速度给出至各轮
	ThreeWheelVelControl(speedOut);
		
	return RETURNOK;
}
