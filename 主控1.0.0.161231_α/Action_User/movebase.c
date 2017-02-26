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
    VelCrl(1, -speed.v1);
    VelCrl(2, -speed.v2);
    VelCrl(3, -speed.v3);
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
                     过程量控制与调节            
============================================================
*/
/**
  * @brief  轨迹理论值计算函数
  * @param  pExpData:运动理论值结构体指针
  * @param  velX:x方向速度     mm/s
  * @param  startPos:起始位置  mm
  * @param  targetPos:目标位置 mm
  * @param  accX:x方向加速度   mm/s^2
  * @retval None
  * @attention
  *         None
  */
extern float moveTimer;
extern uint8_t moveTimFlag;
extern int expSpeed;

void CalcPath(expData_t *pExpData, float velX, float startPos, float targetPos, float accX)
{
	float targetDist = 0.0f;
	float distAcc = 0.0f, timeAcc = 0.0f;
	float distConst = 0.0f, timeConst = 0.0f;
	float distDec = 0.0f, timeDec = 0.0f;
	
	targetDist = fabs(targetPos - startPos);
	
	
	//计算理论距离和理论速度的绝对值
	
	/*梯形速度规划部分*/
	if (2.0f * distAcc < targetDist)
	{
		timeAcc = fabs(velX) / accX;
		distAcc = 0.5f * accX * pow(timeAcc, 2);
		timeDec = timeAcc - ENDSPEED / accX;
		distDec = distAcc - pow(ENDSPEED, 2) / (2 * accX);
		distConst = targetDist - distAcc - distDec;
		timeConst = distConst / fabs(velX);
		
		if (moveTimer <= timeAcc)    /*加速段*/
		{
			pExpData->dist = targetDist - 0.5f * accX * pow(moveTimer, 2);
			pExpData->speed = accX * (moveTimer + 0.01f);
		}
		else if (moveTimer > timeAcc && moveTimer <= (timeAcc + timeConst))    /*匀速段*/
		{
			pExpData->dist = targetDist - distAcc - fabs(velX) * (moveTimer - timeAcc);
			pExpData->speed = fabs(velX);
		}
		else if (moveTimer > (timeAcc + timeConst) && 
			    moveTimer <= (timeAcc + timeConst + timeDec))    /*减速段*/
		{
			pExpData->dist = 0.5f * accX * (pow(timeAcc * 2.0f + timeConst - moveTimer, 2)
                               			  - pow(timeAcc - timeDec, 2));
			pExpData->speed = accX * (2.0f * timeAcc + timeConst - moveTimer - 0.01f);
		}
		else if (moveTimer > (timeAcc + timeConst + timeDec))    /*低速匀速准备停车*/
		{
			pExpData->dist = ENDSPEED * ((timeAcc + timeConst + timeDec) - moveTimer);
			pExpData->speed = ENDSPEED;
		}
	}
	
	/*三角形速度规划部分*/
	else if (2.0f * distAcc >= targetDist)
	{
		timeAcc = sqrt((targetDist + pow(ENDSPEED, 2) / (2 * accX)) / accX);
		distAcc = 0.5f * accX * pow(timeAcc, 2);
		distDec = targetDist - distAcc;
		timeDec = timeAcc - ENDSPEED / accX;
		
		if (moveTimer <= timeAcc)    /*加速段*/
		{
			pExpData->dist = targetDist - 0.5f * accX * pow(moveTimer, 2);
			pExpData->speed = accX * (moveTimer + 0.01f);
		}
		else if (moveTimer > timeAcc && moveTimer <= (timeAcc + timeDec))    /*减速段*/
		{
			pExpData->dist = 0.5f * accX * pow(2.0f * timeAcc - moveTimer, 2)
			                             - pow(timeAcc - timeDec, 2);
			pExpData->speed = accX * (2.0f * timeAcc - moveTimer - 0.01f);
		}
		else if (moveTimer > (timeAcc + timeDec))    /*低速匀速准备停车*/
		{
			pExpData->dist = ENDSPEED * ((timeAcc + timeDec) - moveTimer);
			pExpData->speed = ENDSPEED;
		}
	}
	
	
	//计算理论位置并修正理论速度的符号
	if(velX < 0.0f)
	{
		pExpData->pos = startPos - targetDist + pExpData->dist;
		pExpData->speed = -pExpData->speed;
	}
	else 
	{
		pExpData->pos = startPos + targetDist - pExpData->dist;
	}
	
	expSpeed = pExpData->speed;
}


/**
  * @brief  速度调节函数
  * @param  pSpeedOut:实际输出速度结构体指针
  * @param  pExpData:运动理论值结构体指针
  * @param  velX:x方向速度     mm/s
  * @retval None
  * @attention
  *         None
  */

extern int expSpeedp;

void SpeedAmend(wheelSpeed_t *pSpeedOut, expData_t *pExpData, float velX)
{
	float posErr = 0.0f;                                    //posErr:位置误差
	float outputSpeed = 0.0f;                               //outputSpeed:输出速度
	float velY = 0.0f;
	
	/*存在距离差用PID调速*/
	posErr = pExpData->pos - GetPosX();
	if (fabs(posErr) > 300.0f)
	{
		posErr = 0.0f;
	}
	outputSpeed = pExpData->speed + posErr * PVEL;
	
	expSpeedp = outputSpeed;
	
	/*特殊情况导致速度极不安全时紧急制动*/
	if(fabs(outputSpeed) > MAXSPEED)
	{
		while(1)
		{
			LockWheel();
		}
	}
	
	velX = outputSpeed;
	velY = fabs(0.07f * velX) + 100.0f;
	
	//速度分配至各轮
	pSpeedOut->v1 = Vel2Pulse( velX * 0.5f/*cos60*/ - velY * 0.8660254f/*cos30*/);
	pSpeedOut->v2 = Vel2Pulse(-velX                                             );
	pSpeedOut->v3 = Vel2Pulse( velX * 0.5f/*cos60*/ + velY * 0.8660254f/*cos30*/);

	
	//姿态修正
	if(GetAngle() > 0)
	{
		pSpeedOut->v1 += Vel2Pulse(0.0f      * ANGTORAD(PPOSE * GetAngle()));
		pSpeedOut->v2 += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		pSpeedOut->v3 += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle())); 
	}
	else if(GetAngle() < 0)
	{
		pSpeedOut->v1 += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		pSpeedOut->v2 += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		pSpeedOut->v3 += Vel2Pulse(0.0f      * ANGTORAD(PPOSE * GetAngle())); 
	}
	
	//防止电机速度超过极限速度
	if(fabs(pSpeedOut->v1) > INSANEVEL || fabs(pSpeedOut->v2) > INSANEVEL || fabs(pSpeedOut->v3) > INSANEVEL)
	{
		while(1)
		{
			LockWheel();
		}
	}
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

//x方向定速移动
void MoveX(float velX)
{
	wheelSpeed_t speedOut = {0.0, 0.0, 0.0};
	float velY = 0.0f;
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
	
	ThreeWheelVelControl(speedOut);
}

/**
  * @brief  匀加减速运动控制函数
  * @param  targetPos:目标位置 mm
  * @param  velX:x方向速度     mm/s
  * @param  accX:x方向加速度   mm/s^2
  * @retval None
  * @attention
  *         此函数没有停车语句
  */

void MoveTo(float targetPos, float velX, float accX)
{
	//速度控制需要的过程变量
	static float formerTargetPos = 23333.0f;                 //formerTargetPos:判断是否是不同运动过程
	static float startPos = 0.0f;
	expData_t expData = {0.0f, 0.0f, 0.0f};
	wheelSpeed_t speedOut = {0.0f, 0.0f, 0.0f};
	extern int mv1, mv2, mv3;
	
	//新运动过程初始化
	if(formerTargetPos != targetPos)
	{	
		formerTargetPos = targetPos;
		
		startPos = GetPosX();
		if (velX >= 0)
		{
			SetMotorAcc(CalcMotorAcc(MAXACC, atan2f(-1000.0f, 70.0f)/* velY 约等于  0.07*velX */));
		}
		else
		{
			SetMotorAcc(CalcMotorAcc(MAXACC, atan2f( 1000.0f, 70.0f)/* velY 约等于 -0.07*velX */));
		}
		moveTimer = 0.0f;
		moveTimFlag = 1;
	}

	//轨迹计算
	CalcPath(&expData, velX, startPos, targetPos, accX);

	//速度调节部分
	SpeedAmend(&speedOut, &expData, velX);
	
	mv1 = speedOut.v1;
	mv2 = speedOut.v2;
	mv3 = speedOut.v3;
	
	//速度给出至各轮
	ThreeWheelVelControl(speedOut);
}
