/**
  ******************************************************************************
  * @file      movebase.c
  * @author    ST42 & Meng22
  * @version   V2.0.0
  * @date      2017/03/13
  * @brief     Robocon2017运动控制
  ******************************************************************************
  * @attention
  *            None
  ******************************************************************************
  */

/* Includes -------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "math.h"
#include "usart.h"
#include "stdlib.h"
#include "elmo.h"
#include "timer.h"
#include "movebase.h"
#include "stm32f4xx_usart.h"
#include "robot.h"
/* Exported functions ---------------------------------------------------------*/

extern robot_t gRobot;
float moveTimer=0.0f;
float GetPosX(void)
{
	return -gRobot.moveBase.actualXPos;
}

float GetAngle(void)
{
	return gRobot.moveBase.actualAngle;
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
*名称：MOVEBASE_Init
*功能：机器人底盘初始化，初始化走行电机、定位系统，底盘传感器
*初始化后，需要将机器人状态切换为已初始化
*参数：none
*注意：此函数可以等待定位系统初始化完成，也可不等待，如果不等待，需要
*在定位系统接收到有效数据后，更新机器人状态为初始化。这里的逻辑需要注意
*/
void MOVEBASE_Init(void)
{
	
    //电机初始化及使能
	elmo_Init(CAN2);
	
//	elmo_Enable(1);
//	elmo_Enable(2);
//	elmo_Enable(3);

	Vel_cfg(CAN2, 1, 100000, 100000);
	Vel_cfg(CAN2, 2, 100000, 100000);
	Vel_cfg(CAN2, 3, 100000, 100000);


	return;
}

/*
*MOVEBASE_Run
*功能：此函数完成整个比赛过程中机器人的走行
*根据机器人状态的决定走行方式，同时也会更新机器人的状态
*参数：none
*注意：此函数每个控制周期调用一次
*/
void MOVEBASE_Run(void)
{
	return;
}




/******************************************旧movebase***********************/



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
	Vel_cfg(CAN2, 1,motorAcc.wheel1,motorAcc.wheel1);
    Vel_cfg(CAN2, 2,motorAcc.wheel2,motorAcc.wheel2);
	Vel_cfg(CAN2, 3,motorAcc.wheel3,motorAcc.wheel3);
}

/**
  * @brief  在三个轮子上输出电机速度
  * @param  speed:三轮电机速度结构体
  * @retval None
  */



void ThreeWheelVelControl(wheelSpeed_t speed)
{
	gRobot.moveBase.targetSpeed.leftWheelSpeed = Pulse2Vel(-speed.leftWheelSpeed)/100.0f;
	gRobot.moveBase.targetSpeed.backwardWheelSpeed = Pulse2Vel(-speed.backwardWheelSpeed)/100.0f;
	gRobot.moveBase.targetSpeed.forwardWheelSpeed = Pulse2Vel(-speed.forwardWheelSpeed)/100.0f;
	
	VelCrl(CAN2, 1, -speed.backwardWheelSpeed);
	VelCrl(CAN2, 2, -speed.forwardWheelSpeed);
	VelCrl(CAN2, 3, -speed.leftWheelSpeed);
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


void CalcPath(expData_t *pExpData, float velX, float startPos, float targetPos, float accX)
{
	float targetDist = 0.0f;
	float distAcc = 0.0f, timeAcc = 0.0f;
	float distConst = 0.0f, timeConst = 0.0f;
	float distDec = 0.0f, timeDec = 0.0f;
	
	//计算理论距离和理论速度的绝对值
	targetDist = fabs(targetPos - startPos);
	timeAcc = fabs(velX) / accX;
	distAcc = 0.5f * accX * pow(timeAcc, 2);
	/*梯形速度规划部分*/
	if (2.0f * distAcc < targetDist)
	{
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
			pExpData->dist = ENDSPEED * ((timeAcc + timeConst + timeDec)-moveTimer);
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
	
}
void CalcPathToCenter(expData_t *pExpData, float velX, float startPos, float targetPos, float accX)
{
	float targetDist = 0.0f;
	float distAcc = 0.0f, timeAcc = 0.0f;
	float distConst = 0.0f, timeConst = 0.0f;
	float distDec = 0.0f, timeDec = 0.0f;
	
	//计算理论距离和理论速度的绝对值
	targetDist = fabs(targetPos - startPos);
	timeAcc = fabs(velX) / accX;
	distAcc = 0.5f * accX * pow(timeAcc, 2);
	/*梯形速度规划部分*/
	if (2.0f * distAcc < targetDist)
	{
		timeDec = timeAcc;
		distDec = distAcc;
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
			pExpData->dist = 0.5f * accX * (pow(timeAcc * 2.0f + timeConst - moveTimer, 2));
			pExpData->speed = accX * (2.0f * timeAcc + timeConst - moveTimer - 0.01f);
		}
		else if (moveTimer > (timeAcc + timeConst + timeDec))    /*低速匀速准备停车*/
		{
			pExpData->dist = 0;
			pExpData->speed = 0;
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



void SpeedAmend(wheelSpeed_t *pSpeedOut, expData_t *pExpData, float velX)
{
	float posErr = 0.0f;                                    //posErr:位置误差
	float outputSpeed = 0.0f;                               //outputSpeed:输出速度
	float velY = 0.0f;
	
	/*存在距离差用PID调速*/
	posErr = pExpData->pos - GetPosX();
	if (fabs(posErr) > 150.0f)
	{
		posErr = 150.0f;
	}
	outputSpeed = pExpData->speed + posErr * PVEL;
	
	/*特殊情况导致速度极不安全时紧急制动*/
	if(fabs(outputSpeed) > MAXSPEED)
	{
		while(1)
		{
			LockWheel();
		}
	}
	if((outputSpeed > 4500.0f) ||(outputSpeed < -4500.0f))
	{
		USART_SendData(UART5, (uint8_t)(outputSpeed/100.0f));
	}		
	
	velX = outputSpeed;
//	velY = fabs(0.07f * velX) /*+ 100.0f*/;
	velY = fabs(0.07f * velX) /*+ 100.0f*/;
	if(velY <= 50)
	{
		velY = 50;
	}
#define MAXMOVEACC (Vel2Pulse(2000))
#define PULSE_Y 100
	
//	if(GetPosX() < -12214.46f)
//	{
//		pSpeedOut->forwardWheelSpeed = Vel2Pulse(-velX                                             );
//		//速度分配至各轮
//		if(-gRobot.moveBase.actualSpeed.forwardWheelSpeed > pSpeedOut->forwardWheelSpeed + MAXMOVEACC)
//		{
//			pSpeedOut->forwardWheelSpeed	= 		-gRobot.moveBase.actualSpeed.forwardWheelSpeed +	MAXMOVEACC;
//			pSpeedOut->leftWheelSpeed		= 0.5f * gRobot.moveBase.actualSpeed.forwardWheelSpeed - 0.5f * MAXMOVEACC + PULSE_Y;
//			pSpeedOut->backwardWheelSpeed	= 0.5f * gRobot.moveBase.actualSpeed.forwardWheelSpeed - 0.5f * MAXMOVEACC - PULSE_Y;
//		}
//		else if(-gRobot.moveBase.actualSpeed.forwardWheelSpeed < pSpeedOut->forwardWheelSpeed - MAXMOVEACC)
//		{
//			pSpeedOut->forwardWheelSpeed	= 		-gRobot.moveBase.actualSpeed.forwardWheelSpeed - MAXMOVEACC;
//			pSpeedOut->leftWheelSpeed		= 0.5f * gRobot.moveBase.actualSpeed.forwardWheelSpeed + 0.5f * MAXMOVEACC + PULSE_Y;
//			pSpeedOut->backwardWheelSpeed	= 0.5f * gRobot.moveBase.actualSpeed.forwardWheelSpeed + 0.5f * MAXMOVEACC - PULSE_Y;
//		}
//		else
//		{
//			pSpeedOut->forwardWheelSpeed	=  0.8f * Vel2Pulse(-velX) + 0.2f * (-gRobot.moveBase.actualSpeed.forwardWheelSpeed);
//			pSpeedOut->leftWheelSpeed		= 0.5f * -pSpeedOut->forwardWheelSpeed + 0 + PULSE_Y;
//			pSpeedOut->backwardWheelSpeed	= 0.5f * -pSpeedOut->forwardWheelSpeed + 0 - PULSE_Y;
//		}
//	}
//	else
//	{
		pSpeedOut->backwardWheelSpeed = Vel2Pulse( velX * 0.5f/*cos60*/ - velY * 0.8660254f/*cos30*/);
		pSpeedOut->forwardWheelSpeed = Vel2Pulse(-velX                                             );
		pSpeedOut->leftWheelSpeed = Vel2Pulse( velX * 0.5f/*cos60*/ + velY * 0.8660254f/*cos30*/);
//	}

	
	
////	pSpeedOut->backwardWheelSpeed = Vel2Pulse( velX * 0.5f/*cos60*/ - velY * 0.8660254f/*cos30*/);
//	pSpeedOut->forwardWheelSpeed = Vel2Pulse(-velX                                             );
////	pSpeedOut->leftWheelSpeed = Vel2Pulse( velX * 0.5f/*cos60*/ + velY * 0.8660254f/*cos30*/);

//	pSpeedOut->backwardWheelSpeed = Vel2Pulse(-750.0f)*CalcMotorAcc(MAXACC, atan2f(-1000.0f, 70.0f));/* velY 约等于  0.07*velX */
//	if(GetPosX()<12214.46f)
//	{
//		pSpeedOut->backwardWheelSpeed = (Vel2Pulse(-750.0f) - pSpeedOut->backwardWheelSpeed) * 3.0f / 4.0f + pSpeedOut->backwardWheelSpeed;
//		pSpeedOut->leftWheelSpeed = (Vel2Pulse(-750.0f) - pSpeedOut->leftWheelSpeed) * 1.0f / 10.0f + pSpeedOut->leftWheelSpeed;
//	}

	//姿态修正
	if(GetAngle() >= 0.0f)
	{
		pSpeedOut->backwardWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		pSpeedOut->forwardWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		pSpeedOut->leftWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle())); 
	}
	else if(GetAngle() < 0.0f)
	{
		pSpeedOut->backwardWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		pSpeedOut->forwardWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		pSpeedOut->leftWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle())); 
	}

	//防止电机速度超过极限速度
	if(fabs(pSpeedOut->backwardWheelSpeed) > INSANEVEL || fabs(pSpeedOut->forwardWheelSpeed) > INSANEVEL || fabs(pSpeedOut->leftWheelSpeed) > INSANEVEL)
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
	VelCrl(CAN2, 1, 0);
	VelCrl(CAN2, 2, 0);
	VelCrl(CAN2, 3, 0);
}

//x方向定速移动
void MoveX(float velX)
{
	wheelSpeed_t speedOut = {0.0f, 0.0f, 0.0f};
	float velY = 0.0f;
	velY = fabs(0.07f * velX) + 100.0f;
	
	
	//速度分配至各轮
	speedOut.backwardWheelSpeed = Vel2Pulse( velX * 0.5f/*cos60*/ - velY * 0.8660254f/*cos30*/);
	speedOut.forwardWheelSpeed = Vel2Pulse(-velX                                             );
	speedOut.leftWheelSpeed = Vel2Pulse( velX * 0.5f/*cos60*/ + velY * 0.8660254f/*cos30*/);

	
	//姿态修正
	if(GetAngle() > 0)
	{
		speedOut.backwardWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		speedOut.forwardWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		speedOut.leftWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle())); 
	}
	else if(GetAngle() < 0)
	{
		speedOut.backwardWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		speedOut.forwardWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
		speedOut.leftWheelSpeed += Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle())); 
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
	

	
	//速度给出至各轮
	ThreeWheelVelControl(speedOut);
}
void MoveToCenter(float targetPos, float velX, float accX)
{
	//速度控制需要的过程变量
	static float formerTargetPos = 23333.0f;                 //formerTargetPos:判断是否是不同运动过程
	static float startPos = 0.0f;
	expData_t expData = {0.0f, 0.0f, 0.0f};
	wheelSpeed_t speedOut = {0.0f, 0.0f, 0.0f};
	
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
	CalcPathToCenter(&expData, velX, startPos, targetPos, accX);

	//速度调节部分
	SpeedAmend(&speedOut, &expData, velX);
	

	
	//速度给出至各轮
	ThreeWheelVelControl(speedOut);
}

void MoveY(float velY)
{
	wheelSpeed_t speedOut = {0.0f, 0.0f, 0.0f};

	speedOut.backwardWheelSpeed = Vel2Pulse( - velY * 0.8660254f/*cos30*/);
	speedOut.forwardWheelSpeed = Vel2Pulse(0);
	speedOut.leftWheelSpeed = Vel2Pulse(velY * 0.8660254f/*cos30*/);
	
	ThreeWheelVelControl(speedOut);
}
