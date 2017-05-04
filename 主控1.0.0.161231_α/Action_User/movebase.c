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
#include "dma.h"
/* Exported functions ---------------------------------------------------------*/

extern robot_t gRobot;
float moveTimer=0.0f;
extern float amendX;
float GetPosX(void)
{
	return -gRobot.moveBase.actualXPos+amendX;
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
	
	elmo_Enable(CAN2,1);
	elmo_Enable(CAN2,2);
	elmo_Enable(CAN2,3);

//	Vel_cfg(CAN2, 1, 100000, 100000);
//	Vel_cfg(CAN2, 2, 100000, 100000);
//	Vel_cfg(CAN2, 3, 100000, 100000);


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

/**
  * @brief  在三个轮子自检速度检测
  * @param  speed:三轮电机速度
  * @retval None
  */
void ThreeWheelVelControlSelfCheck(int direction)
{
	if(direction==1)
	{
		VelCrl(CAN2, 1,200000);
		VelCrl(CAN2, 2,200000);
		VelCrl(CAN2, 3,200000);
	}
	else if(direction==2)
	{
		VelCrl(CAN2, 1,-200000);
		VelCrl(CAN2, 2,-200000);
		VelCrl(CAN2, 3,-200000);
	}
	else
	{
		VelCrl(CAN2, 1,0);
		VelCrl(CAN2, 2,0);
		VelCrl(CAN2, 3,0);
	}
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
	gRobot.moveBase.targetSpeed.leftWheelSpeed = speed.leftWheelSpeed/100.0f;
	gRobot.moveBase.targetSpeed.backwardWheelSpeed = speed.backwardWheelSpeed/100.0f;
	gRobot.moveBase.targetSpeed.forwardWheelSpeed = speed.forwardWheelSpeed/100.0f;
	
	VelCrl(CAN2, BACKWARD_WHEEL_ID, speed.backwardWheelSpeed);
	VelCrl(CAN2, FORWARD_WHEEL_ID, speed.forwardWheelSpeed);
	VelCrl(CAN2, LEFT_WHEEL_ID, speed.leftWheelSpeed);
}
/**
* @brief 将车的平动速度分解到三个轮上
* @param  carVel : 车速  
*		velAngle ：速度方向（以面向场地方向为0度，顺时针为负，逆时针为正，范围-180-180）
  * @retval 分解到三个轮的速度
  */
wheelSpeed_t SeperateVelToThreeMotor(float carVel , float velAngle)
{
	#define BACK_WHEEL_VEL_ANG (30.0f)
	#define LEFT_WHEEL_VEL_ANG (-90.0f)
	#define FORWARD_WHEEL_VEL_ANG (150.0f)
	wheelSpeed_t wheelSpeed = {0.0f};
	wheelSpeed.backwardWheelSpeed = carVel*cosf(ANGTORAD(velAngle - gRobot.moveBase.actualAngle - BACK_WHEEL_VEL_ANG));
	wheelSpeed.leftWheelSpeed = carVel*cosf(ANGTORAD(velAngle - gRobot.moveBase.actualAngle - LEFT_WHEEL_VEL_ANG));
	wheelSpeed.forwardWheelSpeed = carVel*cosf(ANGTORAD(velAngle - gRobot.moveBase.actualAngle - FORWARD_WHEEL_VEL_ANG));
	return wheelSpeed;
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


float distDebug = 0.0f;
float speedDebug = 0.0f;
void CalcPath(expData_t *pExpData, float velX, float startPos, float targetPos, float accX ,float decX)
{
	float targetDist = 0.0f , actualDist = 0.0f;
	float distAcc = 0.0f, timeAcc = 0.0f;
	float distDec = 0.0f, timeDec = 0.0f;
	float actualMaxVelX = 0.0f;
	//计算理论距离和理论速度的绝对值
	targetDist = fabs(targetPos - startPos);
	actualDist = fabs(targetPos - GetPosX());
	timeAcc = fabs(velX) / accX;
	distAcc = 0.5f * accX * pow(timeAcc, 2);
	timeDec = fabs(velX)/decX;
	distDec = 0.5f * decX * pow(timeDec, 2);
	/*梯形速度规划部分*/
	if ((distAcc + distDec) < targetDist)
	{
		if (actualDist > distDec)    
		{
			//加速段
			if(fabs(gRobot.moveBase.actualKenimaticInfo.vt) < (velX - accX/100.0f))
			{
				pExpData->dist = actualDist;
				pExpData->speed = fabs(gRobot.moveBase.actualKenimaticInfo.vt) + accX/100.0f;
			}
			//匀速段
			else
			{
				pExpData->dist = actualDist;
				pExpData->speed = fabs(velX);
			}
		}
		else    
		{
			moveTimFlag = 1;
			if(moveTimer < timeDec)
			{
				pExpData->dist = 0.5f * decX * pow(timeDec - moveTimer, 2);
				pExpData->speed = decX * (timeDec - moveTimer);
			}
			else
			{
				pExpData->dist = 0;
				pExpData->speed = 0;			
			}
		}
	}
	/*三角形速度规划部分*/
	else if ((distAcc + distDec) >= targetDist)
	{
		timeAcc = sqrtf(2.0f * targetDist * (accX + decX))/accX;
		distAcc = 0.5f * accX * pow(timeAcc , 2);
		timeDec =sqrtf(2.0f * targetDist * (accX + decX))/decX;
		distDec = 0.5f * decX * pow(timeDec , 2);
		actualMaxVelX = sqrtf(2.0f * targetDist * (accX + decX));
		if (actualDist > distDec)    /*加速段*/
		{
			//加速段
			if(fabs(gRobot.moveBase.actualKenimaticInfo.vt) < (actualMaxVelX - accX/100.0f))
			{
				pExpData->dist = actualDist;
				pExpData->speed = fabs(gRobot.moveBase.actualKenimaticInfo.vt) + accX/100.0f;
			}
			//匀速段
			else
			{
				pExpData->dist = actualDist;
				pExpData->speed = fabs(actualMaxVelX);
			}
		}
		else   /*减速段*/
		{
			moveTimFlag = 1;
			if(moveTimer < timeDec)
			{
				pExpData->dist = 0.5f * decX * pow(timeDec - moveTimer, 2);
				pExpData->speed = decX * (timeDec - moveTimer);
			}
			else
			{
				pExpData->dist = 0;
				pExpData->speed = 0;			
			}
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
	if (posErr > 150.0f)
	{
		posErr = 150.0f;
	}
	else if(posErr < -150.0f)
	{
		posErr = -150.0f;
	}
	outputSpeed = pExpData->speed + posErr * PVEL;
	
	/*特殊情况导致速度极不安全时紧急制动*/
	if(fabs(outputSpeed) > MAXSPEED)
	{
		while(1)
		{
			UART5_OUT((uint8_t *)"Outputspeed Over MAXSPEED error!!!!!!!!!!\r\n");
			LockWheel();
		}
	}
	if((outputSpeed > 5000.0f) ||(outputSpeed < -5000.0f))
	{
		UART5BufPut((uint8_t)(outputSpeed/100.0f));
	}		
	
	velX = outputSpeed;

	velY = sqrtf(gRobot.moveBase.posYSecondDerivative)*PVELY;
	if(velY <= 30.0f)
	{
		velY = 30.0f;
	}
#ifdef BLUE_FIELD
	if(gRobot.moveBase.actualYPos > 40.0f)
	{
		velY = 0.0f;
	}
#endif
#ifdef RED_FIELD
	if(gRobot.moveBase.actualYPos < -40.0f)
	{
		velY = 0.0f;
	}
#endif
#define MAXMOVEACC (Vel2Pulse(2000))
#define PULSE_Y 100
	speedDebug = velX;
	distDebug = posErr;
	if(velX <= 0.0f)
	{
		pSpeedOut->backwardWheelSpeed = Vel2Pulse(SeperateVelToThreeMotor(velX , 90.0f).backwardWheelSpeed + SeperateVelToThreeMotor(velY , 0.0f).backwardWheelSpeed);
		pSpeedOut->forwardWheelSpeed = Vel2Pulse(SeperateVelToThreeMotor(velX , 90.0f).forwardWheelSpeed + SeperateVelToThreeMotor(velY , 0.0f).forwardWheelSpeed);
		pSpeedOut->leftWheelSpeed = Vel2Pulse(SeperateVelToThreeMotor(velX , 90.0f).leftWheelSpeed + SeperateVelToThreeMotor(velY , 0.0f).leftWheelSpeed);
	}
	else if(velX > 0.0f)
	{
		pSpeedOut->backwardWheelSpeed = Vel2Pulse(SeperateVelToThreeMotor(velX , -90.0f).backwardWheelSpeed + SeperateVelToThreeMotor(velY , 0.0f).backwardWheelSpeed);
		pSpeedOut->forwardWheelSpeed = Vel2Pulse(SeperateVelToThreeMotor(velX , -90.0f).forwardWheelSpeed + SeperateVelToThreeMotor(velY , 0.0f).forwardWheelSpeed);
		pSpeedOut->leftWheelSpeed = Vel2Pulse(SeperateVelToThreeMotor(velX , -90.0f).leftWheelSpeed + SeperateVelToThreeMotor(velY , 0.0f).leftWheelSpeed);	
	}
	//姿态修正
	pSpeedOut->backwardWheelSpeed -= Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
	pSpeedOut->forwardWheelSpeed -= Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle()));
	pSpeedOut->leftWheelSpeed -= Vel2Pulse(ROTATERAD * ANGTORAD(PPOSE * GetAngle())); 

	//防止电机速度超过极限速度
	if(fabs(pSpeedOut->backwardWheelSpeed) > INSANEVEL || fabs(pSpeedOut->forwardWheelSpeed) > INSANEVEL || fabs(pSpeedOut->leftWheelSpeed) > INSANEVEL)
	{
		while(1)
		{
			UART5_OUT((uint8_t *)"One Of The Motor Overspeed!!!!!!!!!!!!\r\n");
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

void MoveTo(float targetPos, float velX, float accX , float decX)
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
		moveTimer = 0.0f;
		moveTimFlag = 0;
	}

	//轨迹计算
	CalcPath(&expData, velX, startPos, targetPos, accX , decX);

	//速度调节部分
	SpeedAmend(&speedOut, &expData, velX);
//	USART_SendData(UART5,(uint8_t)moveTimer);
//	USART_SendData(UART5,(int8_t)(expData.pos/100.0f));
//	USART_SendData(UART5,(int8_t)(expData.speed/100.0f));
	UART5_OUT((uint8_t *)"%d\t",(int)(speedDebug/100.0f));
	UART5_OUT((uint8_t *)"%d\t",(int)(distDebug/10.0f));
	
	
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
