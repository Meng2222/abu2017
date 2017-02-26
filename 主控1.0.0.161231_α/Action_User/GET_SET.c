/**
  ******************************************************************************
  * @file    GET_SET.c
  * @author  ST42
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 
  
/* Includes -------------------------------------------------------------------*/
#include "GET_SET.h"
#include "math.h"
#include "movebase.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/


/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/*
===============================================================
                              posX
===============================================================
*/
static float posX = 0;
extern float amendX;

void SetPosX(float value)
{
	posX = value;
}

float GetPosX(void)
{
	return posX + amendX;
}

/*
===============================================================
                              posY
===============================================================
*/
static float posY = 0;

void SetPosY(float value)
{
	posY = value;
}
float GetPosY(void)
{
	return posY;
}

/*
===============================================================
                              angle
===============================================================
*/
static float angle = 0;

void SetAngle(float value)
{
	angle = value;
}

float GetAngle(void)
{
	return angle;
}

/*
===============================================================
                              vel
===============================================================
*/
static float vel = 0.0f;

//逐差法求速度
//公式：v = ((x3 - x0) + (x4 - x1) + (x5 - x2)) / 3t
//其中  t = 3 * 5ms = 15ms
void UpdateVel(void)
{
    static float xPassed[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	static uint8_t timeCounter = 0;

	xPassed[timeCounter % 6] = posX;
	
	if (timeCounter >= 5)
	{
		vel = 0;
		for (uint8_t i = 0; i < 3; i++)
		{
			vel += xPassed[(timeCounter - i) % 6] - xPassed[(timeCounter - i - 3) % 6];
		}
		vel = vel / (3 * 15) * 1000;
	}
	
	timeCounter++;
	if (timeCounter >= 11)
	{
		timeCounter -= 6;
	}
}

float GetVel(void)
{
    return vel;
}

/*
===============================================================
                         motor vel
===============================================================
*/
static float motorVel[3] = {0.0f, 0.0f, 0.0f};
static float motorVelRate = 0.0f, motorVelAng = 0.0f;

void SetMotorVel(float *value)
{
	for (int i = 0; i < 3; i++)
	{
		motorVel[i] = value[i];
	}
}

//三轮各自的脉冲速度
float GetMotorVel(int motorNum)
{
	return motorVel[motorNum - 1];
}

//合速度向量的速率和夹角
float GetMotorVelRate(void)
{
	motorVelRate = sqrtf(pow(motorVel[1], 2) + pow(motorVel[0] - motorVel[2], 2) / 3.0f);
	return motorVelRate;
}
float GetMotorVelAng(void)
{
	motorVelAng = -atan2f(sqrtf(3) * motorVel[1], motorVel[0] - motorVel[2]) / PI * 180.0f;
	return motorVelAng;
}

/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
