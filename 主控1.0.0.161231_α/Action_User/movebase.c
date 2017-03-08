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

/* Exported functions ---------------------------------------------------------*/


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
	
//    //电机初始化及使能
	elmo_Init();
	
	elmo_Enable(1);
	elmo_Enable(2);
	elmo_Enable(3);
	Vel_cfg(1, 100000, 100000);
	Vel_cfg(2, 100000, 100000);
	Vel_cfg(3, 100000, 100000);

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