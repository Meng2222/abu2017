/**
  ******************************************************************************
  * @file      movebase.c
  * @author    ST42 & Meng22
  * @version   V2.0.0
  * @date      2017/02/07
  * @brief     Robocon2017�˶�����
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
         ���٣�mm/s������ת�٣�����/s����ת��             
============================================================
*/

/**
  * @brief  �����ٶ�ת��Ϊ��׼��λ�ٶ�
  * @param  pulse:�ٶ� ����/s
  * @retval vel:�ٶ� mm/s
  */
float Pulse2Vel(float pulse)
{
	float vel = 0.0f;
	vel = pulse * (2.0f * PI * WHEELRADIUS) / REDUCTION / STDPULSE;
	return vel;
}

/**
  * @brief  ��׼��λ�ٶ�ת��Ϊ�����ٶ�
  * @param  vel:�ٶ� mm/s
  * @retval velpulse:�ٶ� ����/s
  */
float Vel2Pulse(float vel)
{
	float pulse = 0.0f;
	pulse = vel / (2.0f * PI * WHEELRADIUS) * STDPULSE * REDUCTION;
	return pulse;
}

/*
*���ƣ�MOVEBASE_Init
*���ܣ������˵��̳�ʼ������ʼ�����е������λϵͳ�����̴�����
*��ʼ������Ҫ��������״̬�л�Ϊ�ѳ�ʼ��
*������none
*ע�⣺�˺������Եȴ���λϵͳ��ʼ����ɣ�Ҳ�ɲ��ȴ���������ȴ�����Ҫ
*�ڶ�λϵͳ���յ���Ч���ݺ󣬸��»�����״̬Ϊ��ʼ����������߼���Ҫע��
*/
void MOVEBASE_Init(void)
{
	
//    //�����ʼ����ʹ��
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
*���ܣ��˺�������������������л����˵�����
*���ݻ�����״̬�ľ������з�ʽ��ͬʱҲ����»����˵�״̬
*������none
*ע�⣺�˺���ÿ���������ڵ���һ��
*/
void MOVEBASE_Run(void)
{
	return;
}