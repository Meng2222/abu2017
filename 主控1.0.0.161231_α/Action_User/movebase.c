/**
  ******************************************************************************
  * @file      movebase.c
  * @author    ������������������޹�˾
  * @version   V1.0.1
  * @date      2017/01/16
  * @brief     ������ȫ���ƶ�����λ�ñջ�����
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
 *�������
 */

/**
* @brief  ���������ٶ�
* @param  carAcc : �����˵ĺϼ��ٶ�
* @param  angle : ������ƽ�Ʒ����
* @retval mototAcc �����ֵ�����ٶ�
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
* @brief  ���õ�����ٶ�
* @param  motorAcc �� ���ֵ�����ٶȽṹ��
* @retval ��
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
* @brief  ���õ���ٶ�
* @param  speed �� ���ֵ���ٶȽṹ��
* @retval ��
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
* @brief   ����ƶ�����
* @param  ��
* @retval ��
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
* @brief   �����ٶ�ת��Ϊ��׼��λ�ٶ�
* @param  velPulse �� �ٶ� ����/��
* @retval velStandard �� �ٶ� ��/��
* @author ACTION
*/

float VelPulse2Standard(float velPulse)
{
	float velStandard = 0.0f;
	velStandard = velPulse/(REDUCTION)/2000.0f*2.0f*PI*WHEELRADIUS;
	return velStandard;
}
/**
* @brief   ��׼��λ�ٶ�ת��Ϊ�����ٶ�
* @param  velStandard �� �ٶ� ��/��
* @retval velPulse �� �ٶ� ����/��
* @author ACTION
*/
float VelStandard2Pulse(float velStandard)
{
	float velPulse = 0.0f;
	velPulse = velStandard/2.0f/PI/WHEELRADIUS*2000.0f*(REDUCTION);
	return velPulse;
}

//X�˶�����
int MoveX(float velX)
{
	static wheelSpeed_t speedOut = {0.0,0.0,0.0,0.0};
	static float p = 14.0f;
	float velY = fabs(0.07f * velX) + 0.1f;
	
	speedOut.v1 =  VelStandard2Pulse( velX * 0.707107f + velY  * 0.707107f);
	speedOut.v2 =  VelStandard2Pulse( velX * 0.707107f - velY  * 0.707107f);
	speedOut.v3 =  VelStandard2Pulse(-velX * 0.707107f - velY  * 0.707107f);
	speedOut.v4 =  VelStandard2Pulse(-velX * 0.707107f + velY  * 0.707107f);

	//��̬����
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

//Y�˶�����
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
//��̬��������
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

//����x�����ٶȺ���
float XSpeedDown(float posX, float dstX, float speedBegin)
{
	float speed = 0.0f;
	speed = (GetPosX() - dstX) / (posX - dstX) * (speedBegin - 0.7f * (speedBegin / fabs(speedBegin)))
        	+ 0.7f * (speedBegin / fabs(speedBegin));;
	return speed;
}

//����x�����ٶȺ���
float XSpeedUp(float posX, float dstX, float speedEnd)
{
	float speed = 0.0f;
	speed = (GetPosX() - posX) / (dstX - posX) * (speedEnd - 1.0f * (speedEnd / fabs(speedEnd)))
        	+ 1.0f * (speedEnd / fabs(speedEnd));
	return speed;
}
