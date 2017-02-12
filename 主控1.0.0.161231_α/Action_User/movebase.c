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
#include "GET_SET.h"

/* Exported functions ---------------------------------------------------------*/

/*
============================================================
                       �������
============================================================
*/

/**
  * @brief  �����������ٶ�
  * @param  carAcc:�����˵ĺϼ��ٶ�
  * @param  angle:������ƽ�Ʒ����
  * @retval mototAcc:���ֵ�����ٶ�
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
  * @brief  ���õ�����ٶ�
  * @param  motorAcc:���ֵ�����ٶȽṹ��
  * @retval None
  */
void SetMotorAcc(motorAcc_t motorAcc)
{
	Vel_cfg(1,motorAcc.wheel1,motorAcc.wheel1);
    Vel_cfg(2,motorAcc.wheel2,motorAcc.wheel2);
	Vel_cfg(3,motorAcc.wheel3,motorAcc.wheel3);
}

/**
  * @brief  �������������������ٶ�
  * @param  speed:���ֵ���ٶȽṹ��
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
============================================================
                      �������˶�����            
============================================================
*/

/**
  * @brief   ����ƶ�
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
  * @brief  �ȼӼ����˶����ƺ���
  * @param  velX:x�����ٶ�     mm/s
  * @param  startPos:��ʼλ��  mm
  * @param  targetPos:Ŀ��λ�� mm
  * @param  accX:x������ٶ�   mm/s^2
  * @retval RETURNOK:״̬�궨��
  * @attention
  *         �˺���û��ͣ�����
  */
int Move(float velX, float startPos, float targetPos, float accX)
{
	//�ٶȿ�����Ҫ�Ĺ��̱���
	float targetDist = 0.0f;                                //targetDist:Ŀ�����
	float timeAcc = 0.0f, timeConst = 0.0f;                 //timeAcc:����ʱ��     timeConst:����ʱ��
	float distAcc = 0.0f, distConst = 0.0f;                 //distAcc:���پ���     distAcc:���پ���
	float expDist = 0.0f, expSpeed = 0.0f, expPos = 0.0f;   //expDist:���۾���     expSpeed:�����ٶ�   expPos:����λ��
	float posErr = 0.0f;                                    //posErr:λ�����
	float outputSpeed = 0.0f;                               //outputSpeed:����ٶ�
	static float timer = 0.0f, formerStartPos = 23333.0f;   //timer:ʱ������       formerStartPos:�ж���ʼλ�øı�
	static float startPosAct = 0.0f;                        //startPosAct:ʵ����ʼ�㣬����·�����¹滮              
	
	
	//ʵ���˶���Ҫ�Ĺ��̱���
	static wheelSpeed_t speedOut = {0.0, 0.0, 0.0};
	float velY = 0.0f;
	
	
	//�����ٶȿ��Ʋ���	
	/*���˶����̳�ʼ��*/
	if(formerStartPos != startPos)
	{
		formerStartPos = startPos;
		startPosAct = startPos;
		if (velX >= 0)
		{
			SetMotorAcc(CalcMotorAcc(1.5f * Vel2Pulse(accX), atan2f(-1000.0f, 70.0f)/* velY Լ����  0.07*velX */));
		}
		else
		{
			SetMotorAcc(CalcMotorAcc(1.5f * Vel2Pulse(accX), atan2f(1000.0f, 70.0f)/* velY Լ���� -0.07*velX */));
		}
		timer = 0.0f;
	}
	
	/*��֤���ٶȰ�ȫ*/
	if(accX > MAXACC)
	{
		accX = MAXACC;
	}
	
	/*�����ٶȺ;��������ֵ*/
	targetDist = fabs(targetPos - startPosAct);
	timeAcc = fabs(velX) / accX;
	distAcc = 0.5f * accX * pow(timeAcc, 2);
	/*�����ٶȹ滮����*/
	if(2.0f * distAcc < targetDist)
	{
		distConst = targetDist - 2.0f * distAcc;
		timeConst = distConst / fabs(velX);
		if(timer <= timeAcc)    /*���ٶ�*/
		{
			expDist = targetDist - 0.5f * accX * pow(timer, 2);
			expSpeed = accX * (timer + 0.01f);
		}
		else if(timer > timeAcc && timer <= (timeAcc + timeConst))    /*���ٶ�*/
		{
			expDist = targetDist - distAcc - fabs(velX) * (timer - timeAcc);
			expSpeed = fabs(velX);
		}
		else if(timer > (timeAcc + timeConst) && timer <= (2.0f * timeAcc + timeConst))    /*���ٶ�*/
		{
			expDist = 0.5f * accX * pow(2.0f * timeAcc + timeConst - timer, 2);
			expSpeed = accX * (2.0f * timeAcc + timeConst - timer - 0.01f);
		}
		else if(timer > (2 * timeAcc + timeConst))    /*����Ԥ���켣*/
		{
			expDist = 0.0f;
			expSpeed = 0.0f;
		}
	}
	/*�������ٶȹ滮����*/
	else if(2.0f * distAcc >= targetDist)
	{
		if(timer <= timeAcc)    /*���ٶ�*/
		{
			expDist = targetDist - 0.5f * accX * pow(timer, 2);
			expSpeed = accX * (timer + 0.01f);
		}
		else if(timer > timeAcc && timer <= 2.0f * timeAcc)    /*���ٶ�*/
		{
			expDist = 0.5f * accX * pow(2.0f * timeAcc - timer, 2);
			expSpeed = accX * (2.0f * timeAcc - timer - 0.01f);
		}
		else if(timer > 2.0f * timeAcc)    /*����Ԥ���켣*/
		{
			expDist = 0.0f;
			expSpeed = 0.0f;
		}		
	}
	timer += 0.01f;
	
	/*�������ۿ��Դﵽ��λ��*/
	if(velX < 0.0f)
	{
		expPos = startPosAct - targetDist + expDist;
		expSpeed = -expSpeed;
	}
	else 
	{
		expPos = startPosAct + targetDist - expDist;
	}
	
	/*���ھ������PID����*/
	posErr = expPos - GetPosX();
	outputSpeed = expSpeed + posErr * PVEL;	
	/*��������ʱ���¹滮*/
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
	/*������������ٶȼ�����ȫʱ�����ƶ�*/
	if(fabs(outputSpeed) > INSANESPEED)
	{
		while(1)
		{
			LockWheel();
		}
	}
	
	/*����ĳ�ϵ��ٶ�����*/
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
	
	
	//�ٶȷ���������
	speedOut.v1 = Vel2Pulse( velX * 0.5f/*cos60*/ - velY * 0.8660254f/*cos30*/);
	speedOut.v2 = Vel2Pulse(-velX                                             );
	speedOut.v3 = Vel2Pulse( velX * 0.5f/*cos60*/ + velY * 0.8660254f/*cos30*/);

	
	//��̬����
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
	
	
	//�ٶȸ���������
	ThreeWheelVelControl(speedOut);
		
	return RETURNOK;
}
