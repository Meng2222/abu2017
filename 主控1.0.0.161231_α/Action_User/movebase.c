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
    VelCrl(1, -speed.v1);
    VelCrl(2, -speed.v2);
    VelCrl(3, -speed.v3);
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
                     ���������������            
============================================================
*/
/**
  * @brief  �켣����ֵ���㺯��
  * @param  pExpData:�˶�����ֵ�ṹ��ָ��
  * @param  velX:x�����ٶ�     mm/s
  * @param  startPos:��ʼλ��  mm
  * @param  targetPos:Ŀ��λ�� mm
  * @param  accX:x������ٶ�   mm/s^2
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
	
	
	//�������۾���������ٶȵľ���ֵ
	
	/*�����ٶȹ滮����*/
	if (2.0f * distAcc < targetDist)
	{
		timeAcc = fabs(velX) / accX;
		distAcc = 0.5f * accX * pow(timeAcc, 2);
		timeDec = timeAcc - ENDSPEED / accX;
		distDec = distAcc - pow(ENDSPEED, 2) / (2 * accX);
		distConst = targetDist - distAcc - distDec;
		timeConst = distConst / fabs(velX);
		
		if (moveTimer <= timeAcc)    /*���ٶ�*/
		{
			pExpData->dist = targetDist - 0.5f * accX * pow(moveTimer, 2);
			pExpData->speed = accX * (moveTimer + 0.01f);
		}
		else if (moveTimer > timeAcc && moveTimer <= (timeAcc + timeConst))    /*���ٶ�*/
		{
			pExpData->dist = targetDist - distAcc - fabs(velX) * (moveTimer - timeAcc);
			pExpData->speed = fabs(velX);
		}
		else if (moveTimer > (timeAcc + timeConst) && 
			    moveTimer <= (timeAcc + timeConst + timeDec))    /*���ٶ�*/
		{
			pExpData->dist = 0.5f * accX * (pow(timeAcc * 2.0f + timeConst - moveTimer, 2)
                               			  - pow(timeAcc - timeDec, 2));
			pExpData->speed = accX * (2.0f * timeAcc + timeConst - moveTimer - 0.01f);
		}
		else if (moveTimer > (timeAcc + timeConst + timeDec))    /*��������׼��ͣ��*/
		{
			pExpData->dist = ENDSPEED * ((timeAcc + timeConst + timeDec) - moveTimer);
			pExpData->speed = ENDSPEED;
		}
	}
	
	/*�������ٶȹ滮����*/
	else if (2.0f * distAcc >= targetDist)
	{
		timeAcc = sqrt((targetDist + pow(ENDSPEED, 2) / (2 * accX)) / accX);
		distAcc = 0.5f * accX * pow(timeAcc, 2);
		distDec = targetDist - distAcc;
		timeDec = timeAcc - ENDSPEED / accX;
		
		if (moveTimer <= timeAcc)    /*���ٶ�*/
		{
			pExpData->dist = targetDist - 0.5f * accX * pow(moveTimer, 2);
			pExpData->speed = accX * (moveTimer + 0.01f);
		}
		else if (moveTimer > timeAcc && moveTimer <= (timeAcc + timeDec))    /*���ٶ�*/
		{
			pExpData->dist = 0.5f * accX * pow(2.0f * timeAcc - moveTimer, 2)
			                             - pow(timeAcc - timeDec, 2);
			pExpData->speed = accX * (2.0f * timeAcc - moveTimer - 0.01f);
		}
		else if (moveTimer > (timeAcc + timeDec))    /*��������׼��ͣ��*/
		{
			pExpData->dist = ENDSPEED * ((timeAcc + timeDec) - moveTimer);
			pExpData->speed = ENDSPEED;
		}
	}
	
	
	//��������λ�ò����������ٶȵķ���
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
  * @brief  �ٶȵ��ں���
  * @param  pSpeedOut:ʵ������ٶȽṹ��ָ��
  * @param  pExpData:�˶�����ֵ�ṹ��ָ��
  * @param  velX:x�����ٶ�     mm/s
  * @retval None
  * @attention
  *         None
  */

extern int expSpeedp;

void SpeedAmend(wheelSpeed_t *pSpeedOut, expData_t *pExpData, float velX)
{
	float posErr = 0.0f;                                    //posErr:λ�����
	float outputSpeed = 0.0f;                               //outputSpeed:����ٶ�
	float velY = 0.0f;
	
	/*���ھ������PID����*/
	posErr = pExpData->pos - GetPosX();
	if (fabs(posErr) > 300.0f)
	{
		posErr = 0.0f;
	}
	outputSpeed = pExpData->speed + posErr * PVEL;
	
	expSpeedp = outputSpeed;
	
	/*������������ٶȼ�����ȫʱ�����ƶ�*/
	if(fabs(outputSpeed) > MAXSPEED)
	{
		while(1)
		{
			LockWheel();
		}
	}
	
	velX = outputSpeed;
	velY = fabs(0.07f * velX) + 100.0f;
	
	//�ٶȷ���������
	pSpeedOut->v1 = Vel2Pulse( velX * 0.5f/*cos60*/ - velY * 0.8660254f/*cos30*/);
	pSpeedOut->v2 = Vel2Pulse(-velX                                             );
	pSpeedOut->v3 = Vel2Pulse( velX * 0.5f/*cos60*/ + velY * 0.8660254f/*cos30*/);

	
	//��̬����
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
	
	//��ֹ����ٶȳ��������ٶ�
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

//x�������ƶ�
void MoveX(float velX)
{
	wheelSpeed_t speedOut = {0.0, 0.0, 0.0};
	float velY = 0.0f;
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
	
	ThreeWheelVelControl(speedOut);
}

/**
  * @brief  �ȼӼ����˶����ƺ���
  * @param  targetPos:Ŀ��λ�� mm
  * @param  velX:x�����ٶ�     mm/s
  * @param  accX:x������ٶ�   mm/s^2
  * @retval None
  * @attention
  *         �˺���û��ͣ�����
  */

void MoveTo(float targetPos, float velX, float accX)
{
	//�ٶȿ�����Ҫ�Ĺ��̱���
	static float formerTargetPos = 23333.0f;                 //formerTargetPos:�ж��Ƿ��ǲ�ͬ�˶�����
	static float startPos = 0.0f;
	expData_t expData = {0.0f, 0.0f, 0.0f};
	wheelSpeed_t speedOut = {0.0f, 0.0f, 0.0f};
	extern int mv1, mv2, mv3;
	
	//���˶����̳�ʼ��
	if(formerTargetPos != targetPos)
	{	
		formerTargetPos = targetPos;
		
		startPos = GetPosX();
		if (velX >= 0)
		{
			SetMotorAcc(CalcMotorAcc(MAXACC, atan2f(-1000.0f, 70.0f)/* velY Լ����  0.07*velX */));
		}
		else
		{
			SetMotorAcc(CalcMotorAcc(MAXACC, atan2f( 1000.0f, 70.0f)/* velY Լ���� -0.07*velX */));
		}
		moveTimer = 0.0f;
		moveTimFlag = 1;
	}

	//�켣����
	CalcPath(&expData, velX, startPos, targetPos, accX);

	//�ٶȵ��ڲ���
	SpeedAmend(&speedOut, &expData, velX);
	
	mv1 = speedOut.v1;
	mv2 = speedOut.v2;
	mv3 = speedOut.v3;
	
	//�ٶȸ���������
	ThreeWheelVelControl(speedOut);
}
