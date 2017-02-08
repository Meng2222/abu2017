/**
  ******************************************************************************
  * @file      movebase.h
  * @author    ST42 & Meng22
  * @version   V2.0.0
  * @date      2017/02/07
  * @brief     Robocon2017�˶�����
  ******************************************************************************
  * @attention
  *            None
  ******************************************************************************
  */
#ifndef __MOVEBASE_H
#define __MOVEBASE_H


/*
 *��ȫ���ֵ�����ز����궨��
 */
 
//���ְ뾶 ��λ:mm
#define WHEELRADIUS 76.0f
//��̬����ʱ������ת�뾶 ��λ:mm
#define ROTATERAD   500.0f

//���ٱ�
#define REDUCTION (299.0f/14.0f)

//ÿȦ������
#define STDPULSE 2000.0f

//�������ٶ�   ��λ:����/s
#define MAXVEL 250000.0f
//��������ٶ� ��λ:mm/s^2
#define MAXACC 5000.0f


/*
 * ���ڲ���ָ����ĺ궨��
 */

//������������
#define RETURNOK 		 1
//��Ч����
#define INVALIDPARM      0
//��Ч����
#define VALIDPARM 		 1
//����������Χ
#define OUTOFRANGE 	    -2

/*
 *�ջ��õ��ĺ궨��
 */

//��̬����PID
#define PPOSE 1.0f

//�ٶȱջ�PID
#define PVEL 10.0f


/*
 *�����궨��
 */
 
#define PI  3.141592653579f


/*
 *��λת��
 */

//�����ƺͽǶ����໥ת��
#define ANGTORAD(x) (float)((x) / 180.0f * 3.141592653579f)
#define RADTOANG(x) (float)((x) / 3.141592653579f * 180.0f)


/*
 *��ؽṹ��
 */

//���ٶȵĽṹ��
typedef struct
{
	float wheel1;
	float wheel2;
	float wheel3;
}motorAcc_t;

//�������ٶȵĽṹ��  ��λ:����/s
typedef struct
{
	float v1;
	float v2;
	float v3;
}wheelSpeed_t;


/*
 *������ٶ�������غ���
 */

/**
  * @brief  ���������ٶ�
  * @param  carAcc:�����˵ĺϼ��ٶ�
  * @param  angle:������ƽ�Ʒ����
  * @retval mototAcc:���ֵ�����ٶ�
  */
motorAcc_t CalcMotorAcc(float carAcc,float angle);

/**
  * @brief  ���õ�����ٶ�
  * @param  motorAcc:���ֵ�����ٶȽṹ��
  * @retval None
  */
void SetMotorAcc(motorAcc_t motorAcc);

/**
  * @brief  �������������������ٶ�
  * @param  speed:���ֵ���ٶȽṹ��
  * @retval None
  */
void ThreeWheelVelControl(wheelSpeed_t speed);

/**
  * @brief  ����ƶ�����
  * @param  ��
  * @retval ��
  */
void LockWheel(void);

/**
  * @brief  �����ٶ�ת��Ϊ��׼��λ�ٶ�
  * @param  pulse:�ٶ� ����/s
  * @retval vel:�ٶ�   m/s
  */
float Pulse2Vel(float pulse);

/**
  * @brief  ��׼��λ�ٶ�ת��Ϊ�����ٶ�
  * @param  vel:�ٶ�   m/s
  * @retval pulse:�ٶ� ����/s
  */
float Vel2Pulse(float vel);

/**
  * @brief  �˶����ƺ���
  * @param  velX:x�����ٶ�     mm/s
  * @param  startPos:��ʼλ��  mm
  * @param  targetPos:Ŀ��λ�� mm
  * @param  accX:x������ٶ�   mm/s^2
  * @retval RETURNOK:״̬�궨��
  */
int Move(float velX, float startPos, float targetPos, float accX);

#endif

