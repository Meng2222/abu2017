/**
  ******************************************************************************
  * @file      movebase.h
  * @author    ������������������޹�˾
  * @version   V1.0.1
  * @date      2017/01/16
  * @brief     ���ļ�����������ȫ���ƶ���������
               ��Ҫ��������Ҫ���ܺ�������
  ******************************************************************************
  * @attention
  *
  *
  *
  *
  ******************************************************************************
  */
#ifndef __MOVEBASE_H
#define __MOVEBASE_H

/*
 *���ڵ�����ز����ĺ궨��
 */
//���ְ뾶 ��λ��m
#define WHEELRADIUS    0.076f
//���ٱ�
#define REDUCTION     (299.0f/14.0f)


//�������ٶ� ��λ������/s
#define MAXVEL 250000.0f
//���� m/s
//��ֱ�����ٶ�
#define VELVERTICAL 0.0f
//ˮƽ�����ٶ�
#define VELHORIZONTAL 1.0f

//�����������  ��λ������/s^2
//�趨��������ٶ�
#define MAXACC     50000.0f



/*
 * ���ڲ���ָ����ĺ궨��
 */

//������������
#define RETURNOK 		 1
//��Ч����
#define IVLDPARM   		 0
//��Ч����
#define VALIDPARM 		 1
//����������Χ
#define OUTOFRANGE 	    -2

/*
 *�ջ��õ��ĺ궨��
 */

//�Ƕȱջ���PID����
#define KPANG       1.5f
#define KIANG       0.0f
#define KDANG       0.0f
//λ�ñջ���PID����
#define KPPOS       0.8f
#define KIPOS       0.0f
#define KDPOS       0.0f
//���ƽǶȱջ���PID������ֵ
#define MAXANGPIDOUT   1080.0f
//����λ�ñջ���PID������ֵ
#define MAXPOSPIDOUT   2.8f

/*
 *�����궨��
 */
#define PI            3.141592653579f


/*
 *��λת��
 */

//�����ƺͽǶ����໥ת��
#define ANGTORAD(x) (float)((x)/180.0f*3.141592653579f)
#define RADTOANG(x) (float)((x)/3.141592653579f*180.0f)

/*
 *��ؽṹ��
 */
//���ڼ��ٶȵĽṹ��
typedef struct
{
	float wheel1;
	float wheel2;
	float wheel3;
	float wheel4;
}motorAcc_t;

//�����������ٶȵĽṹ������
//��λ������/s
typedef struct
{
	float v1;
	float v2;
	float v3;
	float v4;
}wheelSpeed_t;

/*
 *������ٶ�������غ���
 */

/**
* @brief  ���������ٶ�
* @param  carAcc : �����˵ĺϼ��ٶ�
* @param  angle : ������ƽ�Ʒ����
* @retval mototAcc �����ֵ�����ٶ�
* @author ACTION
*/
motorAcc_t CalcMotorAcc(float carAcc,float angle);

/**
* @brief  ���õ�����ٶ�
* @param  motorAcc �� ���ֵ�����ٶȽṹ��
* @retval ��
* @author ACTION
*/
void SetMotorAcc(motorAcc_t motorAcc);
/**
* @brief  ���õ���ٶ�
* @param  speed �� ���ֵ���ٶȽṹ��
* @retval ��
* @author ACTION
*/
void FourWheelVelControl(wheelSpeed_t speed);

/**
* @brief   ����ƶ�����
* @param  ��
* @retval ��
* @author ACTION
*/
void StopMove(void);

/**
* @brief   �����ٶ�ת��Ϊ��׼��λ�ٶ�
* @param  velPulse �� �ٶ� ����/��
* @retval velStandard �� �ٶ� ��/��
* @author ACTION
*/
float VelPulse2Standard(float velPulse);

/**
* @brief   ��׼��λ�ٶ�ת��Ϊ�����ٶ�
* @param  velStandard �� �ٶ� ��/��
* @retval velPulse �� �ٶ� ����/��
* @author ACTION
*/
float VelStandard2Pulse(float velStandard);

//�˶�����
int Move(float velX, float velY);

//����x�����ٶȺ���
float XSpeedDown(float posX, float dstX, float speedBegin);

//����x�����ٶȺ���
float XSpeedUp(float posX, float dstX, float speedEnd);

#endif

