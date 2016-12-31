#ifndef __ROBS_H
#define __ROBS_H
//�򿪶����Ť�����
void Enable_ROBS(void);

//ʹ���ŷ�ģʽ
void Enable_ServoMode(void) ;

//ʹ�ܵ��ģʽ
void Enable_MotorMode(void);

void ROBS_VelCrl(int vel,int time);

void ROBS_PosCrl(float angle,int vel);

void TurnLeft(int vel);

void TurnRight(int vel);

void Stop(void);

void Robs_Control(int laser_left,int laser_right,float AngleMin,float AngleMax,float RotateSpeed);

int SpeedLimit(int PosY);
#endif
