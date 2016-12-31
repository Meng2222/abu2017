/**
  ******************************************************************************
  * @file      
  * @author  2016Robcon-NEU Action�Ŷ�
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include <math.h>
#include "walk.h"
#include "GET_SET.h"
#include "stdint.h"
#include "action_math.h"
#include "math.h"
#include "elmo.h"
#include "timer.h"
#include "robs.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"
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

void LockWheel(void)
{
	VelCrl(1,0);
	VelCrl(2,0);
	VelCrl(3,0);
  VelCrl(5,0);
  VelCrl(6,0);	
}

void PosCrl_mm(int Dis)
{
	 int pulse;     //464-184
	 pulse=Dis*377;
	 PosCrl(4,POS_ABS,pulse); 
}
/**
  * @brief  ����ֱ�߼�����ת
  * @param  Vel:���ĺ��ٶȣ����� Vel>0��
  * @param  ward:�����н�����
                   -180��+180
  * @param  Rotate:���������ת�ٶȣ�����ʱ���� ����ͼ��
  * @param  selfAngle:���������̬�Ƕ�
  * @retval none
  * @author lxy
  */
void BasicLine(int Vel,float ward,float Rotate,float selfAngle)
{ 
	static  int tarV[3];
	        int V_sum;	
					int actV[3];
					int RotateVal;
	        int Acc[3],DifVel[3];
					int maxV=0;
	        int i=0;
	        float reduceP;
	
	ReadActualVel(1);
	ReadActualVel(2);
	ReadActualVel(3);
	
	
	/*��������ϵ����������ϵ*/
	ward=180-ward;
	ward=ward+120;
	ward=ward+selfAngle;
	
	/*������ת������*/
	RotateVal=VelTransform(Rotate);
  /*����ǰ���ܳ��ٵ�������*/
	V_sum=VelTransform(Vel);
	
	/*�Ƕ��򻡶ȱ任*/
	ward=ward/180*PI;
	
	/*��������������ʱ�����������ӽǣ�     */
	
	/*�����ٶȼ���*/
	actV[0]=Get_Vel(1);
	actV[1]=Get_Vel(2);
	actV[2]=Get_Vel(3);
	
	tarV[2]= (V_sum*cos(ward))+RotateVal;
	tarV[1]=-(V_sum*cos(ward+PI/3))+RotateVal;
	tarV[0]=-(V_sum*cos(PI/3-ward))+RotateVal;
	for(i=0;i<3;i++)
	{
		if(fabs(tarV[i])>maxV)
			  maxV=fabs(tarV[i]);
	}
	USART_OUT(USART3,"maxV=%d\r\n",maxV/10);
	if(maxV>210000)
	{
		for(i=0;i<3;i++)
		{
			reduceP=(float)tarV[i]/(float)maxV;
			tarV[i]=210000*reduceP;
		}
	}

  DifVel[0] = tarV[0] - actV[0];
  DifVel[1] = tarV[1] - actV[1];
	DifVel[2] = tarV[2] - actV[2];
	
	Acc[0]=fabs(DifVel[0])*100;
	Acc[1]=fabs(DifVel[1])*100;
	Acc[2]=fabs(DifVel[2])*100;
	USART_OUT(USART3,"actV[1]=%d\tactV[2]=%d\tactV[3]=%d\r\n",actV[0]/10,actV[1]/10,actV[2]/10);
  USART_OUT(USART3,"tarV[1]=%d\ttarV[2]=%d\ttarV[3]=%d\r\n",tarV[0]/10,tarV[1]/10,tarV[2]/10);
 // USART_OUT(USART3,"acc[1]=%d\tacc[2]=%d\tacc[3]=%d\r\n",Acc[0]/10,Acc[1]/10,Acc[2]/10);
	
	if(DifVel[0]!=0&&DifVel[1]!=0&&DifVel[2]!=0)
	{
			Vel_cfg(1,Acc[0],Acc[0]);			//�Ӽ��ٶ�����
			Vel_cfg(2,Acc[1],Acc[1]);
			Vel_cfg(3,Acc[2],Acc[2]);		  //�Ӽ��ٶ�����
	}
	/*�����ٶȸ���*/

	VelCrl(1,tarV[0]);
	VelCrl(2,tarV[1]);
	VelCrl(3,tarV[2]);
}
/**
  * @brief  ֱ�߱ջ�
  * @param  Vel:�����ٶȣ����� Vel>0��
  * @param  Ward��ǰ������
  * @param  Refvalue���ο�ֵ
  * @param  Curvalue��ʵ��ֵ
  * @param  pPosition��λ�ñջ�p����
  * @param  Flag�����ڱ�ʶλ

  * @retval none
  * @author ljc
  */

void CloseLoopLine(float Vel,int Ward,int Refvalue,int Curvalue,float Refangle,float Curangle ,float pPosition,float iPosition,int pAngle,float iAngle,int8_t Flag)
{
	 int                errValue;
	 float	            errAngle;
	 float              newWard;
	 float              changeWard;
	 int                changeVel;
	 int                rotate;
	 static  float      sumErrAngle;
	 static  int        sumErrValue;

	 /*�Ƕ�����λ�����*/
	 errValue=(Refvalue-Curvalue)*Flag;
	 //USART_OUT(USART3,"ref=%d\tcur=%d\r\n",Refvalue,Curvalue);
	
	 errAngle=Refangle-Curangle;
	
	 /*��������1�����ڣ������*/
	 if(fabs(errValue)<10)
	 {
		  errValue=0;
	 }
	 if(fabs(errAngle)<1)
	 {
		  errAngle=0;
	 }
	 /*����5��ͨ����������*/
   if(errAngle>5)
	 {
			rotate=-ACTROTATE;
	 }
	 else  if(errAngle<-5)
	 {
		  rotate=ACTROTATE;
	 }
	 /*С��5�ȣ�ͨ��pid����*/
	 else
	 {
		  sumErrAngle+=errAngle;
		  rotate = -errAngle*pAngle-sumErrAngle*iAngle;
   }
	 sumErrValue+=errValue;
   /*��Ҫ���ڵĽǶ�*/
	 changeWard=errValue*pPosition+sumErrValue*iPosition;
	 /*���º�Ƕ�*/
	 newWard=Ward+changeWard;
	 /*���º��ٶ�*/
	 changeVel=Vel/cos(changeWard/180*3.14);
	 BasicLine(changeVel,newWard,rotate,Curangle);
}
/**
  * @brief  ֱ�߱ջ�(������ʹ��)
  * @param  Vel:�����ٶȣ����� Vel>0��
  * @param  Ward��ǰ������
  * @param  Refvalue���ο�ֵ
  * @param  Curvalue��ʵ��ֵ


  * @retval none
  * @author ljc
  */

void StartLine(float Vel,int Ward,int Refvalue,int Curvalue,float Curangle,int PosX)
{
	 int                errValue;
	 float	            errAngle;
	 float              newWard;
	 float              changeWard;
	 int                changeVel;
	 int                rotate;
	 float              refAngle;
	// static  float      sumErrAngle;
	 static  int        sumErrValue;	
	 static   float     startVel            =0;
	 float              pPosition           =0.31;
	// float							iPosition						=0.08;
	 int 								pAngle							=40;
  // float              iAngle							=0.0;
   
	
	
	 refAngle=-30;//((X1-PosX)/(float)X1)*30;
	// USART_OUT(USART3,"angle=%d\tX=%d\t\n",(int)refAngle*10,PosX);
	/*�Ƕ�����λ�����*/
	 errValue=(Refvalue-Curvalue)*(-1);
	 errAngle=refAngle-Curangle;

	 //sumErrAngle+=errAngle;
	 rotate = -errAngle*pAngle;//-sumErrAngle*iAngle;

	 sumErrValue+=errValue;
   /*��Ҫ���ڵĽǶ�*/
	 changeWard=errValue*pPosition;//+sumErrValue*iPosition;
	 /*���º�Ƕ�*/
	 newWard=Ward+changeWard;
	 /*���º��ٶ�*/
	 changeVel=Vel/cos(changeWard/180*3.14);
	 startVel+=50;//�������ٶȰ���2000����/s����
	 if(startVel>changeVel)
	 {
		 startVel=changeVel;
	 }
	 BasicLine(startVel,newWard,rotate,Curangle);
}

/**
  * @brief  �ջ�����,���귵��ֵ1��û���귵��ֵ0
  * @param  Vel:�������ٶȣ����� Vel>0��
  * @param  WardInit����ʼ�ķ���
  * @param  WardEnd������ķ���
  * @param  Radius�����İ뾶������Ϊ˳ʱ�룬����Ϊ��ʱ�룩
  * @param  IsRotate��IsRotate==1,Ҫ��ת��IsRotate==0������ת

  * @retval none
  * @author tzy
  */
int8_t BasicCircle(int Vel,int WardInit,int WardEnd,float Radius,int8_t IsRotate,int pAngle,float iAngle)
{
	float ActRadius,Pos_Ox,Pos_Oy,angle,WardAdd;
	int curLaser,refLaser;
	curLaser=GetLaserValue(BASIC_RIGHT);

	Pos_Ox=get_origin_x(Radius,WardInit);//����ԭ��
	Pos_Oy=get_origin_y(Radius,WardInit);
	ActRadius=sqrt(pow((Get_POS_X()-Pos_Ox),2)+pow((Get_POS_Y()-Pos_Oy),2));//ʵ�ʰ뾶
	angle=((90-WardInit)-Acos(get_cos(Radius,WardInit)));//��Բ�ĽǶ� 
	
	if(IsRotate==1)
	{
		  //USART_OUT(USART3,"LAS%d\r\n",curLaser);
//			if(curLaser>50&&curLaser<1500)            //С�ڸ������ⷶΧ
//			{			
          refLaser=180;
//			}
//			else
//			{
//				  refLaser=curLaser;
//			}
	}

	WardAdd=WardInit+angle;	
	
	if(Radius>0)//˳ʱ��
	{
		if(IsRotate==1)CloseLoopLine(Vel,WardAdd,refLaser,curLaser,WardAdd,Get_Angle(),P_POSITION_LASER,I_POSITION_LASER,pAngle,iAngle,-1);
    if(IsRotate==0)CloseLoopLine(Vel,WardAdd,Radius,ActRadius,0,Get_Angle(),P_POSITION,0.0,pAngle,iAngle,1);
	  if(WardAdd<=WardEnd)return 1;
	  if(WardAdd>WardEnd)return 0;		
	}
	if(Radius<0)//��ʱ��
	{
		if(IsRotate==1)CloseLoopLine(Vel,WardAdd,refLaser,curLaser,WardAdd,Get_Angle(),P_POSITION_LASER,I_POSITION_LASER,pAngle,iAngle,-1);
    if(IsRotate==0)CloseLoopLine(Vel,WardAdd,-Radius,ActRadius,0,Get_Angle(),P_POSITION,0.0,pAngle,iAngle,-1);
		if(WardAdd>=WardEnd)return 1;
		if(WardAdd<WardEnd)return 0;
	}		
}


int8_t StartCircle(int Vel,int WardInit,int WardEnd,float Radius,int8_t IsRotate,int pAngle,float iAngle)
{
	float ActRadius,Pos_Ox,Pos_Oy,angle,WardAdd;
	int curLaser,refLaser;
	curLaser=GetLaserValue(BASIC_RIGHT);

	Pos_Ox=get_origin_x(Radius,WardInit);//����ԭ��
	Pos_Oy=get_origin_y(Radius,WardInit);
	ActRadius=sqrt(pow((Get_POS_X()-Pos_Ox),2)+pow((Get_POS_Y()-Pos_Oy),2));//ʵ�ʰ뾶
	angle=((90-WardInit)-Acos(get_cos(Radius,WardInit)));//��Բ�ĽǶ� 
	
	if(IsRotate==1)
	{
		  //USART_OUT(USART3,"LAS%d\r\n",curLaser);
			if(curLaser>50&&curLaser<1500)            //С�ڸ������ⷶΧ
			{			
          refLaser=250;
			}
			else
			{
				  refLaser=curLaser;
			}
	}

	WardAdd=WardInit+angle;	
	
	if(Radius>0)//˳ʱ��
	{
		if(IsRotate==1)CloseLoopLine(Vel,WardAdd,refLaser,curLaser,WardAdd,Get_Angle(),P_POSITION_LASER,I_POSITION_LASER,pAngle,iAngle,-1);
    if(IsRotate==0)CloseLoopLine(Vel,WardAdd,Radius,ActRadius,-30,Get_Angle(),P_POSITION,0.0,pAngle,iAngle,1);
	  if(WardAdd<=WardEnd)return 1;
	  if(WardAdd>WardEnd)return 0;		
	}
	if(Radius<0)//��ʱ��
	{
		if(IsRotate==1)CloseLoopLine(Vel,WardAdd,refLaser,curLaser,WardAdd,Get_Angle(),P_POSITION_LASER,I_POSITION_LASER,pAngle,iAngle,-1);
    if(IsRotate==0)CloseLoopLine(Vel,WardAdd,-Radius,ActRadius,-30,Get_Angle(),P_POSITION,0.0,pAngle,iAngle,-1);
		if(WardAdd>=WardEnd)return 1;
		if(WardAdd<WardEnd)return 0;
	}		
}
/**
  * @brief  ����ͷ�ҳ�
  * @param  curHeight:    ����߶� 
  * @param  tarHeight:�s����ͷ��������ֵ
  * @param  maxHeight/minHeight:       ����̨��������ֵ�������ֵ(ע����������������λ���Ǹ�������Ҫ�Ӿ���ֵ)
  * @retval none
  * @author lxy                                                                                2
  */
void Height_CameraAdj(int curHeight,int tarHeight,int maxHeight,int minHeight)
{
	int speed;
	if(fabs(curHeight)>fabs(maxHeight))                             //λ�ò��ڸ�����Χ��ʱ������ٶȽ�����̨���ظ���λ��		
	{    
		speed=+UD_MAX_SPEED;
	}	
	else if(fabs(curHeight)<fabs(minHeight))
	{
		speed=-UD_MAX_SPEED;
	}
	else                                                            //����ڸ����ķ�Χ�����������ͷ���е���
	{
		if(tarHeight<-50)
		{
			tarHeight=-50;
		}
		speed= -tarHeight/SIGHT_RANGE*UD_MAX_SPEED;   								//�������ٶ�
	}
	if(speed>=200000)
		speed=200000;
	else if(speed<-200000)
		speed=-200000;
  //USART_OUT(USART3,"cur%d\ttar%d\tsp%d\r\n",(curHeight/100),tarHeight,(speed/100));	
	VelCrl(4,speed);
}


/**
  * @brief  ��������̨�ĸ߶�
  * @retval none
  * @author lxy
  */
void UpdateUpDown(float PosX,float PosY,int CurHeight)
{	
	// y:  224-1224  С��������
	// y:  1224-2286 б��1
	// y�� 2286-3239 ɽ��1
	// y:  3239-4017 б��2
	// y:  4017-5238 ɽ��2
	// y�� 5238-6301 б��3
	// y�� 6301-7056 ��̨
	Pos_cfg(4,500000,500000,210000);
	if(PosY<800)PosCrl_mm(-100);
	if(PosY>800.0&&PosY<2650.0)PosCrl_mm(-300);//557
	if(PosY>2650.0&&PosY<4114.0)PosCrl_mm(-510);
	if(PosY>4114.0&&PosY<7200.0)PosCrl_mm(-557);
	

}

/**
  * @brief  ���ƶ������
  * @retval none
  * @author ljc
**/
void UpdateWindSpeed(float PosX, float PosY)
{
    float adjSpeed;   //���ٵ�����
	  float addDutyfactor=0.0025;
	  static float dutyFactor=0.05;
	
	  adjSpeed=Get_Windspeed();

		if(PosX>800&&PosY<1900)
		{
			 dutyFactor+=addDutyfactor;
			 if(dutyFactor>=0.085)
				 dutyFactor=0.085;
			 TIM_SetCompare2(TIM3,dutyFactor*2000);
		}
		else if(PosY>=1900&&PosY<4000)
		{
			 TIM_SetCompare2(TIM3,BASIC_WIND_SPEED+adjSpeed);
			 //TIM_SetCompare2(TIM3,0.08*2000);
		}
		else if(PosY>=4000&&PosY<7800)
		{
			 if(adjSpeed<-5)
				 adjSpeed=-5;     //���Ʒ��ٵ�������
			 //TIM_SetCompare2(TIM3,BASIC_WIND_SPEED+adjSpeed);
			  TIM_SetCompare2(TIM3,0.08*2000); 
		}
		else
		{
			 TIM_SetCompare2(TIM3,0.05*2000);
		}
}

/**
  * @brief  ���ƶ���Ƕ�
  * @retval none
  * @author ljc
**/
void UpdateAngle(float PosX,float PosY)
{
    static int   startSearchFlag=0;
					 int   leftLaser; 
		       int   rightLaser;
	
	
		leftLaser  = GetLaserValue(LEFT);
		rightLaser = GetLaserValue(RIGHT);
	  //USART_OUT(USART3,"l=%d\t,r=%d\t\r\n",leftLaser,rightLaser);
	
	  /*�������ⶼ�յ�С���ѱ�־λ��1*/
	  if(GetLaserValue(LEFT)<2000&&GetLaserValue(LEFT)>1000&&GetLaserValue(RIGHT)<2000&&GetLaserValue(RIGHT)>1000&&PosX>800)
		{
			//BEEP_ON;
			startSearchFlag=1;
		}
		/*�����־λΪ0�����̶ֹ�λ��*/
		if(startSearchFlag==0)
		{
			 ROBS_PosCrl(5.6,3000);	
		}
		/*�����־λΪ1����ʼѰ��*/
		else
		{
			if(PosX>1400&&PosY<500)  //б��1
			{
				Robs_Control(leftLaser,rightLaser,10,60,ROBS_LOW_SPEED);    //������ڷ�Χ0~60
			}
			if(PosY>=500&&PosY<800)
			{
				Robs_Control(leftLaser,rightLaser,10,60,ROBS_LOW_SPEED);    //������ڷ�Χ0~60
			}
			if(PosY>=800&&PosY<1300)
			{
				Robs_Control(leftLaser,rightLaser,20,60,ROBS_HIGH_SPEED);    //������ڷ�Χ30~100
			}
			if(PosY>=1300&&PosY<2586)
			{
				Robs_Control(leftLaser,rightLaser,60,90,ROBS_HIGH_SPEED);    //������ڷ�Χ30~100
			}
			if(PosY>=2586&&PosY<3239) //ɽ��1
			{
				Robs_Control(leftLaser,rightLaser,60,90,ROBS_HIGH_SPEED);		//������ڷ�Χ30~110
			}
			if(PosY>=3039&&PosY<4017) //б��2
			{
				Robs_Control(leftLaser,rightLaser,60,100,ROBS_HIGH_SPEED);   //������ڷ�Χ30~100
			}
			if(PosY>=4017&&PosY<5238) //ɽ��2
			{
				Robs_Control(leftLaser,rightLaser,30,65,ROBS_MID_SPEED);   //������ڷ�Χ0~100
			}
			if(PosY>=5238&&PosY<6301) //б��3
			{		
				Robs_Control(leftLaser,rightLaser,35,45,ROBS_LOW_SPEED-20);   //������ڷ�Χ
			}
			if(PosY>=6301&&PosY<7056) //��̨
			{	
				Robs_Control(leftLaser,rightLaser,-10,10,ROBS_LOW_SPEED);		
			}
			if(PosY>=7056&&PosY<7800) //
			{
				Robs_Control(leftLaser,rightLaser,-30,0,ROBS_LOW_SPEED);	
			}
			if(PosY>10000)
			{
				ROBS_PosCrl(90,3000);
			}
		}
}

/**
  * @brief  ��̨����������
  * @retval none
  * @author ljc
**/
void AdjPosX(float PosX,int LaserValue)
{
	 int accurateX;
   accurateX=Get_LaserDistance(LaserValue);
   Set_Offset_X(PosX-accurateX);
}


void AdjPosY(float PosY)
{
   Set_Offset_Y(PosY-ACCURATEY);
}


/**
  * @brief  �Ѽ���ֵת��ΪX����ֵ
  * @param  LaserValue ����ֵ
  * @retval none
  * @author tzy
  */

int Get_LaserDistance(float LaserValue)
{
	return (678-((LaserValue+777.4)/15.9+162));
}


/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

