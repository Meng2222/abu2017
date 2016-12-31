/**
  ******************************************************************************
  * @file      
  * @author  2016Robcon-NEU Action团队
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
  * @brief  开环直线加上旋转
  * @param  Vel:车的和速度（正数 Vel>0）
  * @param  ward:车的行进方向
                   -180到+180
  * @param  Rotate:车自身的旋转速度（正数时右旋 俯视图）
  * @param  selfAngle:车自身的姿态角度
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
	
	
	/*矫正坐标系到世界坐标系*/
	ward=180-ward;
	ward=ward+120;
	ward=ward+selfAngle;
	
	/*计算自转脉冲数*/
	RotateVal=VelTransform(Rotate);
  /*计算前进总车速的脉冲数*/
	V_sum=VelTransform(Vel);
	
	/*角度向弧度变换*/
	ward=ward/180*PI;
	
	/*驱动器都给正数时车右旋（俯视角）     */
	
	/*三轮速度计算*/
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
			Vel_cfg(1,Acc[0],Acc[0]);			//加减速度配置
			Vel_cfg(2,Acc[1],Acc[1]);
			Vel_cfg(3,Acc[2],Acc[2]);		  //加减速度配置
	}
	/*三轮速度给定*/

	VelCrl(1,tarV[0]);
	VelCrl(2,tarV[1]);
	VelCrl(3,tarV[2]);
}
/**
  * @brief  直线闭环
  * @param  Vel:车的速度（正数 Vel>0）
  * @param  Ward：前进方向
  * @param  Refvalue：参考值
  * @param  Curvalue：实际值
  * @param  pPosition：位置闭环p参数
  * @param  Flag：调节标识位

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

	 /*角度误差和位置误差*/
	 errValue=(Refvalue-Curvalue)*Flag;
	 //USART_OUT(USART3,"ref=%d\tcur=%d\r\n",Refvalue,Curvalue);
	
	 errAngle=Refangle-Curangle;
	
	 /*如果误差在1度以内，则忽略*/
	 if(fabs(errValue)<10)
	 {
		  errValue=0;
	 }
	 if(fabs(errAngle)<1)
	 {
		  errAngle=0;
	 }
	 /*大于5度通过自旋调整*/
   if(errAngle>5)
	 {
			rotate=-ACTROTATE;
	 }
	 else  if(errAngle<-5)
	 {
		  rotate=ACTROTATE;
	 }
	 /*小于5度，通过pid调节*/
	 else
	 {
		  sumErrAngle+=errAngle;
		  rotate = -errAngle*pAngle-sumErrAngle*iAngle;
   }
	 sumErrValue+=errValue;
   /*需要调节的角度*/
	 changeWard=errValue*pPosition+sumErrValue*iPosition;
	 /*更新后角度*/
	 newWard=Ward+changeWard;
	 /*更新后速度*/
	 changeVel=Vel/cos(changeWard/180*3.14);
	 BasicLine(changeVel,newWard,rotate,Curangle);
}
/**
  * @brief  直线闭环(出发区使用)
  * @param  Vel:车的速度（正数 Vel>0）
  * @param  Ward：前进方向
  * @param  Refvalue：参考值
  * @param  Curvalue：实际值


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
	/*角度误差和位置误差*/
	 errValue=(Refvalue-Curvalue)*(-1);
	 errAngle=refAngle-Curangle;

	 //sumErrAngle+=errAngle;
	 rotate = -errAngle*pAngle;//-sumErrAngle*iAngle;

	 sumErrValue+=errValue;
   /*需要调节的角度*/
	 changeWard=errValue*pPosition;//+sumErrValue*iPosition;
	 /*更新后角度*/
	 newWard=Ward+changeWard;
	 /*更新后速度*/
	 changeVel=Vel/cos(changeWard/180*3.14);
	 startVel+=50;//出发区速度按照2000脉冲/s递增
	 if(startVel>changeVel)
	 {
		 startVel=changeVel;
	 }
	 BasicLine(startVel,newWard,rotate,Curangle);
}

/**
  * @brief  闭环画弧,画完返回值1，没画完返回值0
  * @param  Vel:车的线速度（正数 Vel>0）
  * @param  WardInit：开始的方向
  * @param  WardEnd：画完的方向
  * @param  Radius：弧的半径（正数为顺时针，负数为逆时针）
  * @param  IsRotate：IsRotate==1,要自转，IsRotate==0，不自转

  * @retval none
  * @author tzy
  */
int8_t BasicCircle(int Vel,int WardInit,int WardEnd,float Radius,int8_t IsRotate,int pAngle,float iAngle)
{
	float ActRadius,Pos_Ox,Pos_Oy,angle,WardAdd;
	int curLaser,refLaser;
	curLaser=GetLaserValue(BASIC_RIGHT);

	Pos_Ox=get_origin_x(Radius,WardInit);//坐标原点
	Pos_Oy=get_origin_y(Radius,WardInit);
	ActRadius=sqrt(pow((Get_POS_X()-Pos_Ox),2)+pow((Get_POS_Y()-Pos_Oy),2));//实际半径
	angle=((90-WardInit)-Acos(get_cos(Radius,WardInit)));//画圆的角度 
	
	if(IsRotate==1)
	{
		  //USART_OUT(USART3,"LAS%d\r\n",curLaser);
//			if(curLaser>50&&curLaser<1500)            //小于给定激光范围
//			{			
          refLaser=180;
//			}
//			else
//			{
//				  refLaser=curLaser;
//			}
	}

	WardAdd=WardInit+angle;	
	
	if(Radius>0)//顺时针
	{
		if(IsRotate==1)CloseLoopLine(Vel,WardAdd,refLaser,curLaser,WardAdd,Get_Angle(),P_POSITION_LASER,I_POSITION_LASER,pAngle,iAngle,-1);
    if(IsRotate==0)CloseLoopLine(Vel,WardAdd,Radius,ActRadius,0,Get_Angle(),P_POSITION,0.0,pAngle,iAngle,1);
	  if(WardAdd<=WardEnd)return 1;
	  if(WardAdd>WardEnd)return 0;		
	}
	if(Radius<0)//逆时针
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

	Pos_Ox=get_origin_x(Radius,WardInit);//坐标原点
	Pos_Oy=get_origin_y(Radius,WardInit);
	ActRadius=sqrt(pow((Get_POS_X()-Pos_Ox),2)+pow((Get_POS_Y()-Pos_Oy),2));//实际半径
	angle=((90-WardInit)-Acos(get_cos(Radius,WardInit)));//画圆的角度 
	
	if(IsRotate==1)
	{
		  //USART_OUT(USART3,"LAS%d\r\n",curLaser);
			if(curLaser>50&&curLaser<1500)            //小于给定激光范围
			{			
          refLaser=250;
			}
			else
			{
				  refLaser=curLaser;
			}
	}

	WardAdd=WardInit+angle;	
	
	if(Radius>0)//顺时针
	{
		if(IsRotate==1)CloseLoopLine(Vel,WardAdd,refLaser,curLaser,WardAdd,Get_Angle(),P_POSITION_LASER,I_POSITION_LASER,pAngle,iAngle,-1);
    if(IsRotate==0)CloseLoopLine(Vel,WardAdd,Radius,ActRadius,-30,Get_Angle(),P_POSITION,0.0,pAngle,iAngle,1);
	  if(WardAdd<=WardEnd)return 1;
	  if(WardAdd>WardEnd)return 0;		
	}
	if(Radius<0)//逆时针
	{
		if(IsRotate==1)CloseLoopLine(Vel,WardAdd,refLaser,curLaser,WardAdd,Get_Angle(),P_POSITION_LASER,I_POSITION_LASER,pAngle,iAngle,-1);
    if(IsRotate==0)CloseLoopLine(Vel,WardAdd,-Radius,ActRadius,-30,Get_Angle(),P_POSITION,0.0,pAngle,iAngle,-1);
		if(WardAdd>=WardEnd)return 1;
		if(WardAdd<WardEnd)return 0;
	}		
}
/**
  * @brief  摄像头找车
  * @param  curHeight:    电机高度 
  * @param  tarHeight:摄像头传回来的值
  * @param  maxHeight/minHeight:       升降台允许的最高值，与最低值(注意驱动器读回来的位置是负数，需要加绝对值)
  * @retval none
  * @author lxy                                                                                2
  */
void Height_CameraAdj(int curHeight,int tarHeight,int maxHeight,int minHeight)
{
	int speed;
	if(fabs(curHeight)>fabs(maxHeight))                             //位置不在给定范围内时以最大速度将升降台调回给定位置		
	{    
		speed=+UD_MAX_SPEED;
	}	
	else if(fabs(curHeight)<fabs(minHeight))
	{
		speed=-UD_MAX_SPEED;
	}
	else                                                            //如果在给定的范围内则根据摄像头进行调节
	{
		if(tarHeight<-50)
		{
			tarHeight=-50;
		}
		speed= -tarHeight/SIGHT_RANGE*UD_MAX_SPEED;   								//计算电机速度
	}
	if(speed>=200000)
		speed=200000;
	else if(speed<-200000)
		speed=-200000;
  //USART_OUT(USART3,"cur%d\ttar%d\tsp%d\r\n",(curHeight/100),tarHeight,(speed/100));	
	VelCrl(4,speed);
}


/**
  * @brief  控制升降台的高度
  * @retval none
  * @author lxy
  */
void UpdateUpDown(float PosX,float PosY,int CurHeight)
{	
	// y:  224-1224  小车出发区
	// y:  1224-2286 斜坡1
	// y： 2286-3239 山岗1
	// y:  3239-4017 斜坡2
	// y:  4017-5238 山岗2
	// y： 5238-6301 斜坡3
	// y： 6301-7056 高台
	Pos_cfg(4,500000,500000,210000);
	if(PosY<800)PosCrl_mm(-100);
	if(PosY>800.0&&PosY<2650.0)PosCrl_mm(-300);//557
	if(PosY>2650.0&&PosY<4114.0)PosCrl_mm(-510);
	if(PosY>4114.0&&PosY<7200.0)PosCrl_mm(-557);
	

}

/**
  * @brief  控制舵机风速
  * @retval none
  * @author ljc
**/
void UpdateWindSpeed(float PosX, float PosY)
{
    float adjSpeed;   //风速调整量
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
				 adjSpeed=-5;     //限制风速调整下限
			 //TIM_SetCompare2(TIM3,BASIC_WIND_SPEED+adjSpeed);
			  TIM_SetCompare2(TIM3,0.08*2000); 
		}
		else
		{
			 TIM_SetCompare2(TIM3,0.05*2000);
		}
}

/**
  * @brief  控制舵机角度
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
	
	  /*两个激光都照到小车把标志位置1*/
	  if(GetLaserValue(LEFT)<2000&&GetLaserValue(LEFT)>1000&&GetLaserValue(RIGHT)<2000&&GetLaserValue(RIGHT)>1000&&PosX>800)
		{
			//BEEP_ON;
			startSearchFlag=1;
		}
		/*如果标志位为0，保持固定位置*/
		if(startSearchFlag==0)
		{
			 ROBS_PosCrl(5.6,3000);	
		}
		/*如果标志位为1，开始寻找*/
		else
		{
			if(PosX>1400&&PosY<500)  //斜坡1
			{
				Robs_Control(leftLaser,rightLaser,10,60,ROBS_LOW_SPEED);    //舵机调节范围0~60
			}
			if(PosY>=500&&PosY<800)
			{
				Robs_Control(leftLaser,rightLaser,10,60,ROBS_LOW_SPEED);    //舵机调节范围0~60
			}
			if(PosY>=800&&PosY<1300)
			{
				Robs_Control(leftLaser,rightLaser,20,60,ROBS_HIGH_SPEED);    //舵机调节范围30~100
			}
			if(PosY>=1300&&PosY<2586)
			{
				Robs_Control(leftLaser,rightLaser,60,90,ROBS_HIGH_SPEED);    //舵机调节范围30~100
			}
			if(PosY>=2586&&PosY<3239) //山岗1
			{
				Robs_Control(leftLaser,rightLaser,60,90,ROBS_HIGH_SPEED);		//舵机调节范围30~110
			}
			if(PosY>=3039&&PosY<4017) //斜坡2
			{
				Robs_Control(leftLaser,rightLaser,60,100,ROBS_HIGH_SPEED);   //舵机调节范围30~100
			}
			if(PosY>=4017&&PosY<5238) //山岗2
			{
				Robs_Control(leftLaser,rightLaser,30,65,ROBS_MID_SPEED);   //舵机调节范围0~100
			}
			if(PosY>=5238&&PosY<6301) //斜坡3
			{		
				Robs_Control(leftLaser,rightLaser,35,45,ROBS_LOW_SPEED-20);   //舵机调节范围
			}
			if(PosY>=6301&&PosY<7056) //高台
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
  * @brief  高台处矫正坐标
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
  * @brief  把激光值转化为X坐标值
  * @param  LaserValue 激光值
  * @retval none
  * @author tzy
  */

int Get_LaserDistance(float LaserValue)
{
	return (678-((LaserValue+777.4)/15.9+162));
}


/************************ (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

