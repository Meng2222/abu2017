#include "ROBS.h"
#include "usart.h"
#include "stm32f4xx_tim.h"
#include "math.h"
#include "walk.h"
#include "GET_SET.h"

//�򿪶����Ť�����
void Enable_ROBS()
{
	USART_OUT(USART1,"#1 W 40,1,1\r\n");
	 //#1 W 40,1,1\r\n
}

//ʹ���ŷ�ģʽ
void Enable_ServoMode()
{
	USART_OUT(USART1,"#1 W 20,1,0\r\n");
	 //#1 W 20,1,0\r\n
}

//ʹ�ܵ��ģʽ
void Enable_MotorMode()
{
	USART_OUT(USART1,"#1 W 20,1,1\r\n");
   //#1 W 20,1,1\r\n
}



//�ŷ�ģʽ�º���

 //��vel���ٶ�ת��angle�Ƕ�
void ROBS_PosCrl(float angle,int vel)
{
	float pos;

	//Enable_ROBS();
  pos= (2048+angle/360*4096/30*52);
	USART_OUT(USART1,"#1 W 42,2,%d:46,2,%d\r\n",(int)pos,vel);
	//#1 W 42,2,pos:46,2,vel\r\nca
}
                           


//���ģʽ�º���


//��vel���ٶ�תtime ms,time=0����һֱת
void ROBS_VelCrl(int vel,int time)
{
	USART_OUT(USART1,"#1 W 46,2,%d:44,2,%d\r\n",vel,time);
	//#1 W 46,2,vel:44,2,time\r\n
}

void TurnLeft(int vel)
{
	  USART_OUT(USART1,"#1 W 46,2,%d:44,2,0\r\n",-vel);
}

void TurnRight(int vel)
{
	USART_OUT(USART1,"#1 W 46,2,%d:44,2,0\r\n",vel);

}

void Stop(void)
{
	USART_OUT(USART1,"#1 W 46,2,0:44,2,0\r\n");
} 

/**
  * @brief  ������С��
  * @param  laser_left��   �󼤹�ֵ
  * @param  laser_right��  �Ҽ���ֵ
  * @param  angle_or��     ���ת�����м��ĽǶ�
  * @param  max��          �����ת�����Ƕ�
  * @retval none
  * @note   example:       angle_or=60,max=10; ���ڷ�Χ�ǣ�60-10-40������60+10��
  * @author lxy  ljc
  */
static float  distance=0;
void Robs_Control(int laser_left,int laser_right,float AngleMin,float AngleMax,float RotateSpeed)
{
	static float  angle_cmd=5;
	static int    preDistance;      //��¼��һ�ξ�����Ϣ�������жϱ��μ���ֵ�ķ�Χ
	static int8_t flag_turn;
	static float  windSpeed;
	static int  adjVel;
	/* �ж�С����λ���Լ�������� */
	if((laser_left < LASER_RANGE && laser_left > 250) && (laser_right < 250 || laser_right > LASER_RANGE))
	{
		/* С�������ʱ */
	   flag_turn=0;
		 distance=laser_left;
		 preDistance=distance;
	}
	else if((laser_right<LASER_RANGE&&laser_right>250)&&(laser_left<250||laser_left>LASER_RANGE))
	{
	  /* С�����Ҳ�ʱ */	
	  flag_turn=1;
	  distance=laser_right;
		preDistance=distance;
	}
	else if((laser_left<LASER_RANGE&&laser_left>250)&&(laser_right<LASER_RANGE&&laser_right>250))
	{		
	  /* ��������ͬʱ����,���Һ�ǰһ�ξ����ֵС��200 */
		if((fabs(laser_left-preDistance)<200&&fabs(laser_right-preDistance)<200)||preDistance==0)
    {
				 distance= (laser_left+laser_right) /2;
				 preDistance=distance;
				 flag_turn=2;
		}
		/*�󼤹⿴��,���Һ�ǰһ�ξ����ֵС��200*/
		else if(fabs(laser_left-preDistance)<200)  //С�������
		{
			  flag_turn=0;
				distance=laser_left;
				preDistance=distance;	
		}
		/*�Ҽ��⿴��,���Һ�ǰһ�ξ����ֵС��200*/
		else   if(fabs(laser_right-preDistance)<200)                                //С�����ұ�
    {
		    flag_turn=1;
			  distance=laser_right;
				preDistance=distance;
	  }

	}
	/* ��ÿ�ֲ�ͬ��״̬������*/
  //USART_OUT(USART3,"A%d\tA%d\t",laser_left,laser_right);
	switch(flag_turn)
	{
		case 0://��ת��������ٶ������ɷ���
			if(distance>=1000)
			{
				  RotateSpeed=2*RotateSpeed;
			}
			angle_cmd=angle_cmd-RotateSpeed/distance;
    //  USART_OUT(USART3,"case0\t");
			break;
		case 1://��ת��������ٶ������ɷ���
			if(distance>=1000)
			{
				  RotateSpeed=2*RotateSpeed;
			}
			angle_cmd=angle_cmd+RotateSpeed/distance;
		 // USART_OUT(USART3,"case1\t");
			break;
		case 2://����
			angle_cmd=angle_cmd;
	  	//USART_OUT(USART3,"case2\t");
		  break;
	}
	
	/* ���Ƹ����ĽǶȵ������� */
	if(angle_cmd<AngleMin)
	 angle_cmd=AngleMin;
	
	if(angle_cmd>AngleMax)
	 angle_cmd=AngleMax;

  windSpeed=(distance-1000)*0.02;	//����ֵ150    ��0.65~0.85֮�����
	Set_Windspeed(windSpeed);
	
	adjVel=(distance-2000)*0.2;
//	if(adjVel>200)
//		adjVel=200;
	if(adjVel<-100)
		adjVel=-100;
	Set_AdjVel(adjVel);
  	
	/* ���Ƕ��������� */
	ROBS_PosCrl(angle_cmd,3000);	
}

int SpeedLimit(int PosY)
{
	 int tarDis,curDis,errDis;
	 tarDis=(PosY-6200)*5+500;
	 curDis=distance;
	 errDis=curDis-tarDis;
	// USART_OUT(USART3,"PosY=%d\ttarDis=%d\tcurDis=%d\r\n",PosY,tarDis,(int)distance);

	 if(errDis>=0)
		 return 0;
	 else
	 {
		 if(errDis<-500)
			 errDis=-500;
		 return errDis;
			 
	 }
}

