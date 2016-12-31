#include "ROBS.h"
#include "usart.h"
#include "stm32f4xx_tim.h"
#include "math.h"
#include "walk.h"
#include "GET_SET.h"

//打开舵机的扭力输出
void Enable_ROBS()
{
	USART_OUT(USART1,"#1 W 40,1,1\r\n");
	 //#1 W 40,1,1\r\n
}

//使能伺服模式
void Enable_ServoMode()
{
	USART_OUT(USART1,"#1 W 20,1,0\r\n");
	 //#1 W 20,1,0\r\n
}

//使能电机模式
void Enable_MotorMode()
{
	USART_OUT(USART1,"#1 W 20,1,1\r\n");
   //#1 W 20,1,1\r\n
}



//伺服模式下函数

 //以vel的速度转到angle角度
void ROBS_PosCrl(float angle,int vel)
{
	float pos;

	//Enable_ROBS();
  pos= (2048+angle/360*4096/30*52);
	USART_OUT(USART1,"#1 W 42,2,%d:46,2,%d\r\n",(int)pos,vel);
	//#1 W 42,2,pos:46,2,vel\r\nca
}
                           


//电机模式下函数


//以vel的速度转time ms,time=0代表一直转
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
  * @brief  激光找小车
  * @param  laser_left：   左激光值
  * @param  laser_right：  右激光值
  * @param  angle_or：     舵机转动的中间点的角度
  * @param  max：          舵机的转动最大角度
  * @retval none
  * @note   example:       angle_or=60,max=10; 调节范围是（60-10-40）到（60+10）
  * @author lxy  ljc
  */
static float  distance=0;
void Robs_Control(int laser_left,int laser_right,float AngleMin,float AngleMax,float RotateSpeed)
{
	static float  angle_cmd=5;
	static int    preDistance;      //记录上一次距离信息，用于判断本次激光值的范围
	static int8_t flag_turn;
	static float  windSpeed;
	static int  adjVel;
	/* 判断小车的位置以及计算距离 */
	if((laser_left < LASER_RANGE && laser_left > 250) && (laser_right < 250 || laser_right > LASER_RANGE))
	{
		/* 小车在左侧时 */
	   flag_turn=0;
		 distance=laser_left;
		 preDistance=distance;
	}
	else if((laser_right<LASER_RANGE&&laser_right>250)&&(laser_left<250||laser_left>LASER_RANGE))
	{
	  /* 小车在右侧时 */	
	  flag_turn=1;
	  distance=laser_right;
		preDistance=distance;
	}
	else if((laser_left<LASER_RANGE&&laser_left>250)&&(laser_right<LASER_RANGE&&laser_right>250))
	{		
	  /* 两个激光同时看到,并且和前一次距离差值小于200 */
		if((fabs(laser_left-preDistance)<200&&fabs(laser_right-preDistance)<200)||preDistance==0)
    {
				 distance= (laser_left+laser_right) /2;
				 preDistance=distance;
				 flag_turn=2;
		}
		/*左激光看到,并且和前一次距离差值小于200*/
		else if(fabs(laser_left-preDistance)<200)  //小车在左边
		{
			  flag_turn=0;
				distance=laser_left;
				preDistance=distance;	
		}
		/*右激光看到,并且和前一次距离差值小于200*/
		else   if(fabs(laser_right-preDistance)<200)                                //小车在右边
    {
		    flag_turn=1;
			  distance=laser_right;
				preDistance=distance;
	  }

	}
	/* 对每种不同的状态做处理*/
  //USART_OUT(USART3,"A%d\tA%d\t",laser_left,laser_right);
	switch(flag_turn)
	{
		case 0://左转，舵机的速度与距离成反比
			if(distance>=1000)
			{
				  RotateSpeed=2*RotateSpeed;
			}
			angle_cmd=angle_cmd-RotateSpeed/distance;
    //  USART_OUT(USART3,"case0\t");
			break;
		case 1://右转，舵机的速度与距离成反比
			if(distance>=1000)
			{
				  RotateSpeed=2*RotateSpeed;
			}
			angle_cmd=angle_cmd+RotateSpeed/distance;
		 // USART_OUT(USART3,"case1\t");
			break;
		case 2://不动
			angle_cmd=angle_cmd;
	  	//USART_OUT(USART3,"case2\t");
		  break;
	}
	
	/* 限制给出的角度的上下限 */
	if(angle_cmd<AngleMin)
	 angle_cmd=AngleMin;
	
	if(angle_cmd>AngleMax)
	 angle_cmd=AngleMax;

  windSpeed=(distance-1000)*0.02;	//基础值150    ，0.65~0.85之间调整
	Set_Windspeed(windSpeed);
	
	adjVel=(distance-2000)*0.2;
//	if(adjVel>200)
//		adjVel=200;
	if(adjVel<-100)
		adjVel=-100;
	Set_AdjVel(adjVel);
  	
	/* 将角度命令给舵机 */
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

