#include <includes.h>
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "cylinder.h"
#include "robs.h"
#include "action_math.h"
#include "GET_SET.h"
#include "stm32f4xx_usart.h"
#include "encoder.h"
#include "gasvalvecontrol.h"
#include "movebase.h"
#include "wifi.h"

//////////////////Area of defining semaphore////////////////////////
 OS_EVENT 		*PeriodSem;

void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err; /* prevent warning... */
	
	/******************Create Semaphore***********************/
    PeriodSem				=	OSSemCreate(0);

    /******************Create Task**************************/	
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,					//Initial Task
	                      	(void          * ) 0,							
													(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],		
													(INT8U           ) Config_TASK_START_PRIO  );	
						
													
	os_err = OSTaskCreate(	(void (*)(void *)) WalkTask,					
	                      	(void          * ) 0,							
													(OS_STK        * )&WalkTaskStk[Walk_TASK_STK_SIZE-1],		
													(INT8U           ) Walk_TASK_PRIO  );													
}

/*
===============================================================
                        初始化任务
===============================================================
*/
void ConfigTask(void)
{	
	CPU_INT08U  os_err;	
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	
	//定时器初始化
	TIM_Init(TIM2, 999, 839, 0, 0);					//主周期定时10ms
	TIM_Init(TIM3, 99 , 839, 0, 0);
	TIM_Delayms(TIM5, 1500);	
	
	//串口初始化
	USART1_Init(115200);
	UART5_Init(115200);		//调试用蓝牙	
	USART3_Init(115200);	
	TIM_Delayms(TIM5, 10000);	
	
//	atk_8266_init();
	KeyInit();
	PhotoelectricityInit();
	
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);

    //电机初始化及使能
	elmo_Init();
	
	elmo_Enable(1);
	elmo_Enable(2);
	elmo_Enable(3);
	elmo_Enable(4);
//	elmo_Enable(5);
	elmo_Enable(6);
	elmo_Enable(7);
	elmo_Enable(8);
	elmo_Enable(9);
	elmo_Enable(10);
	elmo_Enable(11);
	elmo_Enable(12);	
	
	Vel_cfg(1, 100000, 100000);
	Vel_cfg(2, 100000, 100000);
	Vel_cfg(3, 100000, 100000);
	Vel_cfg(4, 100000, 100000);
	
	Pos_cfg(5, 300000, 300000, 12000);
	
	Pos_cfg(6,5000,5000,30000);//航向
	Pos_cfg(7,5000,5000,30000);//翻滚
	Pos_cfg(8,5000,5000,30000);//俯仰
	
	Pos_cfg(9,250000,250000,15000);//推盘(9,250000,250000,25000)
	
	Vel_cfg(10,800000,800000);	//后 发射 
	Vel_cfg(11,800000,800000);	//后 发射 
	
	TIM_Delayms(TIM5,50);

	while(1);

	ClampOpen();
	
	OSTaskSuspend(OS_PRIO_SELF);
}

/*
===============================================================
                        行走移动任务
===============================================================
*/
uint8_t launcherStatus = 0;
extern int32_t launcherPos;
static uint8_t launcherCounter = 0;
extern float speed;
extern float position[4];
float cl_angle(float ex, float act);	
static uint8_t lastXCounter;
static float lastX = 0;
static float presentX = 0;
static float robotSpeed = 0;
static uint8_t stopCounter = 0;

enum StatusMachine
{
	goToLoadingArea,
	load,
	goToLaunchingArea,
	launch
};

static uint8_t status = goToLoadingArea;

void WalkTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
    OSSemSet(PeriodSem, 0, &os_err);

	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		
		updatevel(GetPosX(), GetPosY(), GetAngle());
			
		switch (status)
		{
			//从出发区走向装载区
			case goToLoadingArea:
				if (GetPosX() > -400.0f)
				{
					MoveX(XSpeedUp(-0.0f, -400.0f, -1.5f));
				}
			    else if (GetPosX() <= -400 && GetPosX() > -12400.0f)
				{
					MoveX(-1.5f);
				}
				else if (GetPosX() <= -12400.0f)
				{
					MoveX(XSpeedDown(-12400.0f, -12800.0f, -1.5f));
					if (PHOTOSENSORLEFTFRONT && PHOTOSENSORRIGHTFRONT && GetPosX() <= -12800.0f)
					{
						MoveX(0.0f);
						status++;
					}
				}
				break;
				
			//装载飞盘
			case load:
				MoveX(0.0f);	
				ClampClose();
				if (KEYSWITCHLEFT && KEYSWITCHRIGHT)
				{
					status ++;
				}
				break;
			
            //从装载区走向发射区				
			case goToLaunchingArea:
				PosCrl(5, 0, -150000);
				
			    if (GetPosX() <= -12600.0f)
				{
					MoveX(XSpeedUp(-13040.0f, -12600.0f, 1.5f));
				}
			    else if (GetPosX() <= -7130.0f && GetPosX() > -12600.0f)
				{
					MoveX(1.5f);
				}
				else if (GetPosX() > -7130.0f && GetPosX() <= -6730.0f)
				{
					MoveX(XSpeedDown(-7130.0f, -6730.0f, 1.5f));
				}
				else if (GetPosX() > -6730)
				{
					MoveX(0.7f);
					if (GetPosX() > -6530)
					{
						MoveX(0.0f);
						
						if (GetAngle() < 1.0f && GetAngle() > -1.0f && stopCounter >= 100)
						{
							StopMove();
							stopCounter = 0;
							status++;
						}
						
						stopCounter++;
					}
				}
				break;
			
			//发射飞盘
			case launch:
				break;
			
			default:
				break;		
		}

        //蓝牙 or wifi调试输出		
//		ReadActualPos(6);
//		ReadActualPos(7);
//		ReadActualPos(8);
//		ReadActualPos(9);
//		ReadActualVel(9);
		
//		u5_printf("%d\r\n",(int)(1000*GetPosX()));
		
		lastXCounter++;
		if (lastXCounter == 10)
		{
			presentX = GetPosX();
			robotSpeed = (presentX - lastX) / 100.0f;
			USART_OUT(UART5, (const uint8_t *)"%d.%d   %d   %d  %d\r\n",
				      (int)robotSpeed, (int)((robotSpeed - (int)robotSpeed) * 1000),
				      (int)presentX, (int)GetPosY(), (int)GetAngle());
			lastX = presentX;
			lastXCounter = 0;
		}
		
//		u5_printf("Roll*10 = %d, Pitch*10 = %d, Yaw*10 = %d   LauncherVel = %d   EncVel0 = %d   EncVel1 = %d\r\n", 
//							(int)(position[1] * 10), (int)(position[2] * 10), (int)(position[0] * 10), 
//								speed, GetEncVel(0), GetEncVel(1));
		
//		//发射一秒后发射器退回
//		if (launcherStatus == 1)
//		{
//			launcherCounter++;
//			if (launcherCounter == 100)
//			{
//				launcherPos += 2048;
//				PosCrl(9, 0, launcherPos);
//				launcherStatus = 0;
//				launcherCounter = 0;
//			}
//		}

	} 
}	
