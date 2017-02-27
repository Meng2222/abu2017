#include <includes.h>
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "action_math.h"
#include "GET_SET.h"
#include "stm32f4xx_usart.h"
#include "gasvalvecontrol.h"
#include "movebase.h"
#include "wifi.h"

/*
===============================================================
                        信号量定义
===============================================================
*/
OS_EVENT *PeriodSem;

void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;          /*防止警告...*/
	
	/*创建信号量*/
    PeriodSem				=	OSSemCreate(0);

    /*创建任务*/	
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,				/*初始化任务*/
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
	TIM_Init(TIM2, 99, 839, 0, 0);   //1ms主定时器
	TIM_Delayms(TIM5, 1500);	
	
	//串口初始化
	UART4_Init(115200);     //蓝牙手柄
	UART5_Init(115200);		//调试用wifi	
	USART3_Init(115200);    //定位系统
//	TIM_Delayms(TIM5, 10000);	
	
	
	KeyInit();
	PhotoelectricityInit();
	BeepInit();
	
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

    //电机初始化及使能
//	elmo_Init();
//	
//	elmo_Enable(1);
//	elmo_Enable(2);
//	elmo_Enable(3);

	elmo_Enable(4);
	elmo_Enable(5);
	elmo_Enable(6);
	elmo_Enable(7);
	elmo_Enable(8);
	
//	Vel_cfg(1, 100000, 100000);
//	Vel_cfg(2, 100000, 100000);
//	Vel_cfg(3, 100000, 100000);

	Vel_cfg(4,300000,300000);	//
	Vel_cfg(5,300000,300000);	//

	Pos_cfg(6,5000,5000,30000);//俯仰
	Pos_cfg(7,5000,5000,30000);//翻滚
	Pos_cfg(8,5000,5000,30000);//航向
	
	TIM_Delayms(TIM5, 50);

	
//	atk_8266_init();
//	u5_printf("mv1  mv2  mv3    realmv1  realmv2  realmv3    x\r\n");
	
	ClampClose();
	LeftBack();
	RightBack();
	ClampReset();



	BEEP_ON;
	TIM_Delayms(TIM5, 1000);
	BEEP_OFF;

	OSTaskSuspend(OS_PRIO_SELF);
}

/*
===============================================================
                        行走移动任务
===============================================================
*/
extern float speed;
extern float position[4];
static uint16_t timeCounter = 0;
static uint16_t timeCounterL = 0;
static uint16_t timeCounterR = 0;
static uint16_t flagL = 0;
static uint16_t flagR = 0;
extern int shootFlag;
static int shootCounter = 0;
float amendX = 0.0f;
uint8_t amendXFlag = 0;
extern uint8_t moveTimFlag;
int expSpeed = 0;
int expSpeedp = 0;
int mv1 = 0, mv2 = 0, mv3 = 0;

typedef enum
{
	getReady,
	goToLoadingArea,
	stopRobot,
	load,
	goToLaunchingArea,
	launch
}Status_t;

Status_t status = getReady;

void WalkTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
    OSSemSet(PeriodSem, 0, &os_err);

	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		if(shootFlag ==1 )shootCounter++;
		if(shootCounter>=100)
		{
			GasValveControl(1,5,0);
			shootCounter =0;
			shootFlag=0;
		}



		switch (status)
		{
			//准备阶段
			case getReady:
				
//				if (PHOTOSENSORLEFTUP || PHOTOSENSORRIGHTUP)
//				{
//					status++;
//				}
				if (PHOTOSENSORRIGHTUP)
				{
					flagL = 1;
				}
				if (flagL == 1)
				{
					if (timeCounterL <= 80)
					{
						LeftPush();
					}
					else if (timeCounterL > 80)
					{
						LeftBack();
					}
					timeCounterL++;
					if (timeCounterL >= 200)
					{
						timeCounterL = 0;
						flagL = 0;
					}
				}
				
//				if (PHOTOSENSORRIGHTUP)
//				{
//					flagR = 1;
//				}
//				if (flagR == 1)
//				{
//					if (timeCounterR == 3)
//					{
//						RightPush();
//					}
//					if (timeCounterR == 60)
//					{
//						RightBack();
//					}
//					timeCounterR++;
//					if (timeCounterR >= 200)
//					{
//						timeCounterR = 0;
//						flagR = 0;
//					}
//					
//				}
			   
				break;
			//从出发区走向装载区
			case goToLoadingArea:

			    MoveTo(-13079.29f, -3000.0f, 1200.0f);

				if (GetPosX() <= -12900.0f && PHOTOSENSORRIGHT && PHOTOSENSORLEFT)
				{
					if (amendXFlag == 0)
					{
						amendX = -13079.29f - GetPosX();
						amendXFlag = 1;
					}
					moveTimFlag = 0;
					BEEP_ON;
					status++;					
				}

				break;
			
			//停车
			case stopRobot:
				MoveX(-ENDSPEED);
				if (GetPosX() <= -13146.58f)
				{
					BEEP_OFF;
					status++;
				}	
				break;
				
			//装载飞盘
			case load:
				LockWheel();
//                ClampClose();
				timeCounter++;
			    
			    if (timeCounter >= 100)
				{
//					ClampRotate();
					timeCounter = 0;
//					if (KEYSWITCH)
//					{
						status++;
//					}
				}
				break;
			
            //从装载区走向发射区				
			case goToLaunchingArea:
                MoveTo(/*-6573.29f*/0.0f, 3000.0f, 1200.0f);
			
//				if (flagL == 1)
//				{
//					if (timeCounterL == 3)
//					{
//						LeftPush();
//					}
//					if (timeCounterL == 60)
//					{
//						LeftBack();
//					}
//					timeCounterL++;
//					if (timeCounterL >= 200)
//					{
//						timeCounterL = 0;
//					}
//				}
//				if (flagR == 1)
//				{
//					if (timeCounterR == 3)
//					{
//						RightPush();
//					}
//					if (timeCounterR == 60)
//					{
//						RightBack();
//					}
//					timeCounterR++;
//					if (timeCounterR >= 200)
//					{
//						timeCounterR = 0;
//					}
//					
//				}
				
			    if (GetPosX() >= /*-6573.29f*/0.0f)
				{
					LockWheel();
					moveTimFlag = 0;
					status++;
				}
				break;
			
			//发射飞盘
			case launch:
				if (PHOTOSENSORLEFTUP || PHOTOSENSORRIGHTUP)
				{
					status = goToLoadingArea;
				}
//				if (flagL == 1)
//				{
//					if (timeCounterL == 3)
//					{
//						LeftPush();
//					}
//					if (timeCounterL == 60)
//					{
//						LeftBack();
//					}
//					timeCounterL++;
//					if (timeCounterL >= 200)
//					{
//						timeCounterL = 0;
//					}
//				}
//				if (flagR == 1)
//				{
//					if (timeCounterR == 3)
//					{
//						RightPush();
//					}
//					if (timeCounterR == 60)
//					{
//						RightBack();
//					}
//					timeCounterR++;
//					if (timeCounterR >= 200)
//					{
//						timeCounterR = 0;
//					}
//					
//				}
				break;
			
			default:
				break;		
		}
//		ReadActualVel(1);
//		ReadActualVel(2);
//		ReadActualVel(3);
//		
//        //蓝牙 or wifi调试输出
//		u5_printf("%d  %d  %d    %d  %d  %d    %d\r\n", mv1, mv2, mv3,
//            	  (int)GetMotorVel(1), (int)GetMotorVel(2), (int)GetMotorVel(3), (int)GetPosX());
	} 
}	
