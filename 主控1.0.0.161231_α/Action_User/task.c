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
	TIM_Init(TIM2, 99, 839, 0, 0);   //1ms
	TIM_Delayms(TIM5, 1500);	
	
	//串口初始化
//	UART4_Init(115200);
//	USART1_Init(115200);
	UART5_Init(115200);		//调试用蓝牙	
	USART3_Init(115200);    //陀螺仪	
	TIM_Delayms(TIM5, 10000);	
	
//	atk_8266_init();
	KeyInit();
	PhotoelectricityInit();
	BeepInit();
	
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

    //电机初始化及使能
	elmo_Init();
	
	elmo_Enable(1);
	elmo_Enable(2);
	elmo_Enable(3);

//	elmo_Enable(4);
//	elmo_Enable(5);
//	elmo_Enable(6);
//	elmo_Enable(7);
//	elmo_Enable(8);
//	elmo_Enable(9);
//	elmo_Enable(10);
//	elmo_Enable(11);
	
	Vel_cfg(1, 100000, 100000);
	Vel_cfg(2, 100000, 100000);
	Vel_cfg(3, 100000, 100000);

//	Pos_cfg(6,5000,5000,30000);//俯仰
//	Pos_cfg(7,5000,5000,30000);//翻滚
//	Pos_cfg(8,5000,5000,30000);//航向
//	
//	Pos_cfg(9,250000,250000,10000);//推盘(9,250000,250000,25000)
//	
//	Vel_cfg(4,300000,300000);	//
//	Vel_cfg(5,300000,300000);	//
	
	TIM_Delayms(TIM5, 50);

//	ClampOpen();
//	LeftBack();
//	RightBack();
//	ClampReset();

//	u5_printf("expSpeed  expSpeedp  realSpeed  rate angle\r\n");
	
	BEEP_ON;
	TIM_Delayms(TIM5, 100);
	BEEP_OFF;
	
	
	OSTaskSuspend(OS_PRIO_SELF);
}

/*
===============================================================
                        行走移动任务
===============================================================
*/
uint8_t launcherStatus = 0;
extern int32_t launcherPos;
extern float speed;
extern float position[4];
float cl_angle(float ex, float act);
static uint16_t timeCounter = 0;
static uint16_t timeCounterL = 0;
static uint16_t timeCounterR = 0;
static uint16_t flagL = 1;
static uint16_t flagR = 1;
float amendX = 0.0f;
uint8_t amendXFlag = 0;
float tempX = 0.0f;
extern uint8_t moveTimFlag;
int expSpeed = 0;
int expSpeedp = 0;

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
	
		
		switch (status)
		{
			//准备阶段
			case getReady:
				
			   if (PHOTOSENSORLEFTUP || PHOTOSENSORRIGHTUP)
			   {
				   status++;
			   }
//				if (PHOTOSENSORLEFTUP && !PHOTOSENSORRIGHTUP)
//				{
//					ClampOpen();
//				}
//				else if (!PHOTOSENSORLEFTUP && PHOTOSENSORRIGHTUP)
//				{
//					ClampClose();
//				}
			   
				break;
			//从出发区走向装载区
			case goToLoadingArea:

			    Move(-2000.0f, tempX, -13079.29f, 1000.0f);

				if (GetPosX() <= -12900.0f && PHOTOSENSORRIGHT && PHOTOSENSORLEFT)
				{
					if (amendXFlag == 0)
					{
						amendX = -13079.29f - GetPosX();
						amendXFlag = 1;
					}
					BEEP_ON;
					status++;					
				}

				break;
			
			//停车
			case stopRobot:
				MoveX(-ENDSPEED);
				if (GetPosX() <= -13146.58f)
				{
					tempX = GetPosX();
					moveTimFlag = 0;
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
					if (KEYSWITCH)
					{
						status++;
					}
				}
				break;
			
            //从装载区走向发射区				
			case goToLaunchingArea:
                Move(2000.0f, tempX, -6573.29f, 1000.0f);
//			
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
				
			    if (GetPosX() >= -6573.29f)
				{
					LockWheel();
					tempX = GetPosX();
					moveTimFlag = 0;
					status++;
				}
				break;
			
			//发射飞盘
			case launch:
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
//		u5_printf("%d  %d  %d  %d  %d  %d\r\n", expSpeed, expSpeedp, (int)GetVel(), 
//			      (int)GetMotorVelRate(), (int)GetMotorVelAng());
	} 
}	
