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
#include "stm32f4xx_it.h"
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
extern uint8_t gCurrent[3];
extern uint8_t Speed[3];
extern uint8_t gTemperature[3];
extern SendVel sendVel1,sendVel2,sendVel3;
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
	uint8_t aaaa=33;
	int PowerResetFlag = 0 , LowResetFlag = 0;
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
	USART6_Init(115200);	//摄像头
//	TIM_Delayms(TIM5, 10000);	
	
	
	KeyInit();
	PhotoelectricityInit();
	BeepInit();
	
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

//    //电机初始化及使能
//	elmo_Init();
//	
//	elmo_Enable(1);
//	elmo_Enable(2);
//	elmo_Enable(3);

//	elmo_Enable(4);
//	elmo_Enable(5);
//	elmo_Enable(6);
//	elmo_Enable(7);
//	elmo_Enable(8);
//	elmo_Enable(9);
//	elmo_Enable(10);
//	elmo_Enable(11);
//	
//	Vel_cfg(1, 100000, 100000);
//	Vel_cfg(2, 100000, 100000);
//	Vel_cfg(3, 100000, 100000);

//	Vel_cfg(4,300000,300000);	//
//	Vel_cfg(5,300000,300000);	//

//	Pos_cfg(6,5000,5000,30000);//俯仰
//	Pos_cfg(7,5000,5000,30000);//翻滚
//	Pos_cfg(8,5000,5000,30000);//航向
//	
//	Vel_cfg(9,300000,300000);
//	Pos_cfg(10,5000,5000,30000);//航向
//	Pos_cfg(11,5000,5000,30000);//俯仰
	

//	TIM_Delayms(TIM5, 500);
//	GasValveControl(2,8,1);//上枪推弹
//	TIM_Delayms(TIM5, 500);
//	GasValveControl(2,8,0);//上枪
	GPIO_Init_Pins(GPIOC,GPIO_Pin_9,GPIO_Mode_OUT);
	TIM_Delayms(TIM5, 50);
	
	
//	atk_8266_init();
//	u5_printf("I1    I2    I3\r\n");
//	USART_SendData(UART5,gCurrent[0]);
//	USART_SendData(UART5,gCurrent[1]);
//	USART_SendData(UART5,gCurrent[2]);
//	USART_SendData(UART5,aaaa);
//	USART_SendData(UART5, 0x0d);	   
//	USART_SendData(UART5, 0x0a);	   

////	USART_OUT(UART5,"%s %s %s", gCurrent[0], gCurrent[1], gCurrent[2]);
////	u5_printf("%c    %c    %c\r\n", gCurrent[0], gCurrent[1], gCurrent[2]);
//	u5_printf("I1    I2    I3\r\n");
	ClampClose();
	LeftBack();
	RightBack();
	ClampReset();


	BEEP_ON;
	TIM_Delayms(TIM5, 1000);
	BEEP_OFF;
	PowerResetFlag = RCC_GetFlagStatus(RCC_FLAG_PORRST);
	LowResetFlag = RCC_GetFlagStatus(RCC_FLAG_BORRST);
//	if(PowerResetFlag||LowResetFlag==1)BEEP_ON;
	RCC_ClearFlag();

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
static uint16_t flagL = 1;
static uint16_t flagR = 0;
extern int shootFlagL , shootFlagR , shootFlagU;
static int shootCounterL = 0 , shootCounterR = 0 , shootCounterU = 0;
float amendX = 0.0f;
uint8_t amendXFlag = 0;
extern uint8_t moveTimFlag;
int expSpeed = 0;
int expSpeedp = 0;
int mv1 = 0, mv2 = 0, mv3 = 0;
static int loadFlag = 0 , sendFlag = 0; 
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
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		if(shootFlagL ==1 )shootCounterL++;
		if(shootCounterL>=100)
		{
			GasValveControl(1,5,0);
			shootCounterL =0;
			shootFlagL=0;
			loadFlag = 1;
		}
		if(shootFlagR ==1 )shootCounterR++;
		if(shootCounterR>=100)
		{
			shootCounterR =0;
			shootFlagR=0;
		}
		if(shootFlagU ==1 )shootCounterU++;
		if(shootCounterU>=100)
		{
			GasValveControl(2,8,0);
			shootCounterU =0;
			shootFlagU=0;
		}
		
//		if(loadFlag==1)
//		{
//			if (timeCounterL <= 150&&timeCounterL>=70)
//			{
//				LeftPush();
//			}
//			else if (timeCounterL > 150)
//			{
//				LeftBack();
//			}
//			timeCounterL++;
//			if (timeCounterL >= 200)
//			{
//				timeCounterL = 0;
//				loadFlag = 0;
//			}
//		}

		switch (status)
		{
			//准备阶段
			case getReady:				
//				if (PHOTOSENSORRIGHT)
//				{
//					ClampOpen();
//					status++;
//				}
//				VelCrl(1, 5000);
//				VelCrl(2, 5000);
//				VelCrl(3, 5000);
				if (PHOTOSENSORRIGHT)
				{
					flagL = 1;
				}
				if (flagL == 1)
				{
					if (timeCounterL <= 40)
					{
						LeftPush();
					}
					else if (timeCounterL > 40)
					{
						LeftBack();
					}
					timeCounterL++;
					if (timeCounterL >= 80)
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

			    MoveTo(-12776.96f, -2000.0f, 2400.0f);
				if (GetPosX() <= -12650.0f && PHOTOSENSORLEFTUP && PHOTOSENSORLEFT)
				{
					if (amendXFlag == 0)
					{
						amendX = -12776.96f - GetPosX();
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
				if (GetPosX() <= -13026.96f)
				{
					BEEP_OFF;
					status++;
				}	
				break;
				
			//装载飞盘
			case load:
				LockWheel();
                ClampClose();
				timeCounter++;			    
			    if (timeCounter >= 100)
				{
					ClampRotate();
					timeCounter = 0;
					if (KEYSWITCH)
					{
						status++;
					}
				}
				break;
			
            //从装载区走向发射区				
			case goToLaunchingArea:
                MoveTo(-6459.14f, 2000.0f, 1200.0f);
			
				if (flagL == 1)
				{
					if (timeCounterL == 3)
					{
						LeftPush();
					}
					if (timeCounterL == 60)
					{
						LeftBack();
					}
					timeCounterL++;
					if (timeCounterL >= 200)
					{
						timeCounterL = 0;
						flagL=0;
					}
				}
				if (flagR == 1)
				{
					if (timeCounterR == 3)
					{
						RightPush();
					}
					if (timeCounterR == 60)
					{
						RightBack();
					}
					timeCounterR++;
					if (timeCounterR >= 200)
					{
						timeCounterR = 0;
						flagR=0;
					}
					
				}
				
			    if (GetPosX() >= -6459.14f)
				{
					LockWheel();
					moveTimFlag = 0;
					status++;
				}
				break;
			
			//发射飞盘
			case launch:
				LockWheel();
//				if (PHOTOSENSORLEFTUP || PHOTOSENSORRIGHTUP)
//				{
//					status = goToLoadingArea;
//				}
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
//						flagL=0;
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
//				}
				break;
			
			default:
				break;		
		}
		
//		ReadActualVel(1);
//		ReadActualVel(2);
//		ReadActualVel(3);
//		ReadActualCurrent(1);
//		ReadActualCurrent(2);
//		ReadActualCurrent(3);
//		ReadActualTemperature(1);
//		ReadActualTemperature(2);
//		ReadActualTemperature(3);

//		if(GetPosX()<=-13200.0f)
//		{
//			while(1)
//			{
//				LockWheel();
//			}
//		}
//        //蓝牙 or wifi调试输出
		//电流单位为0.1A，在CAN接收中断中处理过
		if(status == getReady)
		{
			if(sendFlag==0)
			{
				u5_printf("%c %c %c %c%c %c%c %c%c %c %c %c\r\n", gCurrent[0], gCurrent[1], gCurrent[2],
																  sendVel1.vel[0],sendVel1.vel[1],sendVel2.vel[0],sendVel2.vel[1],sendVel3.vel[0],sendVel3.vel[1],
																  gTemperature[0],gTemperature[1],gTemperature[2]);
				sendFlag =1;
			}
		}
		else
		{
				u5_printf("%c %c %c %c%c %c%c %c%c %c %c %c\r\n", gCurrent[0], gCurrent[1], gCurrent[2], 
																  sendVel1.vel[0],sendVel1.vel[1],sendVel2.vel[0],sendVel2.vel[1],sendVel3.vel[0],sendVel3.vel[1],
																  gTemperature[0],gTemperature[1],gTemperature[2]);
		}
		//		u5_printf("%d    %d    %d\r\n", (int)GetPosX(),(int)GetPosY(),(int)GetAngle());
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	} 
}	
