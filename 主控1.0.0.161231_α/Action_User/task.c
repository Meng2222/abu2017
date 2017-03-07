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
//定义机器人全局变量
robot_t gRobot = {0};

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
	USART6_Init(115200);	//摄像头
	TIM_Delayms(TIM5, 10000);	
	
	//行程开关、光电、蜂鸣器初始化
	KeyInit();
	PhotoelectricityInit();
	BeepInit();
	
	//CAN1用于控制走行电机和枪上电机
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	//CAN2用于气阀板通信
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

    //电机初始化及使能
	elmo_Init();
	//底盘电机
	elmo_Enable(1);
	elmo_Enable(2);
	elmo_Enable(3);
	//左枪电机
	elmo_Enable(4);
	elmo_Enable(5);
	elmo_Enable(6);
	elmo_Enable(7);
	elmo_Enable(8);
	//上枪电机
	elmo_Enable(9);
	elmo_Enable(10);
	elmo_Enable(11);

	Vel_cfg(4,300000,300000);	//
	Vel_cfg(5,300000,300000);	//

	Pos_cfg(6,5000,5000,30000);//俯仰
	Pos_cfg(7,5000,5000,30000);//翻滚
	Pos_cfg(8,5000,5000,30000);//航向
	
	Vel_cfg(9,300000,300000);
	Pos_cfg(10,5000,5000,30000);//航向
	Pos_cfg(11,5000,5000,30000);//俯仰
	

	//测运行周期用IO口
	GPIO_Init_Pins(GPIOC,GPIO_Pin_9,GPIO_Mode_OUT);
	TIM_Delayms(TIM5, 50);
	
	//wifi初始化
	atk_8266_init();

	//气缸初始化动作
	ClampClose();
	LeftBack();
	RightBack();
	ClampReset();


	BEEP_ON;
	TIM_Delayms(TIM5, 1000);
	BEEP_OFF;
	//读取寄存器复位标志，没有使用，暂时屏掉
//	PowerResetFlag = RCC_GetFlagStatus(RCC_FLAG_PORRST);
//	LowResetFlag = RCC_GetFlagStatus(RCC_FLAG_BORRST);
//	NRSTResetFlag = RCC_GetFlagStatus(RCC_FLAG_PINRST);
//	RCC_ClearFlag();

	OSTaskSuspend(OS_PRIO_SELF);
}

/*
===============================================================
                        行走移动任务
===============================================================
*/

//射标志位
extern int shootFlagL , shootFlagR , shootFlagU;
//坐标修正量及修正标志位
float amendX = 0.0f;
uint8_t amendXFlag = 0;
//走行移动计时标志位
extern uint8_t moveTimFlag;
//送弹标志位
static int loadFlag = 1; 
//发射参数
extern shootCtr_t shootParam[15];
//状态变量
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

//调试数据发送不能超过30个字节，发送10个字节需要1ms
void sendDebugInfo(void) 
{  
#define POS_X_OFFSET 50
	USART_SendData(UART5, (int8_t)gRobot.actualSpeed.v1); 	
	USART_SendData(UART5, (int8_t)gRobot.actualSpeed.v2); 	
	USART_SendData(UART5, (int8_t)gRobot.actualSpeed.v3); 	
	
	USART_SendData(UART5, (int8_t)gRobot.acturalCurrent.current1); 	
	USART_SendData(UART5, (int8_t)gRobot.acturalCurrent.current2); 	
	USART_SendData(UART5, (int8_t)gRobot.acturalCurrent.current3); 	
	
	USART_SendData(UART5, (uint8_t)gRobot.driverTemperature.temerature1); 	
	USART_SendData(UART5, (uint8_t)gRobot.driverTemperature.temerature2); 	
	USART_SendData(UART5, (uint8_t)gRobot.driverTemperature.temerature3); 
	
	//角度范围【-180，180】，但是实际走行中角度值基本在0度附近，fix me
	USART_SendData(UART5, (int8_t)gRobot.actualAngle); 
	
	//X位移分米部分范围是【-140，10】，单位分米
	USART_SendData(UART5, (int8_t)(gRobot.actualXPos/100.0f+ POS_X_OFFSET)); 
	//X位移厘米部分范围是【-100，100】，单位厘米
	USART_SendData(UART5, (uint8_t)((((int)gRobot.actualXPos))%100/10)); 
	
	//根据场地约束，范围设计为【-130，130】，单位cm
	USART_SendData(UART5, (int8_t)(gRobot.actualYPos/10.0f)); 
	 
	//连续发送4个-100作为结束标识符
	USART_SendData(UART5, (uint8_t)-100); 	
	USART_SendData(UART5, (uint8_t)-100); 
	USART_SendData(UART5, (uint8_t)-100); 	
	USART_SendData(UART5, (uint8_t)-100); 
}

void WalkTask(void)
{
	//上弹计时变量
	static uint16_t timeCounter = 0;
	static uint16_t timeCounterL = 0;
	static uint16_t timeCounterR = 0;
	//上弹标志位
	static uint16_t flagL = 1;
	static uint16_t flagR = 0;
	//射计时变量
	static int shootCounterL = 0 ;
	int shootNum = 0;
	int shootTimeGap = 0;
	CPU_INT08U  os_err;
	os_err = os_err;
    OSSemSet(PeriodSem, 0, &os_err);

	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
//静止测功能使使用暂时屏掉
//		if(shootFlagL ==1 )shootCounterL++;
//		if(shootCounterL>=100)
//		{
//			GasValveControl(1,5,0);
//			shootCounterL =0;
//			shootFlagL=0;
//			loadFlag = 1;
//		}
//		if(shootFlagR ==1 )shootCounterR++;
//		if(shootCounterR>=100)
//		{
//			shootCounterR =0;
//			shootFlagR=0;
//		}
//		if(shootFlagU ==1 )shootCounterU++;
//		if(shootCounterU>=100)
//		{
//			GasValveControl(2,8,0);
//			shootCounterU =0;
//			shootFlagU=0;
//		}
		
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
				if (PHOTOSENSORRIGHT)
				{
					ClampOpen();
					status++;
				}
//静止测功能时使用，暂时屏掉
//				if (PHOTOSENSORRIGHT)
//				{
//					flagL = 1;
//				}
//				if (flagL == 1)
//				{
//					if (timeCounterL <= 80)
//					{
//						LeftPush();
//					}
//					else if (timeCounterL > 80)
//					{
//						LeftBack();
//					}
//					timeCounterL++;
//					if (timeCounterL >= 160)
//					{
//						timeCounterL = 0;
//						flagL = 0;
//					}
//				}
				
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
					status++;

					if (KEYSWITCH)
					{
						status++;
					}
				}
				break;
			
            //从装载区走向发射区				
			case goToLaunchingArea:
                MoveTo(-6459.14f, 2000.0f, 1200.0f);
			//装载区到发射区过程发射两个飞盘，暂时屏掉			
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
//						flagR=0;
//					}
//					
//				}
				
			    if (GetPosX() >= -6459.14f)
				{
					LockWheel();
					moveTimFlag = 0;
					loadFlag = 1;
					status++;
				}
				break;
			
			//发射飞盘
			case launch:
				LockWheel();
			//发射用，暂时屏掉
//				if(shootNum>14)shootNum-=15;
//				ShootCtr(&shootParam[shootNum]);
//				shootTimeGap++;
//				if(loadFlag==1)
//				{
//					if (timeCounterL <= 150&&timeCounterL>=70)
//					{
//						LeftPush();
//					}
//					else if (timeCounterL > 150)
//					{
//						LeftBack();
//					}
//					timeCounterL++;
//					if (timeCounterL >= 200)
//					{
//						timeCounterL = 0;
//						loadFlag = 0;
//					}
//				}
//				if(shootTimeGap>=150)shootFlagL=1;
//				if(shootFlagL ==1 )
//				{
//					GasValveControl(1,5,1);
//					shootCounterL++;
//				}
//				if(shootCounterL>=100)
//				{
//					GasValveControl(1,5,0);
//					shootNum++;
//					shootCounterL =0;
//					shootTimeGap=0;
//					shootFlagL=0;
//					loadFlag = 1;
//				}
				break;
			
			default:
				break;		
		}
		//读取电机速度
		ReadActualVel(1);
		ReadActualVel(2);
		ReadActualVel(3);
		//读取电机电流
		ReadActualCurrent(1);
		ReadActualCurrent(2);
		ReadActualCurrent(3);
		//读取电机温度
		ReadActualTemperature(1);
		ReadActualTemperature(2);
		ReadActualTemperature(3);
		sendDebugInfo();

		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	} 
}	
