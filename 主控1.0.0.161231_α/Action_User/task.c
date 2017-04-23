#include <includes.h>
#include <app_cfg.h>
#include "robot.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "gasvalvecontrol.h"
#include "movebase.h"
#include "database.h"
#include "flash.h"
#include "movebase2.h"
//#define RED_FIELD
#define BLUE_FIELD
#define NO_WALK_TASK
/*
===============================================================
                        信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
OS_EVENT *DebugPeriodSem;
//定义互斥型信号量用于管理CAN发送资源
OS_EVENT *CANSendMutex;
//邮箱定义
OS_EVENT *OpenSaftyMbox;
OS_EVENT *LeftGunShootPointMbox;
OS_EVENT *RightGunShootPointMbox;
OS_EVENT *LeftGunNextPointMbox;
OS_EVENT *RightGunNextPointMbox;

//定义机器人全局变量
extern robot_t gRobot;
extern uint8_t LeftGunPriority[7];
extern uint8_t RightGunPriority[7];

//状态变量
typedef enum
{
	getReady,
	goToLoadingArea,
	stopRobot,
	load,
	beginToGo1,
	goToHalfLaunchingArea,
	beginToGo2,
	goTo3QuarterArea,
	beginToGo3,
	goToLaunchingArea,
	launch
}Status_t;

Status_t status = getReady;

static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];
static  OS_STK  WalkTaskStk[Walk_TASK_STK_SIZE];
static  OS_STK  LeftGunShootTaskStk[LEFT_GUN_AUTO_SHOOT_STK_SIZE];
static  OS_STK  RightGunShootTaskStk[RIGHT_GUN_SHOOT_STK_SIZE];
static  OS_STK  UpperGunShootTaskStk[UPPER_GUN_SHOOT_STK_SIZE];
static 	OS_STK  DebugTaskStk[DEBUG_TASK_STK_SIZE];

void LeftGunShootTask(void);
void RightGunShootTask(void);
void UpperGunShootTask(void);

//调试数据发送不能超过30个字节，发送10个字节需要1ms
void sendDebugInfo(void)
{
#ifdef RED_FIELD
#define POS_X_OFFSET (50)
#endif
#ifdef BLUE_FIELD
#define POS_X_OFFSET (-50)
#endif
	USART_SendData(UART5,(int8_t)status);
	
	USART_SendData(UART5, (int8_t)gRobot.moveBase.targetSpeed.leftWheelSpeed);
	USART_SendData(UART5, (int8_t)gRobot.moveBase.targetSpeed.forwardWheelSpeed);
	USART_SendData(UART5, (int8_t)gRobot.moveBase.targetSpeed.backwardWheelSpeed);
	
	USART_SendData(UART5, (int8_t)gRobot.moveBase.actualSpeed.leftWheelSpeed);
	USART_SendData(UART5, (int8_t)gRobot.moveBase.actualSpeed.forwardWheelSpeed);
	USART_SendData(UART5, (int8_t)gRobot.moveBase.actualSpeed.backwardWheelSpeed);
	
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[0]);
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[1]);
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[2]);
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[3]);

//	USART_SendData(UART5, (int8_t)gRobot.moveBase.acturalCurrent.leftWheelCurrent);
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.acturalCurrent.forwardWheelCurrent);
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.acturalCurrent.backwardWheelCurrent);

//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverTemperature.leftWheelDriverTemperature);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverTemperature.forwardWheelDrvierTemperature);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverTemperature.backwardWheelDriverTemperature);

//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCurrentLimitFlag.leftWheelDriverFlag);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCurrentLimitFlag.forwardWheelDriverFlag);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCurrentLimitFlag.backwardWheelDriverFlag);
	
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverVelocityError.leftMotorVelocityError);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverVelocityError.forwardMotorVelocityError);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverVelocityError.backwardMotorVelocityError);

//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverUnitMode.leftDriverUnitMode);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverUnitMode.forwardDriverUnitMode);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverUnitMode.backwardDriverUnitMode);
//	
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCommandVelocity.leftDriverCommandVelocity);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCommandVelocity.forwardDriverCommandVelocity);
//	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCommandVelocity.backwardDriverCommandVelocity);
//	
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverJoggingVelocity.leftDriverJoggingVelocity);
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverJoggingVelocity.forwardDriverJoggingVelocity);
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverJoggingVelocity.backwardDriverJoggingVelocity);
	
	//角度范围【-180，180】，但是实际走行中角度值基本在0度附近，fix me
	USART_SendData(UART5, (int8_t)gRobot.moveBase.actualAngle);
	//X位移分米部分范围是【-140，10】，单位分米
	USART_SendData(UART5, (uint8_t)(gRobot.moveBase.actualXPos/100.0f+ POS_X_OFFSET));
	//X位移厘米部分范围是【-100，100】，单位厘米
	USART_SendData(UART5, (uint8_t)((((int)gRobot.moveBase.actualXPos))%100/10));

	//根据场地约束，范围设计为【-130，130】，单位cm
	USART_SendData(UART5, (int8_t)(gRobot.moveBase.actualYPos/10.0f));

	USART_SendData(UART5,(uint8_t)(gRobot.moveBase.actualKenimaticInfo.vt*0.01f));
	//连续发送4个-100作为结束标识符
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
}
void LeftGunSendDebugInfo(void)
{
	USART_SendData(UART5,1);
	
	USART_SendData(UART5,gRobot.leftGun.checkTimeUsage);
	
	USART_SendData(UART5,	gRobot.leftGun.targetPlant);
	
	USART_SendData(UART5, gRobot.leftGun.shootParaMode);

	USART_SendData(UART5,(int8_t)gRobot.leftGun.targetPose.yaw);
	USART_SendData(UART5,(int8_t)gRobot.leftGun.actualPose.yaw);
	USART_SendData(UART5,(int8_t)((gRobot.leftGun.actualPose.yaw-(int8_t)gRobot.leftGun.actualPose.yaw)*10));
	USART_SendData(UART5,(int8_t)gRobot.leftGun.targetPose.pitch);
	USART_SendData(UART5,(int8_t)gRobot.leftGun.actualPose.pitch);
	USART_SendData(UART5,(int8_t)((gRobot.leftGun.actualPose.pitch-(int8_t)gRobot.leftGun.actualPose.pitch)*10));
	USART_SendData(UART5,(int8_t)gRobot.leftGun.targetPose.roll);
	USART_SendData(UART5,(int8_t)gRobot.leftGun.actualPose.roll);
	USART_SendData(UART5,(int8_t)((gRobot.leftGun.actualPose.roll-(int8_t)gRobot.leftGun.actualPose.roll)*10));
	USART_SendData(UART5,(int8_t)gRobot.leftGun.targetPose.speed1);
	USART_SendData(UART5,(int8_t)gRobot.leftGun.actualPose.speed1);
	USART_SendData(UART5,(int8_t)gRobot.leftGun.targetPose.speed2);
	USART_SendData(UART5,(int8_t)gRobot.leftGun.actualPose.speed2);
	
	
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	
}
void RightGunSendDebugInfo(void)
{
	USART_SendData(UART5,2);
	
	USART_SendData(UART5,gRobot.rightGun.checkTimeUsage);
	
	USART_SendData(UART5,	gRobot.rightGun.targetPlant);
	
	USART_SendData(UART5, gRobot.rightGun.shootParaMode);
	
	USART_SendData(UART5,(int8_t)gRobot.rightGun.targetPose.yaw);
	USART_SendData(UART5,(int8_t)gRobot.rightGun.actualPose.yaw);
	USART_SendData(UART5,(int8_t)((gRobot.rightGun.actualPose.yaw-(int8_t)gRobot.rightGun.actualPose.yaw)*10));
	USART_SendData(UART5,(int8_t)gRobot.rightGun.targetPose.pitch);
	USART_SendData(UART5,(int8_t)gRobot.rightGun.actualPose.pitch);
	USART_SendData(UART5,(int8_t)((gRobot.rightGun.actualPose.pitch-(int8_t)gRobot.rightGun.actualPose.pitch)*10));
	USART_SendData(UART5,(int8_t)gRobot.rightGun.targetPose.roll);
	USART_SendData(UART5,(int8_t)gRobot.rightGun.actualPose.roll);
	USART_SendData(UART5,(int8_t)((gRobot.rightGun.actualPose.roll-(int8_t)gRobot.rightGun.actualPose.roll)*10));
	USART_SendData(UART5,(int8_t)gRobot.rightGun.targetPose.speed1);
	USART_SendData(UART5,(int8_t)gRobot.rightGun.actualPose.speed1);
	USART_SendData(UART5,(int8_t)gRobot.rightGun.targetPose.speed2);
	USART_SendData(UART5,(int8_t)gRobot.rightGun.actualPose.speed2);
	
	
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
}
void UpperGunSendDebugInfo(void)
{
	USART_SendData(UART5,3);
	
	USART_SendData(UART5,	gRobot.upperGun.targetPlant);
	
	USART_SendData(UART5, gRobot.upperGun.shootParaMode);
	
	USART_SendData(UART5,(int8_t)gRobot.upperGun.targetZone);
		
	USART_SendData(UART5,(int8_t)gRobot.upperGun.targetPose.yaw);
	USART_SendData(UART5,(int8_t)gRobot.upperGun.targetPose.pitch);
	USART_SendData(UART5,(int8_t)gRobot.upperGun.targetPose.speed1);
	
	USART_SendData(UART5,(int8_t)gRobot.upperGun.actualPose.yaw);
	USART_SendData(UART5,(int8_t)gRobot.upperGun.actualPose.pitch);
	USART_SendData(UART5,(int8_t)gRobot.upperGun.actualPose.speed1);

	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
}
//摄像头初始化
void CameraInit(void)
{
	USART_SendData(USART3, 'a');
	USART_SendData(USART3, 'a');
#ifdef BLUE_FIELD
	USART_SendData(USART3, 'b');
#endif
#ifdef RED_FIELD
	USART_SendData(USART3, 'r');
#endif
}
//到达场地中央后通知摄像头开始发坐标
void SendStop2Camera(void)
{
	USART_SendData(USART3, 'c');
}
//坐标调整到位后通知摄像头开始工作
void SendStartWork2Camera(void)
{
	USART_SendData(USART3, 'd');
}
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;          /*防止警告...*/

	/*创建信号量*/
    PeriodSem				=	OSSemCreate(0);
	DebugPeriodSem          =   OSSemCreate(0);
	
	//创建互斥型信号量
	CANSendMutex            =   OSMutexCreate(9,&os_err);

	//创建邮箱
	OpenSaftyMbox            =   OSMboxCreate((void *)0);
	LeftGunShootPointMbox    =   OSMboxCreate((void *)0);
	RightGunShootPointMbox   =   OSMboxCreate((void *)0);
	LeftGunNextPointMbox     =   OSMboxCreate((void *)0);
	RightGunNextPointMbox    =   OSMboxCreate((void *)0);
    /*创建任务*/
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,				/*初始化任务*/
	                      	(void          * ) 0,
													(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],
													(INT8U           ) Config_TASK_START_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) WalkTask,
	                      	(void          * ) 0,
													(OS_STK        * )&WalkTaskStk[Walk_TASK_STK_SIZE-1],
													(INT8U           ) Walk_TASK_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) LeftGunShootTask,
	                      	(void          * ) 0,
													(OS_STK        * )&LeftGunShootTaskStk[LEFT_GUN_AUTO_SHOOT_STK_SIZE-1],
													(INT8U           ) LEFT_GUN_SHOOT_TASK_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) RightGunShootTask,
							(void          * ) 0,
													(OS_STK        * )&RightGunShootTaskStk[RIGHT_GUN_SHOOT_STK_SIZE-1],
													(INT8U           ) RIGHT_GUN_SHOOT_TASK_PRIO);
	os_err = OSTaskCreate(	(void (*)(void *)) UpperGunShootTask,
							(void          * ) 0,
													(OS_STK        * )&UpperGunShootTaskStk[UPPER_GUN_SHOOT_STK_SIZE-1],
													(INT8U           ) UPPER_GUN_SHOOT_TASK_PRIO);
	os_err = OSTaskCreate(	(void (*)(void *)) DebugTask,
							(void          * ) 0,
													(OS_STK        * )&DebugTaskStk[DEBUG_TASK_STK_SIZE-1],
													(INT8U           ) DEBUG_TASK_PRIO);

}

/*
===============================================================
                        初始化任务
===============================================================
*/
uint32_t canErrCode = 0;
void ConfigTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	
	//************************
	USART3_Init(115200);    //摄像头
	CameraInit();
	//***********************
	
	//定时器初始化
	TIM_Init(TIM2, 99, 839, 0, 0);   //1ms主定时器
	TIM_Delayms(TIM5, 1500);
	
	KeyInit();
	PhotoelectricityInit();
	BeepInit();
	
	//串口初始化
	UART4_Init(115200);     //蓝牙手柄
	UART5_Init(115200);		//调试用wifi
	
	//******************
//	USART3_Init(115200);    //摄像头
	//*******************
	USART6_Init(115200);	//定位系统
//	FlashInit();
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

//	TIM_Delayms(TIM5, 10000);
	GPIO_Init_Pins(GPIOC,GPIO_Pin_9,GPIO_Mode_OUT);
	TIM_Delayms(TIM5, 50);

	ROBOT_Init();

	ClampClose();
	LeftBack();
	RightBack();
	ClampReset();

	BEEP_ON;
	TIM_Delayms(TIM5, 1000);
	BEEP_OFF;

//	LeftHold();
//	RightHold();
//////测试使用！！！！！！！！！！！！！！！！
//	MoveY(50.0f);
#ifdef NO_WALK_TASK
	while(!PHOTOSENSORUPGUN)
	{
		//WAIT
	}
	MoveY(50);
	OSTaskSuspend(Walk_TASK_PRIO);
#endif

//	OSTaskSuspend(LEFT_GUN_SHOOT_TASK_PRIO);
//	OSTaskSuspend(RIGHT_GUN_SHOOT_TASK_PRIO);
#ifndef NO_WALK_TASK
	OSTaskSuspend(UPPER_GUN_SHOOT_TASK_PRIO);
#endif
	OSTaskSuspend(DEBUG_TASK_PRIO);

	OSTaskSuspend(OS_PRIO_SELF);
}


//坐标修正量及修正标志位
float amendX = 0.0f;
uint8_t amendXFlag = 0;
//走行移动计时标志位
uint8_t moveTimFlag = 0;


void WalkTask(void)
{
	static uint16_t timeCounter = 0;
	CPU_INT08U  os_err;
	os_err = os_err;
	int shootPointMsg = MOVEBASE_POS_READY;
    OSSemSet(PeriodSem, 0, &os_err);
	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		if(status >= 3)
		{
			ROBOT_CheckGunOpenSafety();
		}
		ReadActualVel(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadActualCurrent(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadActualTemperature(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadCurrentLimitFlag(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadVelocityError(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadCommandVelocity(CAN2, MOVEBASE_BROADCAST_ID);
		ReadJoggingVelocity(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadMotorFailure(CAN2,MOVEBASE_BROADCAST_ID);

		UpdateKenimaticInfo();	
		sendDebugInfo();
		switch (status)
		{
			//准备阶段
			case getReady:				
				if(PHOTOSENSORUPGUN)
				{
					ClampOpen();
					ROBOT_LeftGunAim();
					ROBOT_RightGunAim();
					status ++;
				}
				break;
			//从出发区走向装载区
			case goToLoadingArea:
//				MoveToCenter(-13023.14f, -3200.0f, 2000.0f);
#ifdef RED_FIELD
				MoveToCenter(-13023.14f, -3500.0f, 2000.0f);
				if (GetPosX() <= -12650.0f && PHOTOSENSORRIGHT)
				{
					if (amendXFlag == 0)
					{
						amendX = -12776.96f - GetPosX();
						amendXFlag = 1;
					}
					BEEP_ON;
//					status++;					
				}
				if(GetPosX()<=-13023.14f)
				{
					moveTimFlag = 0;
					status = load;
					BEEP_OFF;
				}
#endif				
#ifdef BLUE_FIELD
				MoveToCenter(13023.14f, 3000.0f, 2000.0f);		
				if (GetPosX() >= 12650.0f && PHOTOSENSORLEFT)
				{
					if (amendXFlag == 0)
					{
						amendX = 12776.96f - GetPosX();
						amendXFlag = 1;
					}
					BEEP_ON;
				}
				if(GetPosX()>=13023.14f)
				{
					moveTimFlag = 0;
					status = load;
					BEEP_OFF;
				}				
#endif

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
			    if (timeCounter >= 28)
				{
					timeCounter = 0;
					ClampRotate();
					status ++;
				}
				break;
			
			case beginToGo1:
				if (PHOTOSENSORUPGUN)
				{
//					status++;
//					OSTaskResume(DEBUG_TASK_PRIO);
					ROBOT_UpperGunAim();	
					status+=5;
				}
				break;
				
			//四分之三位置停车
			case goToHalfLaunchingArea:
			{
				int backCnt = 0u;
				if(backCnt < 50u)
				{
					backCnt ++;
				}
				else if(backCnt == 50u)
				{
					LeftBack();
					RightBack();
				}
				MoveToCenter(-9459.14f, 2000.0f, 2000.0f);
			    if (GetPosX() >= -9459.14f)
				{
					LockWheel();
					MoveY(50.0f);
					moveTimFlag = 0;
					status++;
					OSMboxPostOpt(LeftGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
					OSMboxPostOpt(RightGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
				}
				break;
			}
			case beginToGo2:
				if (PHOTOSENSORUPGUN)
				{
					OSMboxPend(LeftGunNextPointMbox, 0, &os_err);
					OSMboxPend(RightGunNextPointMbox, 0, &os_err);
					OSSemSet(PeriodSem, 0, &os_err);
//					status++;
					status+=3;
				}
				break;
			case goTo3QuarterArea:
				MoveTo(-2925.14f, 1500.0f, 1200.0f);
			    if (GetPosX() >= -2925.14f)
				{
					LockWheel();
					moveTimFlag = 0;
					status++;
					OSTaskSuspend(OS_PRIO_SELF);
				}
				break;
			
			case beginToGo3:
				if (PHOTOSENSORUPGUN)
				{
					status++;
				}
				break;	
            //从装载区走向发射区				
			case goToLaunchingArea:
#ifdef RED_FIELD
                MoveToCenter(-6459.14f, 3000.0f, 2000.0f);
			    if (GetPosX() >= -6459.14f)
				{
					ClampReset();
					LockWheel();
					moveTimFlag = 0;
					OSMboxPostOpt(LeftGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
					OSMboxPostOpt(RightGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
					status++;
				}
#endif
#ifdef BLUE_FIELD
                MoveToCenter(6459.14f, -3000.0f, 2000.0f);
			    if (GetPosX() <= 6459.14f)
				{
					ClampReset();
					LockWheel();
					moveTimFlag = 0;
					OSMboxPostOpt(LeftGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
					OSMboxPostOpt(RightGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
					status++;
				}
#endif
				break;
			
			//发射飞盘
			case launch:
				LockWheel();
//				CameraInit();
				MoveY(50.0f);
				SendStop2Camera();
				OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
				OSTaskSuspend(OS_PRIO_SELF);
				break;
			
			default:
				break;		
		}

		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	}
}

void LeftGunShootTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
#ifndef NO_WALK_TASK
	OSMboxPend(OpenSaftyMbox, 0, &os_err);
#endif
//	LeftBack();
	OSTimeDly(50);
	LeftPush();
	gRobot.leftGun.mode = GUN_AUTO_MODE;
	//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
	while(1)
	{
		if(gRobot.leftGun.modeChangeFlag == 1)
		{
			LeftBack();
			gRobot.leftGun.modeChangeFlag = 0;
		}
		//自动模式
		if(ROBOT_GunCheckMode(LEFT_GUN) == GUN_AUTO_MODE)
		{

//			if(gRobot.leftGun.shootTimes >=   ROBOT_LeftGunPoint1ShootTimes() + ROBOT_LeftGunPoint3ShootTimes())
//			{
				//自动调度
			//

				shoot_command_t leftGunShootCommand = ROBOT_LeftGunGetShootCommand();

				if(gRobot.leftGun.commandState == GUN_HAVE_COMMAND)
				{
					gRobot.leftGun.targetPlant = leftGunShootCommand.plantNum;
					gRobot.leftGun.nextStep = 2;
					gRobot.leftGun.shootParaMode = leftGunShootCommand.shootMethod;
					
					if(leftGunShootCommand.plantNum == PLANT7 || leftGunShootCommand.plantNum == PLANT3)
					{
						//获取并更新枪目标姿态  上弹姿态
						gRobot.leftGun.targetPose = gLeftGunReloadPosDatabase[leftGunShootCommand.shootPoint]\
																			[leftGunShootCommand.shootMethod]\
																			[leftGunShootCommand.plantNum];

						ROBOT_LeftGunAim();
						ROBOT_LeftGunCheckAim();
					}
					else
					{
						gRobot.leftGun.targetPose = gLeftGunPosDatabase[leftGunShootCommand.shootPoint]\
																				[leftGunShootCommand.shootMethod]\
																				[leftGunShootCommand.plantNum];
						ROBOT_LeftGunAim();
					}
					ROBOT_LeftGunReload();				
					
					//检查上弹是否到位
//					ROBOT_LeftGunCheckReload();
					//获取并更新枪目标姿态  发射姿态
					gRobot.leftGun.targetPose = gLeftGunPosDatabase[leftGunShootCommand.shootPoint]\
																		[leftGunShootCommand.shootMethod]\
																		[leftGunShootCommand.plantNum];

					ROBOT_LeftGunAim();
					if(leftGunShootCommand.plantNum == PLANT6)		//PLANT6不检查姿态
					{
						ROBOT_LeftGunCheckAim();
//						OSTimeDly(300);
//						gRobot.leftGun.ready = GUN_AIM_DONE;
					}
					else
					{
//							OSTimeDly(150);
							ROBOT_LeftGunCheckAim();
//							gRobot.leftGun.ready = GUN_AIM_DONE;					
					}
//				OSTimeDly(75);
//				if(leftGunShootCommand.plantNum==PLANT6)
//					gRobot.leftGun.ready = GUN_AIM_DONE;
//					OSTimeDly(250);	
								
//				//再次检查该柱子的状态，确定是否发射				
//				if((leftGunShootCommand.shootMethod == SHOOT_METHOD1)&&(gRobot.plantState[leftGunShootCommand.plantNum].ball == 1))
//					gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;
//				if((leftGunShootCommand.shootMethod == SHOOT_METHOD2)&&(gRobot.plantState[leftGunShootCommand.plantNum].plate == 1))
//					gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;
				
				//fix me 此处应当再次检查命令
#ifndef NO_WALK_TASK
					ROBOT_LeftGunCheckShootPoint();
#endif
					ROBOT_LeftGunShoot();
					if(gRobot.leftGun.shootParaMode == SHOOT_METHOD2)
					{
						gRobot.plantState[gRobot.leftGun.targetPlant].plateState = COMMAND_DONE;
					}
					else
					{
						gRobot.plantState[gRobot.leftGun.targetPlant].ballState = COMMAND_DONE;						
					}
				}
				else
				{
					OSTimeDly(30);
					LeftBack();
				}

		}
		//手动模式用于调试过程中，对端设备只会发送枪号和着陆号，枪的姿态
		//调试过程中着陆台信息没有用，根据shoot标志来开枪
		else if(ROBOT_GunCheckMode(LEFT_GUN) == GUN_MANUAL_MODE)
		{
			gRobot.leftGun.nextStep = 3;
			if(gRobot.leftGun.aim == GUN_START_AIM)
			{
				//获得目标位姿，这里应该由对端设备发送过来，直接更新的gRobot.leftGun中的目标位姿
				//瞄准，此函数最好瞄准完成后再返回
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!					
				ROBOT_LeftGunAim();
				//更新数据库中参数并写入FLASH
//				UpdateLeftGunPosDatabaseManualMode();
//				FlashWriteGunPosData();
				ROBOT_LeftGunCheckAim();
				gRobot.leftGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.leftGun.shoot==GUN_START_SHOOT)
			{
				ROBOT_LeftGunCheckAim();
				ROBOT_LeftGunShoot();
				ROBOT_LeftGunReload();
//				OSTimeDly(50);
				//更改射击命令标记，此标记在接收到对端设备发生命令时更新
				gRobot.leftGun.shoot = GUN_STOP_SHOOT;
			}
			else
			{
				OSTaskSuspend(OS_PRIO_SELF);
			}
		}
		else
		{
			BEEP_ON;
			USART_SendData(UART5, 1);	
			USART_SendData(UART5, 44);
			USART_SendData(UART5, (uint8_t)-100);
			USART_SendData(UART5, (uint8_t)-100);
			USART_SendData(UART5, (uint8_t)-100);
			USART_SendData(UART5, (uint8_t)-100);
			while(1) {}
		}
		LeftGunSendDebugInfo();
		gRobot.leftGun.checkTimeUsage = 0;
	}
}

void RightGunShootTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
#ifndef NO_WALK_TASK
	OSMboxPend(OpenSaftyMbox, 0, &os_err);
#endif
//	RightBack();
	OSTimeDly(50);
	RightPush();
	gRobot.rightGun.mode = GUN_AUTO_MODE;
	//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
	while(1)
	{
		if(gRobot.rightGun.modeChangeFlag == 1)
		{
			RightBack();
			gRobot.rightGun.modeChangeFlag = 0;
		}
		//检查手动or自动
		//auto mode用在正式比赛中，平板上位机只会发送枪号和柱子号
		if(ROBOT_GunCheckMode(RIGHT_GUN) == GUN_AUTO_MODE)
		{
//			if(gRobot.rightGun.shootTimes >= ROBOT_RightGunPoint1ShootTimes() + ROBOT_RightGunPoint3ShootTimes())
//			{


				shoot_command_t rightGunShootCommand = ROBOT_RightGunGetShootCommand();

				if(gRobot.rightGun.commandState == GUN_HAVE_COMMAND)
				{
					gRobot.rightGun.nextStep = 2;
					gRobot.rightGun.targetPlant = rightGunShootCommand.plantNum;
					gRobot.rightGun.shootParaMode = rightGunShootCommand.shootMethod;

					if(rightGunShootCommand.plantNum == PLANT7 || rightGunShootCommand.plantNum == PLANT3)
					{
						//获取并更新枪目标姿态  上弹姿态
						gRobot.rightGun.targetPose = gRightGunReloadPosDatabase[rightGunShootCommand.shootPoint]\
																			[rightGunShootCommand.shootMethod]\
																			[rightGunShootCommand.plantNum];

						//调整枪姿为上弹姿态 need some time
						ROBOT_RightGunAim();
						ROBOT_RightGunCheckAim();
					}
					else
					{
						gRobot.rightGun.targetPose = gRightGunPosDatabase[rightGunShootCommand.shootPoint]\
																		[rightGunShootCommand.shootMethod]\
																		[rightGunShootCommand.plantNum];
						ROBOT_RightGunAim();
					}
					ROBOT_RightGunReload();				

					//检查上弹是否到位
//					ROBOT_RightGunCheckReload();
					//瞄准，此函数最好瞄准完成后再返回

					//获取并更新枪目标姿态  发射姿态
					gRobot.rightGun.targetPose = gRightGunPosDatabase[rightGunShootCommand.shootPoint]\
																		[rightGunShootCommand.shootMethod]\
																		[rightGunShootCommand.plantNum];

					//调整枪姿为发射姿态 need some time
					ROBOT_RightGunAim();

					if(rightGunShootCommand.plantNum == PLANT6)
					{
//						OSTimeDly(300);
						ROBOT_RightGunCheckAim();
//						gRobot.rightGun.ready = GUN_AIM_DONE;
					}
					else
					{
						ROBOT_RightGunCheckAim();
//						OSTimeDly(150);					
//						gRobot.rightGun.ready = GUN_AIM_DONE;
					}
//					ROBOT_RightGunCheckAim();
//					OSTimeDly(250);					
//					gRobot.rightGun.ready = GUN_AIM_DONE;
//				OSTimeDly(75);
//				if(rightGunShootCommand.plantNum==PLANT6)


				
//				//再次检查该柱子的状态，确定是否发射
//				if((rightGunShootCommand.shootMethod == SHOOT_METHOD1)\
//					&&(gRobot.plantState[rightGunShootCommand.plantNum].ball == 1))
//				{
//					gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
//				}
//				if((rightGunShootCommand.shootMethod == SHOOT_METHOD2)\
//					&&(gRobot.plantState[rightGunShootCommand.plantNum].plate == 1))
//				{
//					gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
//				}

#ifndef NO_WALK_TASK
					ROBOT_RightGunCheckShootPoint();
#endif
					ROBOT_RightGunShoot();
					if(gRobot.rightGun.shootParaMode == SHOOT_METHOD2)
					{
						gRobot.plantState[gRobot.rightGun.targetPlant].plateState = COMMAND_DONE;
					}
					else
					{
						gRobot.plantState[gRobot.rightGun.targetPlant].ballState = COMMAND_DONE;						
					}
				}
				else
				{
					OSTimeDly(30);
					RightBack();
				}
				
		}
		//手动模式用于调试过程中，对端设备只会发送枪号和着陆号，枪的姿态
		//调试过程中着陆台信息没有用，根据shoot标志来开枪
		else if(ROBOT_GunCheckMode(RIGHT_GUN) == GUN_MANUAL_MODE)
		{
			gRobot.rightGun.nextStep = 3;
			if(gRobot.rightGun.aim == GUN_START_AIM)
			{
				//获得目标位姿，这里应该由对端设备发送过来，直接更新的gRobot.leftGun中的目标位姿
				//瞄准，此函数最好瞄准完成后再返回
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!					
				ROBOT_RightGunAim();
				//更新数据库中参数并写入FLASH
//				UpdateRightGunPosDatabaseManualMode();
//				FlashWriteGunPosData();
				ROBOT_RightGunCheckAim();
				gRobot.rightGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.rightGun.shoot==GUN_START_SHOOT)
			{
				ROBOT_RightGunCheckAim();
				ROBOT_RightGunShoot();
				ROBOT_RightGunReload();
//				OSTimeDly(50);
				//更改射击命令标记，此标记在接收到对端设备发生命令时更新
				gRobot.rightGun.shoot = GUN_STOP_SHOOT;
			}
			else
			{
				OSTaskSuspend(OS_PRIO_SELF);
			}
		}
		else
		{
			BEEP_ON;
			USART_SendData(UART5, 2);	
			USART_SendData(UART5, 44);
			USART_SendData(UART5, (uint8_t)-100);
			USART_SendData(UART5, (uint8_t)-100);
			USART_SendData(UART5, (uint8_t)-100);
			USART_SendData(UART5, (uint8_t)-100);
			while(1) {}
		}
		RightGunSendDebugInfo();
		gRobot.rightGun.checkTimeUsage = 0;
	}

}

void UpperGunShootTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;

	//fix me, if camera send data, this flag = 1
	uint8_t upperGunShootFlag = 0;
	gRobot.upperGun.mode = GUN_ATTACK_MODE;
	while(1)
	{
#ifndef NO_WALK_TASK
		if(gRobot.upperGun.targetZone & 0xff)gRobot.upperGun.mode = GUN_DEFEND_MODE;
#endif
		//检查手动or自动
		//auto mode用在正式比赛中，与左右两枪不同，通过摄像头的反馈发射飞盘
		if(ROBOT_GunCheckMode(UPPER_GUN) == GUN_DEFEND_MODE)
		{
			//fix me,此处应该检查目标区域是否合法
			if(gRobot.upperGun.targetZone & 0xff) upperGunShootFlag = 1;
			if(upperGunShootFlag == 1)
			{
				gRobot.upperGun.targetPlant = PLANT7;
				gRobot.upperGun.shootParaMode = SHOOT_METHOD3;
				int zoneId = INVALID_ZONE_NUMBER;
				//fix me,此处应该检查着陆台编号是否合法
				if(gRobot.upperGun.targetZone & 0x01)      zoneId = ZONE1;
				else if(gRobot.upperGun.targetZone & 0x02) zoneId = ZONE2;
				else if(gRobot.upperGun.targetZone & 0x04) zoneId = ZONE3;
				else if(gRobot.upperGun.targetZone & 0x08) zoneId = ZONE4;
				else if(gRobot.upperGun.targetZone & 0x10) zoneId = ZONE5;
				else if(gRobot.upperGun.targetZone & 0x20) zoneId = ZONE6;
				//fix me,there may be something wrong
				else                                       continue;

				//获取目标位姿
				gun_pose_t pose = gUpperGunPosDatabase[gRobot.upperGun.targetPlant][gRobot.upperGun.shootParaMode][zoneId];
				//fix me,这里存在的风险是，自动过程中，手动修改柱子命令，这时候有可能结果不一致，要改

				//更新枪目标位姿
				gRobot.upperGun.targetPose.pitch = pose.pitch;
				gRobot.upperGun.targetPose.yaw = pose.yaw;
				gRobot.upperGun.targetPose.speed1 = pose.speed1;

				//瞄准，此函数最好瞄准完成后再返回 
				ROBOT_UpperGunAim();
				ROBOT_UpperGunCheckAim();
				if (gRobot.upperGun.shoot == GUN_START_SHOOT)
				{
					ROBOT_UpperGunShoot();
					gRobot.upperGun.shoot = GUN_STOP_SHOOT;
					gRobot.upperGun.targetZone = 0x00;
					upperGunShootFlag = 0;
					OSTimeDly(40);
				}
				if(gRobot.upperGun.targetZone == 0)
				{
					upperGunShootFlag = 0;
				}
				UpperGunSendDebugInfo();
			}
			else
			{
				gRobot.upperGun.mode = GUN_ATTACK_MODE;
//				OSTaskSuspend(OS_PRIO_SELF);
			}
		}
		else if(ROBOT_GunCheckMode(UPPER_GUN) == GUN_ATTACK_MODE)
		{
			shoot_command_t shootCommand = ROBOT_UpperGunGetShootCommand();
			gRobot.upperGun.targetPlant = shootCommand.plantNum;
			gRobot.upperGun.shootParaMode = shootCommand.shootMethod;
			if(gRobot.upperGun.commandState == GUN_HAVE_COMMAND)
			{
				uint8_t targetPlant = shootCommand.plantNum;
				uint8_t shootMethod = shootCommand.shootMethod;
				uint8_t shootZone = ZONE1;
				//获取目标位姿
				gun_pose_t pose = gUpperGunPosDatabase[targetPlant][shootMethod][shootZone];
				//更新枪目标位姿
				gRobot.upperGun.targetPose.pitch = pose.pitch;
				gRobot.upperGun.targetPose.yaw = pose.yaw;
				gRobot.upperGun.targetPose.speed1 = pose.speed1;
				
				ROBOT_UpperGunAim();
				ROBOT_UpperGunCheckAim();
				if(gRobot.upperGun.targetZone & 0xff)
				{
					gRobot.upperGun.mode = GUN_DEFEND_MODE;
				}
				else
				{
					ROBOT_UpperGunShoot();
					if(gRobot.upperGun.shootParaMode == SHOOT_METHOD2)
					{
						gRobot.plantState[gRobot.upperGun.targetPlant].plateState = COMMAND_DONE;
					}
					else
					{
						gRobot.plantState[gRobot.upperGun.targetPlant].ballState = COMMAND_DONE;						
					}
				}
				gRobot.upperGun.commandState = GUN_NO_COMMAND;
			}
			else
			{
				ROBOT_UpperGunHome();
				OSTimeDly(6);
			}
		}
		//手动模式用于调试过程中，对端设备只会发送枪号和着陆号，枪的姿态
		//调试过程中着陆台信息没有用，根据shoot标志来开枪
		else if(ROBOT_GunCheckMode(UPPER_GUN) == GUN_MANUAL_MODE)
		{
			if(gRobot.upperGun.aim == GUN_START_AIM)
			{
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!					
				ROBOT_UpperGunAim();
				gRobot.upperGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.upperGun.shoot==GUN_START_SHOOT)
			{
				ROBOT_UpperGunCheckAim();				
				ROBOT_UpperGunShoot();
				//更改射击命令标记，此标记在接收到对端设备发生命令时更新
				gRobot.upperGun.shoot = GUN_STOP_SHOOT;
			}
			else
			{
				OSTaskSuspend(OS_PRIO_SELF);
			}
		}
		else
		{
			BEEP_ON;
			while(1) {}
		}
	}
}
void DebugTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	OSSemSet(DebugPeriodSem, 0, &os_err);
	while(1)
	{
			OSSemPend(DebugPeriodSem, 0, &os_err);
//			if(PHOTOSENSORUPGUN)
//			{
//				ClampClose();
//			}
			ReadActualPos(CAN1,RIGHT_GUN_GROUP_ID);		
			ReadActualVel(CAN1,RIGHT_GUN_VEL_GROUP_ID);
			ReadActualPos(CAN1,LEFT_GUN_GROUP_ID);		
			ReadActualVel(CAN1,LEFT_GUN_VEL_GROUP_ID);
//			USART_SendData(UART5,OSCPUUsage);
//			USART_SendData(UART5,(uint8_t)-100);
//			USART_SendData(UART5,(uint8_t)-100);
//			USART_SendData(UART5,(uint8_t)-100);
//			USART_SendData(UART5,(uint8_t)-100);			
//			ReadActualPos(CAN1,LEFT_GUN_GROUP_ID);		
//			ReadActualVel(CAN1,LEFT_GUN_VEL_GROUP_ID);		
//			ReadActualPos(CAN1,RIGHT_GUN_GROUP_ID);		
//			ReadActualVel(CAN1,RIGHT_GUN_VEL_GROUP_ID);
//			LeftGunSendDebugInfo();
//			RightGunSendDebugInfo();
	}

}

