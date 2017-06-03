#include "includes.h"
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
#include "dma.h"

//#define NO_WALK_TASK

//宏定义标记左右枪没有命令时收回气缸的时间
#define NO_COMMAND_COUNTER 250			//0.25s
/*重试时记录角度和x y方向 角度的误差*/
float gyroAngleErr = 0.0f;
	//此变量记录离开出发区时 在前方的光电不触发时的X方向的坐标
	float startLeaveX = 0.0f;
	/*供重试时矫正原点的偏移使用*/
	//记录光电没有出发的次数 10ms 一次
	uint8_t startLeaveCnt = 0u;
float gyroXErr = 0.0f;
float gyroYErr = 0.0f;

extern uint8_t receive_data;
extern uint8_t receiveDataTrust;
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
OS_EVENT *DebugPeriodSem;
OS_EVENT *GyroSem;
//定义互斥型信号量用于管理CAN发送资源
OS_EVENT *CANSendMutex;
//邮箱定义
OS_EVENT *OpenSaftyMbox;
OS_EVENT *LeftGunShootPointMbox;
OS_EVENT *RightGunShootPointMbox;
OS_EVENT *UpperGunShootPointMbox;

//定义机器人全局变量
extern robot_t gRobot;

//状态变量
typedef enum
{
	getReady,
	goToLoadingArea,
	load,
	beginToGo1,
	goToLaunchingArea,
	stopRobot,
	launch,
	reset,
	resetConfig,
	resetRunToLoad,
	resetRunToLaunch
}Status_t;

Status_t status = getReady;
//自检状态变量
typedef enum
{
	wheelSpeedCheck,
	gpsCheck,
	gunCheck,
	cameraCheck,
	photoelectricCheck,
	commicationCheck
}StatusCheck_t;
StatusCheck_t status_check=wheelSpeedCheck;

static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];
static  OS_STK  WalkTaskStk[Walk_TASK_STK_SIZE];
static  OS_STK  LeftGunShootTaskStk[LEFT_GUN_AUTO_SHOOT_STK_SIZE];
static  OS_STK  RightGunShootTaskStk[RIGHT_GUN_SHOOT_STK_SIZE];
static  OS_STK  UpperGunShootTaskStk[UPPER_GUN_SHOOT_STK_SIZE];
static 	OS_STK  DebugTaskStk[DEBUG_TASK_STK_SIZE];
static 	OS_STK  SelfCheckTaskStk[SELFCHECK_TASK_STK_SIZE];
void LeftGunShootTask(void);
void RightGunShootTask(void);
void UpperGunShootTask(void);
//定位系统初始化
void GyroInit(void)
{
	USART_SendData(USART6,'A');
	USART_SendData(USART6,'C');
	USART_SendData(USART6,'C');
	USART_SendData(USART6,'T');
	USART_SendData(USART6,'0');
	USART_SendData(USART6,'0');
	USART_SendData(USART6,'0');
}
//调试数据发送不能超过30个字节，发送10个字节需要1ms
void sendDebugInfo(void)
{

	UART5_OUT((uint8_t *)"%d\t%d\t%d\t%d\t",status,\
			(int)(gRobot.moveBase.actualAngle * 100.0f),(int)gRobot.moveBase.actualXPos,\
			(int)gRobot.moveBase.actualYPos);

	UART5_OUT((uint8_t *)"%d\t%d\t%d\t",(int)gRobot.moveBase.targetSpeed.leftWheelSpeed,\
			(int)gRobot.moveBase.targetSpeed.forwardWheelSpeed,(int)gRobot.moveBase.targetSpeed.backwardWheelSpeed);

	UART5_OUT((uint8_t *)"%d\t%d\t%d\t",(int)gRobot.moveBase.actualSpeed.leftWheelSpeed,\
			(int)gRobot.moveBase.actualSpeed.forwardWheelSpeed,(int)gRobot.moveBase.actualSpeed.backwardWheelSpeed);

//	UART5_OUT((char *)"%d\t%d\t%d\t%d\t",(int)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[0],\
//			(int)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[1],\
//			(int)(int8_t)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[2],\
//			(int)(int8_t)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[3]);

	UART5_OUT((uint8_t *)"%d\t%d\t%d\t",(int)gRobot.moveBase.acturalCurrent.leftWheelCurrent,\
			(int)gRobot.moveBase.acturalCurrent.forwardWheelCurrent,(int)gRobot.moveBase.acturalCurrent.backwardWheelCurrent);

//	UART5_OUT((uint8_t *)"%d\t%d\t%d\t",(int)gRobot.moveBase.driverTemperature.leftWheelDriverTemperature,\
//			(int)gRobot.moveBase.driverTemperature.forwardWheelDrvierTemperature,(int)gRobot.moveBase.driverTemperature.backwardWheelDriverTemperature);

//	UART5_OUT((char *)"%d\t%d\t%d\t",(int)gRobot.moveBase.driverCurrentLimitFlag.leftWheelDriverFlag,\
//			(int)gRobot.moveBase.driverCurrentLimitFlag.forwardWheelDriverFlag,(int)gRobot.moveBase.driverCurrentLimitFlag.backwardWheelDriverFlag);

	UART5_OUT((uint8_t *)"%d\t",(int)gRobot.moveBase.driverCommandVelocity.leftDriverCommandVelocity );/*\
			(int)gRobot.moveBase.driverCommandVelocity.forwardDriverCommandVelocity,(int)gRobot.moveBase.driverCommandVelocity.backwardDriverCommandVelocity);*/

//	UART5_OUT((uint8_t *)"%d\t%d\t%d\t",(int)gRobot.moveBase.driverJoggingVelocity.leftDriverJoggingVelocity,\
//			(int)gRobot.moveBase.driverJoggingVelocity.forwardDriverJoggingVelocity,(int)gRobot.moveBase.driverJoggingVelocity.backwardDriverJoggingVelocity);

	UART5_OUT((uint8_t *)"%d",(int)(gRobot.moveBase.actualKenimaticInfo.vt*0.1f));

	UART5_OUT((uint8_t *)"X\t%d\t%d\t%d\t%d\t%d\t%d",(int)PHOTOSENSORLEFT, (int)PHOTOSENSORRIGHT, (int)startLeaveX, (int)startLeaveCnt, (int)(gyroXErr*10.0f), (int)(gyroYErr * tan(ANGTORAD(-gyroAngleErr))*10.0f));
	UART5BufPut('\r');
	UART5BufPut('\n');
}
void LeftGunSendDebugInfo(void)
{
	UART5_OUT((uint8_t *)"l\t%d\t%d\t%d\t%d\t",(int)gRobot.leftGun.checkTimeUsage,\
		(int)gRobot.leftGun.targetPlant,(int) gRobot.leftGun.shootParaMode,(int)gRobot.leftGun.commandState);

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.yaw*10.0f),\
		(int)(gRobot.leftGun.actualPose.yaw*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.pitch*10.0f),\
		(int)(gRobot.leftGun.actualPose.pitch*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.roll*10.0f),\
		(int)(gRobot.leftGun.actualPose.roll*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.speed1),\
		(int)(gRobot.leftGun.actualPose.speed1));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.speed2),\
		(int)(gRobot.leftGun.actualPose.speed2));

	UART5_OUT((uint8_t *)"su%d\t%d\t%d",(int)receive_data, (int)receiveDataTrust,\
		(int)(gRobot.leftGun.gunCommand == (plant_t *)gRobot.plantState));


	UART5BufPut('\r');
	UART5BufPut('\n');

}
void RightGunSendDebugInfo(void)
{
	UART5_OUT((uint8_t *)"r\t%d\t%d\t%d\t%d\t",(int)gRobot.rightGun.checkTimeUsage,\
		(int)gRobot.rightGun.targetPlant,(int) gRobot.rightGun.shootParaMode,(int)gRobot.rightGun.commandState);

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.rightGun.targetPose.yaw*10.0f),\
		(int)(gRobot.rightGun.actualPose.yaw*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.rightGun.targetPose.pitch*10.0f),\
		(int)(gRobot.rightGun.actualPose.pitch*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.rightGun.targetPose.roll*10.0f),\
		(int)(gRobot.rightGun.actualPose.roll*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.rightGun.targetPose.speed1),\
		(int)(gRobot.rightGun.actualPose.speed1));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.rightGun.targetPose.speed2),\
		(int)(gRobot.rightGun.actualPose.speed2));

	UART5_OUT((uint8_t *)"su%d\t%d\t%d",(int)receive_data, (int)receiveDataTrust,\
		(int)(gRobot.rightGun.gunCommand == (plant_t *)gRobot.plantState));

	UART5BufPut('\r');
	UART5BufPut('\n');

}
void UpperGunSendDebugInfo(void)
{
	UART5_OUT((uint8_t *)"u\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",(int)gRobot.upperGun.checkTimeUsage,\
		(int)gRobot.upperGun.targetPlant,(int)gRobot.upperGun.presentDefendZoneId,(int)gRobot.upperGun.lastDefendZoneId,\
		(int)gRobot.upperGun.defendZone1,(int)gRobot.upperGun.defendZone2,\
		(int)gRobot.upperGun.shootParaMode,(int)gRobot.upperGun.commandState);

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.upperGun.targetPose.yaw*10.0f),\
		(int)(gRobot.upperGun.actualPose.yaw*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.upperGun.targetPose.pitch*10.0f),\
		(int)(gRobot.upperGun.actualPose.pitch*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.upperGun.targetPose.speed1),\
		(int)(gRobot.upperGun.actualPose.speed1));

	UART5BufPut('\r');
	UART5BufPut('\n');

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
//通知摄像头自动射击完成 看近台6#
void SendAutoOver2Camera(void)
{
	USART_SendData(USART3, 'e');
	USART_SendData(USART3, 'e');
	USART_SendData(USART3, 's');
}

//通知摄像头看1245#柱子
void SendWatchWholeArena2Camera(void)
{
//	UART5_OUT()
	USART_SendData(USART3, 'e');
	USART_SendData(USART3, 'e');
	USART_SendData(USART3, 'p');
}
void CameraSelfCheck(void)
{
	USART_SendData(USART3, 'z');
	USART_SendData(USART3, 'z');
	USART_SendData(USART3, 'z');	
}
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;		  /*防止警告...*/

	/*创建信号量*/
	PeriodSem				=	OSSemCreate(0);
	DebugPeriodSem		  =   OSSemCreate(0);
	GyroSem				 =	OSSemCreate(0);

	//创建互斥型信号量
	CANSendMutex			=   OSMutexCreate(9,&os_err);

	//创建邮箱
	OpenSaftyMbox			=   OSMboxCreate((void *)0);
	LeftGunShootPointMbox	=   OSMboxCreate((void *)0);
	RightGunShootPointMbox   =   OSMboxCreate((void *)0);
	UpperGunShootPointMbox   =   OSMboxCreate((void *)0);

	/*创建任务*/
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,				/*初始化任务*/
							(void		  * ) 0,
							(OS_STK		* )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],
							(INT8U		   ) Config_TASK_START_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) WalkTask,
							(void		  * ) 0,
							(OS_STK		* )&WalkTaskStk[Walk_TASK_STK_SIZE-1],
							(INT8U		   ) Walk_TASK_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) LeftGunShootTask,
							(void		  * ) 0,
							(OS_STK		* )&LeftGunShootTaskStk[LEFT_GUN_AUTO_SHOOT_STK_SIZE-1],
							(INT8U		   ) LEFT_GUN_SHOOT_TASK_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) RightGunShootTask,
							(void		  * ) 0,
							(OS_STK		* )&RightGunShootTaskStk[RIGHT_GUN_SHOOT_STK_SIZE-1],
							(INT8U		   ) RIGHT_GUN_SHOOT_TASK_PRIO);
	os_err = OSTaskCreate(	(void (*)(void *)) UpperGunShootTask,
							(void		  * ) 0,
							(OS_STK		* )&UpperGunShootTaskStk[UPPER_GUN_SHOOT_STK_SIZE-1],
							(INT8U		   ) UPPER_GUN_SHOOT_TASK_PRIO);
	os_err = OSTaskCreate(	(void (*)(void *)) DebugTask,
							(void		  * ) 0,
							(OS_STK		* )&DebugTaskStk[DEBUG_TASK_STK_SIZE-1],
							(INT8U		   ) DEBUG_TASK_PRIO);
	os_err = OSTaskCreate(	(void (*)(void *)) SelfCheckTask,
							(void		  * ) 0,
							(OS_STK		* )&SelfCheckTaskStk[SELFCHECK_TASK_STK_SIZE-1],
							(INT8U		   ) SELFCHECK_TASK_PRIO);
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


	
//	while(!KEYSWITCH)
//	{
//		//等待给摄像头发送初始化命令
//	}
	//************************
	USART3_Init(115200);	//摄像头
	CameraInit();
	//***********************

	//定时器初始化
	TIM_Init(TIM2, 99, 839, 0, 0);   //1ms主定时器

	KeyInit();
	LEDInit();
	PhotoelectricityInit();
	BeepInit();

	//串口初始化
	UART4_Init(115200);	 //蓝牙手柄
	USART1_Init(115200);
	USART2_Init(115200);
	//WIFI串口初始化
	UART5DMAInit();

	USART6_Init(115200);	//定位系统
	//	FlashInit();
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
	
//	while(!RESET_SWITCH)
//	{
//		//等待初始化
//	} 
	TIM_Delayms(TIM5, 1000);

	ROBOT_Init();
	
	ClampClose();
	//	LeftBack();
	//	RightBack();
	LeftPush();
	RightPush();
	ClampReset();
	
#ifndef NO_WALK_TASK
#ifdef BLUE_FIELD
	BLUE_LED_ON;
#endif
#ifdef RED_FIELD
	RED_LED_ON;
#endif

	TIM_Delayms(TIM5, 14000);

#ifdef BLUE_FIELD
	BLUE_LED_OFF;
#endif
#ifdef RED_FIELD
	RED_LED_OFF;
#endif
#endif



//	TIM_Delayms(TIM5, 50);

//	ROBOT_Init();

//	ClampClose();
//	//	LeftBack();
//	//	RightBack();
//	LeftPush();
//	RightPush();
//	ClampReset();

#ifndef NO_WALK_TASK
#ifdef BLUE_FIELD
	BLUE_LED_ON;
	BEEP_ON;
	TIM_Delayms(TIM5, 1000);
	BEEP_OFF;
	BLUE_LED_OFF;
#endif
#ifdef RED_FIELD
	RED_LED_ON;
	BEEP_ON;
	TIM_Delayms(TIM5, 300);
	BEEP_OFF;
	TIM_Delayms(TIM5, 200);
	BEEP_ON;
	TIM_Delayms(TIM5, 300);
	BEEP_OFF;
	TIM_Delayms(TIM5, 200);
	RED_LED_OFF;
#endif
#endif
#ifdef NO_WALK_TASK
	BEEP_ON;
	TIM_Delayms(TIM5, 300);
	BEEP_OFF;
	TIM_Delayms(TIM5, 200);
	BEEP_ON;
	TIM_Delayms(TIM5, 300);
	BEEP_OFF;
	TIM_Delayms(TIM5, 200);
	BEEP_ON;
	TIM_Delayms(TIM5, 300);
	BEEP_OFF;
	TIM_Delayms(TIM5, 200);
	BEEP_ON;
	TIM_Delayms(TIM5, 300);
	BEEP_OFF;
	TIM_Delayms(TIM5, 200);
#endif
	/*
	   如果行程开关触发  挂起所有枪 走形任务
	   进入自检任务
	   */

	if(RESET_SWITCH == 1)
	{
		BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
		BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
		BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
#ifdef RED_FIELD
		RED_LED_ON;
#endif
#ifdef BLUE_FIELD
		BLUE_LED_ON;
#endif
		OSTaskSuspend(Walk_TASK_PRIO);
		OSTaskSuspend(LEFT_GUN_SHOOT_TASK_PRIO);
		OSTaskSuspend(RIGHT_GUN_SHOOT_TASK_PRIO);
		OSTaskSuspend(UPPER_GUN_SHOOT_TASK_PRIO);
		OSTaskSuspend(DEBUG_TASK_PRIO);
		OSTaskSuspend(OS_PRIO_SELF);
	}
	else
	{
		OSTaskDel(SELFCHECK_TASK_PRIO);
	}

	//	LeftHold();
	//	RightHold();
	//////测试使用！！！！！！！！！！！！！！！！
	//	MoveY(50.0f);
#ifdef NO_WALK_TASK
	while(!PHOTOSENSORUPGUN)
	{
		//WAIT
	}
	SendStop2Camera();
	//	MoveY(50);
	OSTaskSuspend(Walk_TASK_PRIO);
#endif

	//	OSTaskSuspend(LEFT_GUN_SHOOT_TASK_PRIO);
	//	OSTaskSuspend(RIGHT_GUN_SHOOT_TASK_PRIO);
#ifndef NO_WALK_TASK
//	OSTaskSuspend(UPPER_GUN_SHOOT_TASK_PRIO);
#endif
//	OSTaskSuspend(DEBUG_TASK_PRIO);
	OSTaskSuspend(OS_PRIO_SELF);
}



/*
   ===============================================================
   自检任务
   1.轮子正转2s  反转2s
   2.定位系统
   3.下枪两个+上枪 依次推弹1次
   4.下枪两个 依次测横滚   三个枪依次测左右  三个枪依次测俯仰

   5.
   摄像头发送SendStop2Camera
   开始通过串口接受数据
   然后通过屏幕校验

   5.三个光电检测
   （触发蜂鸣器状态取反）
   ===============================================================
   */

void SelfCheckTask(void)
{
	static int self_circle=0,self_circle_end=3;
	static uint8_t emptyQueueFlag = 1;
	CameraSelfCheck();
	while(!PHOTOSENSORUPGUN)
	{
		//WAIT
		UART5_OUT((uint8_t *)"wait for self check\r\n");
		TIM_Delayms(TIM5, 400);
	}
	while(1)
	{
		switch(status_check)
		{
			case wheelSpeedCheck:
				UART5_OUT((uint8_t *)"motor check\r\n");

				//正转1s
				ThreeWheelVelControlSelfCheck(1);
				TIM_Delayms(TIM5, 4000);

				ThreeWheelVelControlSelfCheck(3);
				TIM_Delayms(TIM5, 1000);
				//反转1s
				ThreeWheelVelControlSelfCheck(2);
				TIM_Delayms(TIM5, 4000);
				ThreeWheelVelControlSelfCheck(3);
				TIM_Delayms(TIM5, 1000);
				//正转1s
				ThreeWheelVelControlSelfCheck(1);
				TIM_Delayms(TIM5, 4000);
				ThreeWheelVelControlSelfCheck(3);
				TIM_Delayms(TIM5, 1000);
				//反转1s
				ThreeWheelVelControlSelfCheck(2);
				TIM_Delayms(TIM5, 4000);
				ThreeWheelVelControlSelfCheck(3);
				TIM_Delayms(TIM5, 1000);


				VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(50.0f));
				VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(50.0f));
				VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(50.0f));
				TIM_Delayms(TIM5, 8000);
				VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(0.0f));
				VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(0.0f));

				TIM_Delayms(TIM5, 1000);

				VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(50.0f));
				VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(50.0f));
				TIM_Delayms(TIM5, 8000);
				VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(0.0f));
				VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(0.0f));
				VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(0.0f));
				status_check++;

				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				break;
			case gpsCheck:
				GyroInit();

				UART5_OUT((uint8_t *)"angle:%d\tX:%d\tY:%d\r\n",\
						(int)gRobot.moveBase.actualAngle,(int)gRobot.moveBase.actualXPos,\
						(int)gRobot.moveBase.actualYPos);

				TIM_Delayms(TIM5, 200);


				if(RESET_SWITCH==1)
				{
					status_check++;
					BEEP_ON;TIM_Delayms(TIM5, 1000);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				}
				break;
			case gunCheck:
				UART5_OUT((uint8_t *)"gun check\r\n");

				//夹子开
				ClampOpen();TIM_Delayms(TIM5, 1000);

				//夹子关
				ClampClose();TIM_Delayms(TIM5, 1000);

				PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(20.0f));
				PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(20.0f));

				for(self_circle=0;self_circle<self_circle_end;self_circle++)
				{
					LeftPush();RightPush();TIM_Delayms(TIM5, 500);LeftBack();RightBack();TIM_Delayms(TIM5, 500);
				}

				for(self_circle=0;self_circle<self_circle_end;self_circle++)
				{
					LeftShoot();RightShoot();UpperShoot();
					TIM_Delayms(TIM5, 500);
					LeftShootReset();RightShootReset();UpperShootReset();
					TIM_Delayms(TIM5, 500);
				}

				/************下枪左*********/
				VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(140.0f));
				VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(140.0f));
				/************下枪右*********/
				VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(140.0f));
				VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(140.0f));
				/************上枪*********/
				VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(140.0f));
				
				for(self_circle=0;self_circle<self_circle_end;self_circle++)
				{
					//横滚向一侧一定角度
					PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(40.0f));
					//横滚向一侧一定角度
					PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(40.0f));					
					TIM_Delayms(TIM5, 1000);
					//横滚向另一侧一定角度
					PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(0.0f));
					//横滚向另一侧一定角度
					PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(0.0f));
					TIM_Delayms(TIM5, 1000);

				}

				for(self_circle=0;self_circle<self_circle_end;self_circle++)
				{
					//左转一定角度
					PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(20.0f));
					//左转一定角度
					PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(20.0f));
					//左转一定角度
					PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(20.0f));
					TIM_Delayms(TIM5, 1000);					
					//右转一定角度
					PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(-20.0f));
					//右转一定角度
					PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(-20.0f));
					//右转一定角度
					PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(-20.0f));
					TIM_Delayms(TIM5, 1000);
				}

				PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(0.0f));
				PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(0.0f));
				PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(0.0f));
				TIM_Delayms(TIM5, 1000);

				for(self_circle=0;self_circle<self_circle_end;self_circle++)
				{
					//俯仰向上一定角度
					PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(40.0f));
					//俯仰向上一定角度
					PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(40.0f));
					//俯仰向上一定角度 起始角度
					PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(20.0f));
					TIM_Delayms(TIM5, 1000);
					//俯仰向下一定角度
					PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(10.0f));
					//俯仰向下一定角度
					PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(10.0f));
					//俯仰向下一定角度
					PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(-8.0f));
					TIM_Delayms(TIM5, 1000);
				}


				PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(20.0f));
				VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(0.0f));
				VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(0.0f));

				PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(20.0f));
				VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(0.0f));
				VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(0.0f));

				PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(0.0f));
				VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(0.0f));

				status_check++;
				BEEP_ON;TIM_Delayms(TIM5, 1000);BEEP_OFF;TIM_Delayms(TIM5, 100);
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				SendStop2Camera();
				break;
			case cameraCheck:

				UART5_OUT((uint8_t *)"%d\r\n",(int)gRobot.upperGun.defendZone1);
				UART5_OUT((uint8_t *)"%d\r\n",(int)gRobot.upperGun.defendZone2);
			
				Sendfloat(gRobot.moveBase.actualYPos);
				if(RESET_SWITCH==1)
				{
					BEEP_ON;TIM_Delayms(TIM5, 1000);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					status_check++;
				}
				//gRobot.upperGun.targetZone
				//接受摄像头发送的区域number
				break;
			case photoelectricCheck:

				if(PHOTOSENSORLEFT ||PHOTOSENSORRIGHT||PHOTOSENSORUPGUN||PHOTOSENSORLEFTGUN||PHOTOSENSORRIGHTGUN||KEYSWITCH)
				{
					GPIOE->ODR^=0X80;
					TIM_Delayms(TIM5, 20);
				}
				if(RESET_SWITCH==1)
				{
					status_check++;

					BEEP_ON;TIM_Delayms(TIM5, 1000);BEEP_OFF;TIM_Delayms(TIM5, 1000);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
					BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);

					//夹子翻
					ClampRotate();TIM_Delayms(TIM5, 100);
					//夹子翻转复位
					ClampReset();TIM_Delayms(TIM5, 100);
				}
				break;
			case commicationCheck:
			//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
				if(emptyQueueFlag==1)
				{
					gRobot.manualCmdQueue.headNum = gRobot.manualCmdQueue.tailNum;
					emptyQueueFlag = 0;
				}
				//手自动切换时上弹气缸推到头后收回
				if(gRobot.leftGun.modeChangeFlag == 1)
				{
					gRobot.leftGun.lastPlant = INVALID_PLANT_NUMBER;
					gRobot.leftGun.lastParaMode = INVALID_SHOOT_METHOD;
					LeftBack();
					gRobot.leftGun.modeChangeFlag = 0;
				}
				//等待命令时上弹气缸推到头后收回
				if(gRobot.leftGun.noCommandTimer >= NO_COMMAND_COUNTER)
				{
					LeftBack();
				}
				//自动模式
				if(ROBOT_GunCheckMode(LEFT_GUN) == GUN_AUTO_MODE)
				{
					//自获取命令
		//			shoot_command_t leftGunShootCommand = ROBOT_LeftGunGetShootCommand();
					shoot_command_t leftGunShootCommand = ROBOT_LeftGunGetShootCommandFIFO();
					if(gRobot.leftGun.commandState == GUN_HAVE_COMMAND)
					{
						gRobot.leftGun.noCommandTimer = 0;
						gRobot.leftGun.targetPlant = leftGunShootCommand.plantNum;
						gRobot.leftGun.nextStep = 2;
						gRobot.leftGun.shootParaMode = leftGunShootCommand.shootMethod;
						//对于7#柱子先到位后再上弹，其它柱子直接瞄准
						if(gRobot.leftGun.lastPlant == PLANT7 || leftGunShootCommand.plantNum == PLANT3 )
						{
							//获取并更新枪上弹姿态
							gRobot.leftGun.targetPose = gLeftGunReloadPosDatabase[leftGunShootCommand.shootMethod]\
														[leftGunShootCommand.plantNum];

							ROBOT_LeftGunAim();
							ROBOT_LeftGunCheckAim();
						}
						else/* if(gRobot.leftGun.shootTimes == 0)*/
						{
							gRobot.leftGun.targetPose = gLeftGunPosDatabase[leftGunShootCommand.shootMethod]\
														[leftGunShootCommand.plantNum];
							ROBOT_LeftGunAim();
						}
						//第一发弹先调整姿态一段时间后再上弹
		//				if(gRobot.leftGun.shootTimes == 0 && gRobot.leftGun.champerErrerState == GUN_RELOAD_OK)
		//				{
		//					OSTimeDly(90);
		//					LeftPush();
		//				}
						//上弹
						ROBOT_LeftGunReload();

						gRobot.leftGun.targetPose = gLeftGunPosDatabase[leftGunShootCommand.shootMethod]\
													[leftGunShootCommand.plantNum];
						ROBOT_LeftGunAim();

						//检测枪是否到位
						ROBOT_LeftGunCheckAim();
						//检查上弹是否到位
						ROBOT_LeftGunCheckReload();

						//					OSTimeDly(100);
						//落盘时，检查对应柱子是否已经打球
						if(gRobot.leftGun.shootParaMode%2)
						{
							while(gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].ballState == COMMAND_IN_PROCESS)
							{
								OSTimeDly(1);
							}
						}
						//发射
						ROBOT_LeftGunShoot();

						if(gRobot.leftGun.ready == GUN_AIM_DONE)
						{
							if(gRobot.leftGun.champerErrerState == GUN_RELOAD_OK)
							{
								//记录每个柱子的发射命令
								gRobot.plateShootTimes[gRobot.leftGun.targetPlant]+=1;
								//记录发射命令
								gRobot.leftGun.lastPlant = leftGunShootCommand.plantNum;
								gRobot.leftGun.lastParaMode = leftGunShootCommand.shootMethod;
							}
							else
							{
								if(gRobot.leftGun.shootParaMode%2)
								{
									gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].plate += 1;
								}
								else
								{
									gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].ball += 1;
								}
							}
							SetShootPlantTime(leftGunShootCommand.plantNum, leftGunShootCommand.shootMethod);
						}
						//对命令状态进行复位
						if(gRobot.leftGun.shootParaMode%2)
						{
							gRobot.manualCmdQueue.cmdPlateState^=(0x01<<(gRobot.leftGun.targetPlant));
							gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].plateState = COMMAND_DONE;
						}
						else
						{
							gRobot.manualCmdQueue.cmdBallState^=(0x01<<(gRobot.leftGun.targetPlant));
							gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].ballState = COMMAND_DONE;
						}
					}
					else
					{
						//没有命令时回到6#落盘姿态
						OSTimeDly(6);
						ROBOT_LeftGunReturn();
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
						ROBOT_LeftGunAim();
						//更新数据库中参数并写入FLASH
						//				UpdateLeftGunPosDatabaseManualMode();
						//				FlashWriteGunPosData();
						gRobot.leftGun.aim = GUN_STOP_AIM;
					}
					else if(gRobot.leftGun.shoot==GUN_START_SHOOT)
					{
						//7#柱子需要瞄准，因为需要特殊角度上弹
						if(gRobot.leftGun.targetPlant == PLANT7)
						{
							ROBOT_LeftGunAim();
						}
						//检测左枪是否到位
						ROBOT_LeftGunCheckAim();
						//发射
						ROBOT_LeftGunShoot();

						//7#需要先到上弹角度再上弹
						if(gRobot.leftGun.targetPlant == PLANT7)
						{
							gRobot.leftGun.reloadPose = gLeftGunReloadPosDatabase[gRobot.leftGun.shootParaMode]\
														[gRobot.leftGun.targetPlant];

							ROBOT_LeftGunReloadAim();
							//检测是否到位
							ROBOT_LeftGunCheckReloadAim();
							LeftPush();
						}
						//上弹
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
					while(1)
					{
						UART5_OUT((uint8_t *)"Left Gun Mode Error!!!!!!!!!!\r\n");
					}
				}
				LeftGunSendDebugInfo();
				gRobot.leftGun.checkTimeUsage = 0;
				break;
			default :
				break;
		}
	}
}




//坐标修正量及修正标志位
float amendX = 0.0f;
uint8_t amendXFlag = 0;
//走行移动计时标志位
uint8_t moveTimFlag = 0;


void WalkTask(void)
{
#define LOAD_AREA_STOP_X 13033.14f
#define LAUNCH_STOP_X 6500.14f

	
	//仅在 load 中使用 计时400ms
	static uint16_t timeCounter = 0;
	CPU_INT08U  os_err;
	os_err = os_err;
	int shootPointMsg = MOVEBASE_POS_READY;
	uint8_t setLaunchPosFlag = 1;
	uint8_t sendSignal = 1;
	uint8_t sendSignal2Camera = 1;
	uint8_t clampSmallOpenFlag = 1;
	uint8_t clampSmallOpenCounter = 0;
	//仅在beginToGO1中计时使用
	uint8_t upperPhotoSensorCounter = 0;
	OSSemSet(PeriodSem, 0, &os_err);
	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		//装弹后检查行程开关是否触发 如果触发则发送邮箱 左枪和右枪的任务在此之前一直在等待OpenSaftyMbox邮箱
		if(status >= load)
		{
			ROBOT_CheckGunOpenSafety();
		}
		//检查是否需要重启 fix me 需要更好的处理方式来避免一直进入重试
		if(status == launch)
		{
			if(gRobot.isReset == ROBOT_RESET||RESET_SWITCH)
			{
				//失能电机，中断发射任务
				elmo_Disable(CAN2 , MOVEBASE_BROADCAST_ID);
				if(RESET_SWITCH)
				{
					elmo_Disable(CAN2 , MOVEBASE_BROADCAST_ID);
					//按下以后等待一秒 再进入reset 目的是防止多次进入重试 进入reset后会马上又检测开关是否触发
					gRobot.isReset = ROBOT_RESET;
					BEEP_ON;
					TIM_Delayms(TIM5, 1000);
					BEEP_OFF;
					OSSemSet(PeriodSem,0,&os_err);
				}
				status = reset;
			}
		}
		//在发射以及重启的过程中不读取elmo状态，不发送走行信息
		if(status != launch)
		{
			ReadActualVel(CAN2, MOVEBASE_BROADCAST_ID);
			ReadActualCurrent(CAN2, MOVEBASE_BROADCAST_ID);
			//			ReadActualTemperature(CAN2, MOVEBASE_BROADCAST_ID);
			//			ReadCurrentLimitFlag(CAN2, MOVEBASE_BROADCAST_ID);
			//			ReadVelocityError(CAN2, MOVEBASE_BROADCAST_ID);
			ReadCommandVelocity(CAN2, MOVEBASE_BROADCAST_ID);
			//			ReadJoggingVelocity(CAN2, MOVEBASE_BROADCAST_ID);
			//			ReadMotorFailure(CAN2,MOVEBASE_BROADCAST_ID);
			UpdateKenimaticInfo();
			sendDebugInfo();
		}
		//不在发射时不检测蓝牙通信是否正常
		if(status != launch)
		{
			gRobot.isBleOk.bleCheckStartFlag = BLE_CHECK_STOP;
		}
		switch (status)
		{
			//准备阶段
			case getReady:
				if(PHOTOSENSORUPGUN)
				{
					//photeElectricCounter 五次触发了上枪光电时才会执行命令跳到下一步
					static uint8_t photoElectricCounter = 0;
					photoElectricCounter++;
					if(photoElectricCounter >= 5)
					{
						//在车启动前才发命令给陀螺仪 通知其完成初始化
						GyroInit();
						//等待定位系统信号量
						//OSSemSet 命令先清空了信号量
						OSSemSet(GyroSem, 0, &os_err);
						//OSSemPend 等待信号量 如果一直等待不到 也只会等待 200 clock tick 即 2s
						OSSemPend(GyroSem,200, &os_err);
						//错误处理  将会通过wifi显示错误 并且蜂鸣器响提示
						if(os_err == OS_ERR_TIMEOUT)
						{
							//如果超时没有接收到定位系统数据则提示错误
							while(1)
							{
								UART5_OUT((uint8_t *)"GYRO NO DATA ERROR!!!\r\n");
								BEEP_ON;
								TIM_Delayms(TIM5,500);
								BEEP_OFF;
								TIM_Delayms(TIM5,500);
							}
						}
						//出发后爪子张开
						ClampOpen();
						//fix me maybe useless
						TIM_Delayms(TIM5,20);
						//出发时左右枪复位
//						ROBOT_LeftGunHome();
//						ROBOT_RightGunHome();
						status = goToLoadingArea;
						//在触发光电前 信号量也在一直累加 在等待陀螺仪时也有可能等待了几个tick 故在此必须清信号量
						OSSemSet(PeriodSem, 0, &os_err);
					}
				}
				break;
				//走行开始 从出发区走向装载区
			case goToLoadingArea:
			{
#ifdef RED_FIELD
				MoveTo(-LOAD_AREA_STOP_X, -4200.0f, 2500.0f , 2000.0f);

				//接近装载区时通过光电校正坐标 红场使用右侧光电（处于行进方向后方的光电）
				if (GetPosX() <= -12650.0f && PHOTOSENSORRIGHT)
				{
					//为了防止一直进入矫正环节 设置amendFlag 仅矫正1次
					if (amendXFlag == 0)
					{
						amendX = -12776.96f - GetPosX();
						amendXFlag = 1;
					}
					//矫正的那一瞬间蜂鸣器开始响  直到到达目的地停止下来轮子锁死 蜂鸣器灭
					BEEP_ON;
				}
				//离开出发区时通过光电记录坐标为重试时使用 红场使用左侧光电（处于行进方向前方的光电）
				if(GetPosX() > -500.0f && !PHOTOSENSORLEFT)
				{
					//有3次没有触发才记录
					if(startLeaveCnt < 3u)
					{
						startLeaveCnt++;
					}
					else if(startLeaveCnt == 3u)
					{
						startLeaveX = GetPosX();
						startLeaveCnt++;
					}
				}
				//到达装弹位置
				if(GetPosX()<=-LOAD_AREA_STOP_X)
				{
					//moveTimFlag 是用来控制是否进行走形计时的 在TIM2 中使用
					//由于停止运动了 MoveTo() 停止调用，故必须停止计时
					moveTimFlag = 0;
					status = load;
					BEEP_OFF;
				}
#endif
				//红蓝场走形对称 实现是一样的
#ifdef BLUE_FIELD
				MoveTo(LOAD_AREA_STOP_X, 4200.0f, 2500.0f, 2000.0f);
				//接近装载区时通过光电校正坐标 蓝场使用左侧光电（处于行进方向后方的光电）
				if (GetPosX() >= 12650.0f && PHOTOSENSORLEFT)
				{
					if (amendXFlag == 0)
					{
						amendX = 12776.96f - GetPosX();
						amendXFlag = 1;
					}
					BEEP_ON;
				}
				//离开出发区时通过光电记录坐标为重试时使用 蓝场使用右侧光电（处于行进方向前方的光电）
				if(GetPosX() < 900.0f && !PHOTOSENSORRIGHT)
				{
					//有3次没有触发才记录
					if(startLeaveCnt < 3u)
					{
						startLeaveCnt++;
					}
					else if(startLeaveCnt == 3u)
					{
						startLeaveX = GetPosX();
						startLeaveCnt++;
					}
				}
				//到达装弹位置
				if(GetPosX() >= LOAD_AREA_STOP_X)
				{
					moveTimFlag = 0;
					status = load;
					BEEP_OFF;
				}
#endif
				//光电检测是否下错程序 如果走了一段距离 光电仍然出发 则判断走反了
				if(fabs(gRobot.moveBase.actualXPos)>200.0f && fabs(gRobot.moveBase.actualXPos)<300.0f)
				{
					if(PHOTOSENSORLEFT&&PHOTOSENSORRIGHT)
					{
						while(1)
						{
							LockWheel();
							UART5_OUT((uint8_t *)"WRONG PROGRAM IN FIELD DEFINE!!!\r\n");
							TIM_Delayms(TIM5,200);
						}
					}
				}
				break;
			}
			
				//装载飞盘
			case load:
			{
				//停车
				LockWheel();
				//弹匣的爪子收起
				ClampClose();
				timeCounter++;
				//爪子关一段时间 400 ms 后 爪子翻翻并开始检测光电
				if (timeCounter >= 40)
				{
					//复位
					timeCounter = 0;
					//爪子翻
					ClampRotate();
					status = beginToGo1;
				}
				break;
			}
			
			case beginToGo1:
			{
				//检测上枪光电
				if (PHOTOSENSORUPGUN)
				//检测行程开关
//				if(RESET_SWITCH)
				{
					upperPhotoSensorCounter++;
					//触发10次后开始走向发射区
					if(upperPhotoSensorCounter >= 10)
					{
						ROBOT_UpperGunAim();
						status=goToLaunchingArea;
					}
				}
				break;
			}
				//从装载区走向发射区
			case goToLaunchingArea:
			{
#ifdef RED_FIELD
				//				MoveTo(-6459.14f, 3000.0f, 2500.0f , 2000.0f);
				MoveTo(-LAUNCH_STOP_X, 3000.0f, 2000.0f , 2000.0f);

				//				if (GetPosX() >= -6459.14f)
				if (GetPosX() >= -LAUNCH_STOP_X)
				{
					//翻弹匣的气缸恢复
					ClampReset();
					//停下以后给y方向向前50mm/s的速度向前拱 保证贴墙
					MoveY(50.0f);
					moveTimFlag = 0;
					status = stopRobot;
				}
#endif
#ifdef BLUE_FIELD
				//				MoveTo(6459.14f, -3000.0f, 2500.0f , 2000.0f);
				MoveTo(LAUNCH_STOP_X, -3000.0f, 2000.0f , 2000.0f);

				//到位后给靠墙速度
				//				if (GetPosX() <= 6459.14f)
				if (GetPosX() <= LAUNCH_STOP_X)
				{
					ClampReset();
					MoveY(50.0f);
					//由于停止运动了 MoveTo() 停止调用，故必须停止计时
					moveTimFlag = 0;
					status = stopRobot;
				}
#endif
				if(KEYSWITCH)
				{
					if(clampSmallOpenFlag == 1)
					{
						GasValveControl(CLAMP_CLOSE_BOARD_ID , CLAMP_CLOSE_IO_ID , 0);
						GasValveControl(CLAMP_OPEN_BOARD_ID , CLAMP_OPEN_IO_ID , 1);
						clampSmallOpenFlag = 0;
					}
					if(clampSmallOpenFlag == 0)
					{
						clampSmallOpenCounter++;
					}
					if(clampSmallOpenCounter > 2)
					{
						GasValveControl(CLAMP_CLOSE_BOARD_ID , CLAMP_CLOSE_IO_ID , 1);
						GasValveControl(CLAMP_OPEN_BOARD_ID , CLAMP_OPEN_IO_ID , 0);						
					}
				}
				break;
			}
				//停车
			case stopRobot:
			{
				//通知摄像头开始工作
				SendStop2Camera();
				SendStop2Camera();				
				SendStop2Camera();
				//靠墙一段时间 0.5s 后抱死
				OSTimeDly(20);
				LockWheel();
				//开始执行发射任务
				//三枪的任务都有对应的ROBOT_xxxGunCheckShootPoint()函数 等待着邮箱的发送
				OSMboxPostOpt(LeftGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
				OSMboxPostOpt(RightGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
				OSMboxPostOpt(UpperGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
				//已经完成向前拱的动作，抱死 此时记录1次当前的x y 坐标
				if(setLaunchPosFlag == 1)
				{
					gRobot.launchPosX = gRobot.moveBase.actualXPos;
					gRobot.launchPosY = gRobot.moveBase.actualYPos;
					setLaunchPosFlag-=1;
				}
				//				CameraInit();
				status = launch;
				//如果是从重启进入的 stopRobot 则清除标志位
				if(gRobot.isReset == ROBOT_RESET)
				{
					gRobot.isReset = ROBOT_NOT_RESET;
				}
				OSSemSet(PeriodSem, 0, &os_err);
				break;
			}
				//发射飞盘
			case launch:
			{
				//调节保持位置不动 带死区
				StickPos(gRobot.launchPosX,gRobot.launchPosY);
				gRobot.isReset = ROBOT_NOT_RESET;
				/*对蓝牙是否断开的检测*/
				//把检查ble的Flag 置位
				gRobot.isBleOk.bleCheckStartFlag = BLE_CHECK_START;
				
				/*对于蓝牙失联的处理*/
				if(gRobot.isBleOk.noBleFlag == BLE_LOST)
				{
					//蜂鸣器将长鸣
					BEEP_ON;
					//如果蓝牙失联 则通知视觉模块看全场按照视觉模块反馈的信息打 但是实际上现在视觉反馈的信息没有使用
					if(sendSignal2Camera == 1)
					{
						//向视觉模块发信号，看全场 信息的接收部分再中断中
//						SendWatchWholeArena2Camera();
						sendSignal2Camera = 0;
					}
					//蓝牙失联时，如果任务队列为空 则按照6631245的顺序打全场
					if(gRobot.manualCmdQueue.headNum == gRobot.manualCmdQueue.tailNum &&\
						//加入以下判断 使得 能够按照6631245顺序 一轮一轮打 而不会不停地有命令进队
						gRobot.plantState[PLANT1].plateState == COMMAND_DONE&&\
						gRobot.plantState[PLANT2].plateState == COMMAND_DONE&&\
						gRobot.plantState[PLANT4].plateState == COMMAND_DONE&&\
						gRobot.plantState[PLANT5].plateState == COMMAND_DONE&&\
						gRobot.plantState[PLANT6].plateState == COMMAND_DONE&&\
						gRobot.plantState[PLANT3].plateState == COMMAND_DONE
					)
					{
						cmd_t cmd = {INVALID_PLANT_NUMBER , INVALID_SHOOT_METHOD};
						cmd.plantNum = PLANT6;
						cmd.method = SHOOT_METHOD4;
						InCmdQueue(cmd);
						InCmdQueue(cmd);
						cmd.plantNum = PLANT3;
						cmd.method = SHOOT_METHOD4;
						InCmdQueue(cmd);
						cmd.plantNum = PLANT1;
						cmd.method = SHOOT_METHOD4;
						InCmdQueue(cmd);						
						cmd.plantNum = PLANT2;
						cmd.method = SHOOT_METHOD4;
						InCmdQueue(cmd);						
						cmd.plantNum = PLANT4;
						cmd.method = SHOOT_METHOD4;
						InCmdQueue(cmd);						
						cmd.plantNum = PLANT5;
						cmd.method = SHOOT_METHOD4;
						InCmdQueue(cmd);						
					}
					//视觉模块只能看12457台，如果视觉反馈这些台上都没有需要打球或补盘则落3台和6台
					//这一部分现在实际上没有使用
					if(gRobot.plantState[PLANT1].plate == 0 && gRobot.plantState[PLANT2].plate == 0
						&& gRobot.plantState[PLANT4].plate == 0 && gRobot.plantState[PLANT5].plate == 0
						&& gRobot.plantState[PLANT1].ball == 0 && gRobot.plantState[PLANT2].ball == 0
						&& gRobot.plantState[PLANT4].ball == 0 && gRobot.plantState[PLANT5].ball == 0)
					{
						if(gRobot.plantState[PLANT6].plate == 0)
						{
							UART5_OUT((uint8_t*)"BLE lost Plant6");
//							gRobot.plantState[PLANT6].plate = 1;
						}
						if(gRobot.plantState[PLANT3].plate==0)
						{
							UART5_OUT((uint8_t*)"BLE lost Plant3");
//							gRobot.plantState[PLANT3].plate = 1;
						}
					}
				}
				else
					//如果蓝牙没有断开，则会关掉蜂鸣器
					BEEP_OFF;
				//如果左右两枪都超过自动子弹数 发一个信息 命令视觉模块看近台
				if(gRobot.leftGun.shootTimes >= LEFT_AUTO_NUMBER && gRobot.rightGun.shootTimes >= RIGHT_AUTO_NUMBER)
				{
					if(sendSignal == 1)
					{
						SendAutoOver2Camera();
						sendSignal=0;
					}
				}
				break;
			}
			/*重试*/
			case reset:
			{
				//清空计数 ResetRunRoLaunch 中将再次使用
				startLeaveCnt = 0u;
				/*等待按下重试开关 */
				if(RESET_SWITCH)
				{
					gyroAngleErr = gRobot.moveBase.actualAngle;
					//Err = 理想-实际
					gyroYErr = gRobot.moveBase.actualYPos;
					status = resetConfig;
				}
				break;
			}
			case resetConfig:
			{
				elmo_Init(CAN2);
				elmo_Enable(CAN2 , MOVEBASE_BROADCAST_ID);
				TIM_Delayms(TIM5,50);
				setLaunchPosFlag = 1;
				sendSignal2Camera = 1;
				sendSignal = 1;
				status = resetRunToLaunch;
				OSSemSet(PeriodSem, 0, &os_err);
				break;
			}
			case resetRunToLoad:
			{
				break;
			}
			case resetRunToLaunch:
			{
#ifdef RED_FIELD
				
				//				MoveTo(-6459.14f, -3000.0f, 2000.0f, 2000.0f);
				//由于重试后陀螺仪零漂较严重，矫正角度后也位置也有偏差
				MoveTo((-LAUNCH_STOP_X/cosf(ANGTORAD(gyroAngleErr)) + gyroYErr * tan(ANGTORAD(-gyroAngleErr))), -3000.0f, 2000.0f, 2000.0f);

				//离开出发区时通过光电矫正X方向坐标 红场使用左侧光电（处于行进方向前方的光电）
				if(GetPosX() > -500.0f && !PHOTOSENSORLEFT)
				{
					//有3次没有触发才记录
					if(startLeaveCnt < 3u)
					{
						startLeaveCnt++;
					}
					else if(startLeaveCnt == 3u)
					{
						//Err = Err = 实际-标准
						gyroXErr = GetPosX() - startLeaveX;
						startLeaveCnt++;
					}
				}

				//到位后给靠墙速度
				//				if (GetPosX() <= -6459.14f)
				if ((GetPosX() <= -LAUNCH_STOP_X/cosf(ANGTORAD(gyroAngleErr)) + gyroYErr * tan(ANGTORAD(-gyroAngleErr)))
				{
					MoveY(50.0f);
					moveTimFlag = 0;
					//重回stopRobot
					status = stopRobot;
				}
#endif
#ifdef BLUE_FIELD
				//				MoveTo(6459.14f, 3000.0f, 2000.0f, 2000.0f);
				MoveTo((LAUNCH_STOP_X/cosf(ANGTORAD(gyroAngleErr)) + gyroYErr * tan(ANGTORAD(-gyroAngleErr))), 3000.0f, 2000.0f, 2000.0f);
				
				//离开出发区时通过光电矫正X方向坐标 蓝场使用右侧光电（处于行进方向前方的光电）
				if(GetPosX() < 500.0f && !PHOTOSENSORRIGHT)
				{
					//有3次没有触发才记录
					if(startLeaveCnt < 3u)
					{
						startLeaveCnt++;
					}
					else if(startLeaveCnt == 3u)
					{
						//Err = 实际-标准
						gyroXErr = GetPosX() - startLeaveX;
						startLeaveCnt++;
					}
				}
				
				//到位后给靠墙速度
				//				if (GetPosX() >= 6459.14f)
				if (GetPosX() >= LAUNCH_STOP_X/cosf(ANGTORAD(gyroAngleErr)) + gyroYErr * tan(ANGTORAD(-gyroAngleErr)))
				{
					MoveY(50.0f);
					moveTimFlag = 0;
					status = stopRobot;
				}
#endif
				break;
			}
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
	//此处等待邮箱 此邮箱在walktask 中 states >= load 且行程开关触发后 立刻发送 
	OSMboxPend(OpenSaftyMbox, 0, &os_err);
#endif
	//然后延时0.2s以后 弹匣推弹收回
	OSTimeDly(20);
	LeftBack();
	//改为手动点命令后开始可能没有命令，需要转到一个姿态上第一发弹
//	ROBOT_LeftGunReturn();
//	PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(24.0f));
	PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(gLeftGunPosDatabase[SHOOT_METHOD4][PLANT3].yaw));
	PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(gLeftGunPosDatabase[SHOOT_METHOD4][PLANT3].pitch));
	PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(gLeftGunPosDatabase[SHOOT_METHOD4][PLANT3].roll));
	//再等0.5s 上第一发弹 
	OSTimeDly(50);
	LeftPush();
	//0.5s 后 气缸收回
	OSTimeDly(50);
	LeftBack();
//	OSTimeDly(20);
	//	LeftPush();
	gRobot.leftGun.noCommandTimer = 0;
	gRobot.leftGun.mode = GUN_AUTO_MODE;
	//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
	while(1)
	{
		//手自动切换时上弹气缸推到头后收回
		if(gRobot.leftGun.modeChangeFlag == 1)
		{
			gRobot.leftGun.lastPlant = INVALID_PLANT_NUMBER;
			gRobot.leftGun.lastParaMode = INVALID_SHOOT_METHOD;
			LeftBack();
			gRobot.leftGun.modeChangeFlag = 0;
		}
		//等待命令时上弹气缸推到头后收回
		if(gRobot.leftGun.noCommandTimer >= NO_COMMAND_COUNTER)
		{
			LeftBack();
		}
		//自动模式
		if(ROBOT_GunCheckMode(LEFT_GUN) == GUN_AUTO_MODE)
		{
			shoot_command_t leftGunShootCommand;
			//自获取命令
//			if(gRobot.leftGun.shootTimes < LEFT_AUTO_NUMBER)
//			{
//				leftGunShootCommand = ROBOT_LeftGunGetShootCommand();
//			}
//			else
//			{
				leftGunShootCommand = ROBOT_LeftGunGetShootCommandFIFO();
//			}
			if(gRobot.leftGun.commandState == GUN_HAVE_COMMAND)
			{
				gRobot.leftGun.noCommandTimer = 0;
				gRobot.leftGun.targetPlant = leftGunShootCommand.plantNum;
				gRobot.leftGun.nextStep = 2;
				gRobot.leftGun.shootParaMode = leftGunShootCommand.shootMethod;
				//对于7#柱子先到位后再上弹，其它柱子直接瞄准
				if(gRobot.leftGun.lastPlant == PLANT7 || leftGunShootCommand.plantNum == PLANT7 )
				{
					//获取并更新枪上弹姿态
					gRobot.leftGun.targetPose = gLeftGunReloadPosDatabase[leftGunShootCommand.shootMethod]\
												[leftGunShootCommand.plantNum];

					ROBOT_LeftGunAim();
					ROBOT_LeftGunCheckAim();
				}
				else/* if(gRobot.leftGun.shootTimes == 0)*/
				{
					gRobot.leftGun.targetPose = gLeftGunPosDatabase[leftGunShootCommand.shootMethod]\
												[leftGunShootCommand.plantNum];
					ROBOT_LeftGunAim();
				}
				//第一发弹先调整姿态一段时间后再上弹
//				if(gRobot.leftGun.shootTimes == 0 && gRobot.leftGun.champerErrerState == GUN_RELOAD_OK)
//				{
//					OSTimeDly(90);
//					LeftPush();
//				}
				//上弹
				ROBOT_LeftGunReload();

				gRobot.leftGun.targetPose = gLeftGunPosDatabase[leftGunShootCommand.shootMethod]\
											[leftGunShootCommand.plantNum];
				ROBOT_LeftGunAim();

#ifndef NO_WALK_TASK
				//第一发弹等待到位后发射，fix me 重试也需要检测
				ROBOT_LeftGunCheckShootPoint();
#endif
				//检测枪是否到位
				ROBOT_LeftGunCheckAim();
				//检查上弹是否到位
				ROBOT_LeftGunCheckReload();

				//					OSTimeDly(100);
				//落盘时，检查对应柱子是否已经打球
				if(gRobot.leftGun.shootParaMode%2)
				{
					while(gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].ballState == COMMAND_IN_PROCESS)
					{
						OSTimeDly(1);
					}
					OSTimeDly(20);
				}
				//发射
				ROBOT_LeftGunShoot();

				if(gRobot.leftGun.ready == GUN_AIM_DONE)
				{
					if(gRobot.leftGun.champerErrerState == GUN_RELOAD_OK)
					{
						//记录每个柱子的发射命令
						gRobot.plateShootTimes[gRobot.leftGun.targetPlant]+=1;
						//记录发射命令
						gRobot.leftGun.lastPlant = leftGunShootCommand.plantNum;
						gRobot.leftGun.lastParaMode = leftGunShootCommand.shootMethod;
					}
					else
					{
						if(gRobot.leftGun.shootParaMode%2)
						{
							gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].plate += 1;
						}
						else
						{
							gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].ball += 1;
						}
					}
					SetShootPlantTime(leftGunShootCommand.plantNum, leftGunShootCommand.shootMethod);
				}
				//对命令状态进行复位
				if(gRobot.leftGun.shootParaMode%2)
				{
					gRobot.manualCmdQueue.cmdPlateState&=(~(uint8_t)(0x01<<(gRobot.leftGun.targetPlant)))&0x7f;
//					CheckCmdQueueState();
					gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].plateState = COMMAND_DONE;
				}
				else
				{
					gRobot.manualCmdQueue.cmdBallState&=(~(uint8_t)(0x01<<(gRobot.leftGun.targetPlant)))&0x7f;
//					CheckCmdQueueState();
					gRobot.leftGun.gunCommand[gRobot.leftGun.targetPlant].ballState = COMMAND_DONE;
				}
			}
			else
			{
				//没有命令时回到6#落盘姿态
				OSTimeDly(6);
				ROBOT_LeftGunReturn();
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
				ROBOT_LeftGunAim();
				//更新数据库中参数并写入FLASH
				//				UpdateLeftGunPosDatabaseManualMode();
				//				FlashWriteGunPosData();
				gRobot.leftGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.leftGun.shoot==GUN_START_SHOOT)
			{
				//7#柱子需要瞄准，因为需要特殊角度上弹
				if(gRobot.leftGun.targetPlant == PLANT7)
				{
					ROBOT_LeftGunAim();
				}
				//检测左枪是否到位
				ROBOT_LeftGunCheckAim();
				//发射
				ROBOT_LeftGunShoot();

				//7#需要先到上弹角度再上弹
				if(gRobot.leftGun.targetPlant == PLANT7)
				{
					gRobot.leftGun.reloadPose = gLeftGunReloadPosDatabase[gRobot.leftGun.shootParaMode]\
												[gRobot.leftGun.targetPlant];

					ROBOT_LeftGunReloadAim();
					//检测是否到位
					ROBOT_LeftGunCheckReloadAim();
					LeftPush();
				}
				//上弹
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
			while(1)
			{
				UART5_OUT((uint8_t *)"Left Gun Mode Error!!!!!!!!!!\r\n");
			}
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
	//此处等待邮箱 此邮箱在walktask 中 states >= load 且行程开关触发后 立刻发送 
	OSMboxPend(OpenSaftyMbox, 0, &os_err);
#endif
	//然后延时0.2s以后 弹匣推弹收回
	OSTimeDly(20);
	RightBack();
	//改为手动点命令后开始可能没有命令，需要转到一个姿态上第一发弹
//	ROBOT_RightGunReturn();
//	PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(24.0f));
	PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(gRightGunPosDatabase[SHOOT_METHOD4][PLANT3].yaw));
	PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(gRightGunPosDatabase[SHOOT_METHOD4][PLANT3].pitch));
	PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(gRightGunPosDatabase[SHOOT_METHOD4][PLANT3].roll));
	//上第一发弹
	OSTimeDly(50);
	RightPush();
	OSTimeDly(50);
	RightBack();
//	OSTimeDly(20);
	gRobot.rightGun.noCommandTimer = 0;
	gRobot.rightGun.mode = GUN_AUTO_MODE;
	//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
	while(1)
	{
		//手自动切换时上弹气缸推到头后收回
		if(gRobot.rightGun.modeChangeFlag == 1)
		{
			gRobot.rightGun.lastPlant = INVALID_PLANT_NUMBER;
			gRobot.rightGun.lastParaMode = INVALID_SHOOT_METHOD;
			RightBack();
			gRobot.rightGun.modeChangeFlag = 0;
		}
		//等待命令时上弹气缸推到头后收回
		if(gRobot.rightGun.noCommandTimer >= NO_COMMAND_COUNTER)
		{
			RightBack();
		}
		//检查手动or自动
		//auto mode用在正式比赛中，平板上位机只会发送枪号和柱子号
		if(ROBOT_GunCheckMode(RIGHT_GUN) == GUN_AUTO_MODE)
		{
			shoot_command_t rightGunShootCommand;
			//获取命令
//			if(gRobot.rightGun.shootTimes < RIGHT_AUTO_NUMBER)
//			{
//				rightGunShootCommand = ROBOT_RightGunGetShootCommand();
//			}
//			else
//			{
				rightGunShootCommand = ROBOT_RightGunGetShootCommandFIFO();
//			}

			if(gRobot.rightGun.commandState == GUN_HAVE_COMMAND)
			{
				gRobot.rightGun.noCommandTimer = 0;
				gRobot.rightGun.nextStep = 2;
				gRobot.rightGun.targetPlant = rightGunShootCommand.plantNum;
				gRobot.rightGun.shootParaMode = rightGunShootCommand.shootMethod;

				//7#柱子需要到上弹姿态后再上弹，其它直接瞄准
				if(gRobot.rightGun.lastPlant == PLANT7 || rightGunShootCommand.plantNum == PLANT7)
				{
					//获取并更新枪目标姿态  上弹姿态
					gRobot.rightGun.targetPose = gRightGunReloadPosDatabase[rightGunShootCommand.shootMethod]\
												 [rightGunShootCommand.plantNum];

					//调整枪姿为上弹姿态 need some time
					ROBOT_RightGunAim();
					ROBOT_RightGunCheckAim();
				}
				else/*if(gRobot.rightGun.shootTimes == 0)*/
				{
					gRobot.rightGun.targetPose = gRightGunPosDatabase[rightGunShootCommand.shootMethod]\
												 [rightGunShootCommand.plantNum];
					ROBOT_RightGunAim();
				}
//				//第一发弹调整姿态一段时间后开始上弹
//				if(gRobot.rightGun.shootTimes == 0 && gRobot.rightGun.champerErrerState == GUN_RELOAD_OK)
//				{
//					OSTimeDly(110);
//					RightPush();
//				}
				//上弹
				ROBOT_RightGunReload();

				gRobot.rightGun.targetPose = gRightGunPosDatabase[rightGunShootCommand.shootMethod]\
											 [rightGunShootCommand.plantNum];
				ROBOT_RightGunAim();

#ifndef NO_WALK_TASK
				//第一发弹收到到位信息后发射，fix me 重试也需要检测
				ROBOT_RightGunCheckShootPoint();
#endif
				//检查是否到位
				ROBOT_RightGunCheckAim();
				//检查上弹是否到位
				ROBOT_RightGunCheckReload();

				//					OSTimeDly(100);
				//落盘时，检查对应柱子是否已经打球
				if(gRobot.rightGun.shootParaMode%2)
				{
					while(gRobot.rightGun.gunCommand[gRobot.rightGun.targetPlant].ballState == COMMAND_IN_PROCESS)
					{
						OSTimeDly(1);
					}
					OSTimeDly(20);
				}
				//发射
				ROBOT_RightGunShoot();

				if(gRobot.rightGun.ready == GUN_AIM_DONE)
				{
					if(gRobot.rightGun.champerErrerState == GUN_RELOAD_OK)
					{
						//记录每个柱子的发射命令
						gRobot.plateShootTimes[gRobot.rightGun.targetPlant]+=1;
						//记录发射命令
						gRobot.rightGun.lastPlant = rightGunShootCommand.plantNum;
						gRobot.rightGun.lastParaMode = rightGunShootCommand.shootMethod;
					}
					else
					{
						if(gRobot.rightGun.shootParaMode%2)
						{
							gRobot.rightGun.gunCommand[gRobot.rightGun.targetPlant].plate += 1;
						}
						else
						{
							gRobot.rightGun.gunCommand[gRobot.rightGun.targetPlant].ball += 1;
						}
					}
					SetShootPlantTime(rightGunShootCommand.plantNum, rightGunShootCommand.shootMethod);
				}
				//对命令状态进行复位
				if(gRobot.rightGun.shootParaMode%2)
				{
					gRobot.manualCmdQueue.cmdPlateState&=(~((uint8_t)0x01<<(gRobot.rightGun.targetPlant)))&0x7f;
//					CheckCmdQueueState();
					gRobot.rightGun.gunCommand[gRobot.rightGun.targetPlant].plateState = COMMAND_DONE;
				}
				else
				{
					gRobot.manualCmdQueue.cmdBallState&=(~((uint8_t)0x01<<(gRobot.rightGun.targetPlant)))&0x7f;
//					CheckCmdQueueState();
					gRobot.rightGun.gunCommand[gRobot.rightGun.targetPlant].ballState = COMMAND_DONE;
				}
			}
			else
			{
				//没有命令时回到6#落盘姿态
				OSTimeDly(6);
				ROBOT_RightGunReturn();
			}

		}
		//手动模式用于调试过程中，对端设备只会发送枪号和着陆号，枪的姿态
		//调试过程中着陆台信息没有用，根据shoot标志来开枪
		else if(ROBOT_GunCheckMode(RIGHT_GUN) == GUN_MANUAL_MODE)
		{
			gRobot.rightGun.nextStep = 3;
			if(gRobot.rightGun.aim == GUN_START_AIM)
			{
				//获得目标位姿，这里应该由对端设备发送过来，直接更新的gRobot.rightGun中的目标位姿
				ROBOT_RightGunAim();
				//更新数据库中参数并写入FLASH
				//				UpdateRightGunPosDatabaseManualMode();
				//				FlashWriteGunPosData();
				gRobot.rightGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.rightGun.shoot==GUN_START_SHOOT)
			{
				//7#柱子需要瞄准，因为会到上弹角度上弹
				if(gRobot.rightGun.targetPlant == PLANT7)
				{
					ROBOT_RightGunAim();
				}
				//检查是否到位
				ROBOT_RightGunCheckAim();
				//发射
				ROBOT_RightGunShoot();
				//7#柱子到上弹姿态
				if(gRobot.rightGun.targetPlant == PLANT7)
				{
					gRobot.rightGun.reloadPose = gRightGunReloadPosDatabase[gRobot.rightGun.shootParaMode]\
												 [gRobot.rightGun.targetPlant];

					ROBOT_RightGunReloadAim();
					//检查是否到位
					ROBOT_RightGunCheckReloadAim();
					RightPush();
				}
				//上弹
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
			while(1)
			{
				UART5_OUT((uint8_t *)"Right Gun Mode Error!!!!!!!!\r\n");
			}
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
#ifdef NO_WALK_TASK
	gRobot.upperGun.gunCommand = gRobot.plantState;
	gRobot.upperGun.mode = GUN_DEFEND_MODE;
#endif
	while(1)
	{
#ifndef NO_WALK_TASK
		ROBOT_UpperGunCheckShootPoint();
#endif
		//如果接收到防守命令进入防守模式
		if(gRobot.upperGun.defendZone1 & 0x0f)
		{
			gRobot.upperGun.mode = GUN_DEFEND_MODE;
		}
#ifndef NO_WALK_TASK
		//当蓝牙中断时不需要防守时如果7#需要落盘则对7#落盘命令置位，通信正常时由平板控制7#落盘时机
		else if(gRobot.upperGun.isSelfEmpty == SELF_EMPTY && gRobot.isBleOk.noBleFlag == BLE_LOST)
		{
			if(gRobot.plantState[PLANT7].plateState == COMMAND_DONE)
			{
				//等待2.0s避免已经发射弹盘没落上时重复发射
				uint8_t checkGap = 40;
				while(checkGap--)
				{
					/*等待过程中如果视觉模块告知需要防守 进入防守模式*/
					if(gRobot.upperGun.defendZone1 & 0x0f)
					{
						break;
					}
					OSTimeDly(5);
				}
				//如果进入防守模式则不补7#
				if((gRobot.upperGun.defendZone1 & 0x0f) == 0x00)
				{
					//对7#落盘命令进行置位
					if(gRobot.upperGun.isSelfEmpty == SELF_EMPTY)
					{
						uint8_t putPlateFlag = 1;
						cmd_t selfCmd = {INVALID_PLANT_NUMBER , INVALID_SHOOT_METHOD};
						gRobot.plantState[PLANT7].plate = 1;
						//搜索队列中是否有7#落盘命令
						if (gRobot.manualCmdQueue.headNum != gRobot.manualCmdQueue.tailNum)
						{
							for(uint8_t i = gRobot.manualCmdQueue.headNum;i<gRobot.manualCmdQueue.headNum + gRobot.manualCmdQueue.elementNum;i++)
							{
								uint8_t counter = 0;
								counter = i%CMD_QUEUE_LENGTH;
								if(gRobot.manualCmdQueue.cmdArr[counter].plantNum == PLANT7 && \
									gRobot.manualCmdQueue.cmdArr[counter].method%2)
								{
									//队列中有7#落盘命令时不再重复入队
									putPlateFlag = 0;
								}
							}
						}
						//正在执行7#落盘命令时
						if(gRobot.plantState[PLANT7].plateState == COMMAND_IN_PROCESS)
						{
							putPlateFlag = 0;
						}
						//如果队列中没有7#落盘命令时在队列中加入7#落盘命令
						if(putPlateFlag==1)
						{
							selfCmd.plantNum = PLANT7;
							selfCmd.method = SHOOT_METHOD4;
							//为下枪补7# 入队
							InCmdQueue(selfCmd);
						}
					}
				}
				else
				{
					gRobot.upperGun.mode = GUN_DEFEND_MODE;
				}
			}
		}
#endif
		//检查手动or自动
		//auto mode用在正式比赛中，与左右两枪不同，通过摄像头的反馈发射飞盘
		if(ROBOT_GunCheckMode(UPPER_GUN) == GUN_DEFEND_MODE)
		{
			//fix me,此处应该检查目标区域是否合法
			if(gRobot.upperGun.defendZone1 & 0x0f)
			{
				upperGunShootFlag = 1;
			}
			
			if(upperGunShootFlag == 1)
			{
				gRobot.upperGun.targetPlant = PLANT7;
				gRobot.upperGun.shootParaMode = SHOOT_METHOD3;
				//主副防守区Id
				int mainZoneId = INVALID_ZONE_NUMBER;
				int viceZoneId = INVALID_ZONE_NUMBER;
				
				//7号台敌方落盘状况枚举：有一个盘，有两个以上的盘
				typedef enum
				{
					//一个盘
					one,
					//两个盘以上
					twoAndMore,
				}DiskNum_t;
				DiskNum_t diskNum = one;
				
				//主防守区
				if (gRobot.upperGun.defendZone1 >= 0x01 && gRobot.upperGun.defendZone1 <= 0x06)
				{
					mainZoneId = gRobot.upperGun.defendZone1 - 0x01;
				}
				else
				{
					OSTimeDly(2);
					continue;
				}
				//副防守区
				if (gRobot.upperGun.defendZone2 == 0x00)
				{
					diskNum = one;
				}
				else if (gRobot.upperGun.defendZone2 >= 0x01 && gRobot.upperGun.defendZone2 <= 0x06)
				{
					viceZoneId = gRobot.upperGun.defendZone2 - 0x01;
					diskNum = twoAndMore;
				}
				else
				{
					//如果发过来其他值 传输可能错误
					OSTimeDly(2);
					continue;
				}
				
				//当前防守分区为主防守分区
				gRobot.upperGun.presentDefendZoneId = mainZoneId;
				
				//如果台上敌盘数为2+，且和上次射击位置相同（无需CheckAim），延时700ms
				if (gRobot.upperGun.presentDefendZoneId == gRobot.upperGun.lastDefendZoneId &&
					gRobot.upperGun.lastDefendZoneId != INVALID_ZONE_NUMBER)
				{
					OSTimeDly(70);
					gRobot.upperGun.lastDefendZoneId = INVALID_ZONE_NUMBER;
				}
				
				//获取目标位姿
				gun_pose_t pose = gUpperGunPosDatabase[gRobot.upperGun.targetPlant][gRobot.upperGun.shootParaMode][mainZoneId];
				//fix me,这里存在的风险是，自动过程中，手动修改柱子命令，这时候有可能结果不一致，要改

				//更新枪目标位姿
				gRobot.upperGun.targetPose.pitch = pose.pitch;
				gRobot.upperGun.targetPose.yaw = pose.yaw;
				gRobot.upperGun.targetPose.speed1 = pose.speed1;

				//瞄准，此函数最好瞄准完成后再返回
				ROBOT_UpperGunAim();
				ROBOT_UpperGunCheckAim();
				
				//gRobot.upperGun.shoot 在 ROBOT_UpperGunCheckAim() 中置位
				if (gRobot.upperGun.shoot == GUN_START_SHOOT)
				{
					//发射
					ROBOT_UpperGunShoot();
					//标志位复位
					gRobot.upperGun.shoot = GUN_STOP_SHOOT;
					//记录所打区域
					gRobot.upperGun.lastDefendZoneId = gRobot.upperGun.presentDefendZoneId;
					
					//清除defendZone对应位
					gRobot.upperGun.defendZone1 = NO_ENEMY_DISK;
					
					mainZoneId = INVALID_ZONE_NUMBER;
					upperGunShootFlag = 0;
					
					if(gRobot.upperGun.isManualDefend == UPPER_MANUAL_DEFEND)
					{
						gRobot.upperGun.isManualDefend = UPPER_AUTO_DEFEND;
					}
					
					OSTimeDly(10);
				}
				else if (gRobot.upperGun.shoot == GUN_STOP_SHOOT)
				{
					//防止不断进入循环占用大量CPU
					OSTimeDly(2);
					continue;
				}
				
				//分两种情况：7号台一个敌盘，两个及以上敌盘
				if (diskNum == one)
				{
					//如果仅一个盘 等待0.5s 但期间检测是否再落上盘
					for (int i = 0; i < 10; i++)
					{
						if (gRobot.upperGun.defendZone2 != NO_ENEMY_DISK)
						{
							diskNum = twoAndMore;
							viceZoneId = gRobot.upperGun.defendZone2 - 0x01;
							break;
						}
						OSTimeDly(5);
					}
					gRobot.upperGun.lastDefendZoneId = INVALID_ZONE_NUMBER;
				}

				if (diskNum == twoAndMore)
				{
					//当前防守分区为副防守分区
					gRobot.upperGun.presentDefendZoneId = viceZoneId;
					
					//如果台上敌盘数为2+，且和上次射击位置相同（无需CheckAim），延时700ms
					if (gRobot.upperGun.presentDefendZoneId == gRobot.upperGun.lastDefendZoneId &&
						gRobot.upperGun.lastDefendZoneId != INVALID_ZONE_NUMBER)
					{
						OSTimeDly(70);
						gRobot.upperGun.lastDefendZoneId = INVALID_ZONE_NUMBER;
					}
					
					//获取目标位姿
					pose = gUpperGunPosDatabase[gRobot.upperGun.targetPlant][gRobot.upperGun.shootParaMode][viceZoneId];
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
						gRobot.upperGun.lastDefendZoneId = gRobot.upperGun.presentDefendZoneId;
						gRobot.upperGun.defendZone2 = NO_ENEMY_DISK;
						viceZoneId = INVALID_ZONE_NUMBER;
						upperGunShootFlag = 0;
						OSTimeDly(10);
					}
				}
				
				if(gRobot.upperGun.defendZone1 == NO_ENEMY_DISK)
				{
					upperGunShootFlag = 0;
				}
			}
			else
			{
				gRobot.upperGun.mode = GUN_ATTACK_MODE;
				//				OSTaskSuspend(OS_PRIO_SELF);
			}
		}
		else if(ROBOT_GunCheckMode(UPPER_GUN) == GUN_ATTACK_MODE)
		{
			//获取命令
			shoot_command_t uppershootCommand = ROBOT_UpperGunGetShootCommand();
			gRobot.upperGun.targetPlant = uppershootCommand.plantNum;
			gRobot.upperGun.shootParaMode = uppershootCommand.shootMethod;
			if(gRobot.upperGun.commandState == GUN_HAVE_COMMAND)
			{
				uint8_t targetPlant = uppershootCommand.plantNum;
				uint8_t shootMethod = uppershootCommand.shootMethod;
				uint8_t shootZone = ZONE1;
				//获取目标位姿
				gun_pose_t pose = gUpperGunPosDatabase[targetPlant][shootMethod][shootZone];
				//更新枪目标位姿
				gRobot.upperGun.targetPose.pitch = pose.pitch;
				gRobot.upperGun.targetPose.yaw = pose.yaw;
				gRobot.upperGun.targetPose.speed1 = pose.speed1;

				ROBOT_UpperGunAim();
				//检查是否到位
				ROBOT_UpperGunCheckAim();
				if(gRobot.upperGun.defendZone1 & 0x0f)
				{
					gRobot.upperGun.mode = GUN_DEFEND_MODE;
					//执行命令过程中若切到防守模式将命令状态复位
					if(gRobot.upperGun.shootParaMode%3)
					{
						gRobot.upperGun.gunCommand[gRobot.upperGun.targetPlant].plate += 1;
						gRobot.upperGun.gunCommand[gRobot.upperGun.targetPlant].plateState = COMMAND_DONE;
					}
					else
					{
						gRobot.upperGun.gunCommand[gRobot.upperGun.targetPlant].ball += 1;
						gRobot.upperGun.gunCommand[gRobot.upperGun.targetPlant].ballState = COMMAND_DONE;
					}
				}
				else
				{
					ROBOT_UpperGunShoot();
					gRobot.upperGun.lastPlant = gRobot.upperGun.targetPlant;
					gRobot.upperGun.lastParaMode = gRobot.upperGun.shootParaMode;
					SetShootPlantTime(uppershootCommand.plantNum, uppershootCommand.shootMethod);
					//发射完成后标志任务执行完成
					if(gRobot.upperGun.shootParaMode%3)
					{
						gRobot.upperGun.gunCommand[gRobot.upperGun.targetPlant].plateState = COMMAND_DONE;
					}
					else
					{
						gRobot.upperGun.gunCommand[gRobot.upperGun.targetPlant].ballState = COMMAND_DONE;
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
			while(1)
			{
				UART5_OUT((uint8_t *)"Upper Gun Mode Error!!!!!!!!!\r\n");
			}
		}
		UpperGunSendDebugInfo();
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
		UART5_OUT((uint8_t *)"CPU%d\r\n", OSCPUUsage);
		//			GPIO_ResetBits(GPIOE, GPIO_Pin_2);
		//			OSTimeDly(10);
		//			GPIO_SetBits(GPIOE, GPIO_Pin_2);
		//			if(gRobot.autoCommand[PLANT1].ball == 0&&\
		//				gRobot.autoCommand[PLANT2].ball == 0 &&\
		//				gRobot.autoCommand[PLANT1].plate == 0 &&\
		//				gRobot.autoCommand[PLANT2].plate == 0)
		//			{
		//				for(uint8_t i = 0; i < 7;i++)
		//				{
		//					gRobot.autoCommand[i].ball = 1;
		//				}
		//				for(uint8_t i = 0; i < 7;i++)
		//				{
		//					gRobot.autoCommand[i].plate = 1;
		//				}
		//			}
		//			if(PHOTOSENSORUPGUN)
		//			{
		//				ClampClose();
		//			}
		//			ReadActualPos(CAN1,RIGHT_GUN_GROUP_ID);
		//			ReadActualVel(CAN1,RIGHT_GUN_VEL_GROUP_ID);
		//			ReadActualPos(CAN1,LEFT_GUN_GROUP_ID);
		//			ReadActualVel(CAN1,LEFT_GUN_VEL_GROUP_ID);
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

