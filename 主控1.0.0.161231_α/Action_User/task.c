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
#include "dma.h"
//#define RED_FIELD
#define BLUE_FIELD
//#define NO_WALK_TASK
#define NO_COMMAND_COUNTER 300
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
	
	u5_printf((char *)"%d\t%d\t%d\t%d\t%d\t",KEYSWITCH,status,\
			(int)gRobot.moveBase.actualAngle,(int)gRobot.moveBase.actualXPos,\
			(int)gRobot.moveBase.actualYPos);

	u5_printf((char *)"%d\t%d\t%d\t",(int)gRobot.moveBase.targetSpeed.leftWheelSpeed,\
			(int)gRobot.moveBase.targetSpeed.forwardWheelSpeed,(int)gRobot.moveBase.targetSpeed.backwardWheelSpeed);			

	u5_printf((char *)"%d\t%d\t%d\t",(int)gRobot.moveBase.actualSpeed.leftWheelSpeed,\
			(int)gRobot.moveBase.actualSpeed.forwardWheelSpeed,(int)gRobot.moveBase.actualSpeed.backwardWheelSpeed);			

//	u5_printf((char *)"%d\t%d\t%d\t%d\t",(int)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[0],\
//			(int)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[1],\
//			(int)(int8_t)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[2],\
//			(int)(int8_t)gRobot.moveBase.motorFailure.forwardMotorFailure.failureInfo[3]);
			
	u5_printf((char *)"%d\t%d\t%d\t",(int)gRobot.moveBase.acturalCurrent.leftWheelCurrent,\
			(int)gRobot.moveBase.acturalCurrent.forwardWheelCurrent,(int)gRobot.moveBase.acturalCurrent.backwardWheelCurrent);			

//	UART5_OUT((uint8_t *)"%d\t%d\t%d\t",(int)gRobot.moveBase.driverTemperature.leftWheelDriverTemperature,\
//			(int)gRobot.moveBase.driverTemperature.forwardWheelDrvierTemperature,(int)gRobot.moveBase.driverTemperature.backwardWheelDriverTemperature);			

//	u5_printf((char *)"%d\t%d\t%d\t",(int)gRobot.moveBase.driverCurrentLimitFlag.leftWheelDriverFlag,\
//			(int)gRobot.moveBase.driverCurrentLimitFlag.forwardWheelDriverFlag,(int)gRobot.moveBase.driverCurrentLimitFlag.backwardWheelDriverFlag);			

	u5_printf((char *)"%d\t%d\t%d\t",(int)gRobot.moveBase.driverCommandVelocity.leftDriverCommandVelocity,\
			(int)gRobot.moveBase.driverCommandVelocity.forwardDriverCommandVelocity,(int)gRobot.moveBase.driverCommandVelocity.backwardDriverCommandVelocity);			

//	UART5_OUT((uint8_t *)"%d\t%d\t%d\t",(int)gRobot.moveBase.driverCommandVelocity.leftDriverCommandVelocity,\
//			(int)gRobot.moveBase.driverCommandVelocity.forwardDriverCommandVelocity,(int)gRobot.moveBase.driverCommandVelocity.backwardDriverCommandVelocity);			

	u5_printf((char *)"%d\t%d\t%d\t",(int)gRobot.moveBase.driverJoggingVelocity.leftDriverJoggingVelocity,\
			(int)gRobot.moveBase.driverJoggingVelocity.forwardDriverJoggingVelocity,(int)gRobot.moveBase.driverJoggingVelocity.backwardDriverJoggingVelocity);			
		
	u5_printf((char *)"%d",(int)(gRobot.moveBase.actualKenimaticInfo.vt*0.1f));

	UART5BufPut('\r');
	UART5BufPut('\n');
}
void LeftGunSendDebugInfo(void)
{
	UART5_OUT((uint8_t *)"l\t%d\t%d\t%d\t",(int)gRobot.leftGun.checkTimeUsage,\
		(int)gRobot.leftGun.targetPlant,(int) gRobot.leftGun.shootParaMode);
	
	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.yaw*10.0f),\
		(int)(gRobot.leftGun.actualPose.yaw*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.pitch*10.0f),\
		(int)(gRobot.leftGun.actualPose.pitch*10.0f));

	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.roll*10.0f),\
		(int)(gRobot.leftGun.actualPose.roll*10.0f));
	
	UART5_OUT((uint8_t *)"%d\t%d\t",(int)(gRobot.leftGun.targetPose.speed1),\
		(int)(gRobot.leftGun.actualPose.speed1));
	
	UART5_OUT((uint8_t *)"%d\t%d",(int)(gRobot.leftGun.targetPose.speed2),\
		(int)(gRobot.leftGun.actualPose.speed2));	
	
	UART5BufPut('\r');
	UART5BufPut('\n');
	
}
void RightGunSendDebugInfo(void)
{
	UART5_OUT((uint8_t *)"r\t%d\t%d\t%d\t",(int)gRobot.rightGun.checkTimeUsage,\
		(int)gRobot.rightGun.targetPlant,(int) gRobot.rightGun.shootParaMode);
	
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
	
	UART5BufPut('\r');
	UART5BufPut('\n');

}
void UpperGunSendDebugInfo(void)
{
	UART5_OUT((uint8_t *)"u\t%d\t%d\t%d\t",(int)gRobot.upperGun.checkTimeUsage,\
		(int)gRobot.upperGun.targetPlant,(int) gRobot.upperGun.shootParaMode);
	
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
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;          /*防止警告...*/

	/*创建信号量*/
    PeriodSem				=	OSSemCreate(0);
	DebugPeriodSem          =   OSSemCreate(0);
	GyroSem                 =	OSSemCreate(0);
	
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
	os_err = OSTaskCreate(	(void (*)(void *)) SelfCheckTask,
							(void          * ) 0,
													(OS_STK        * )&SelfCheckTaskStk[SELFCHECK_TASK_STK_SIZE-1],
													(INT8U           ) SELFCHECK_TASK_PRIO);
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
//	UART5_Init(115200);		//调试用wifi
	UART5DMAInit();
	//******************
//	USART3_Init(115200);    //摄像头
	//*******************
	USART6_Init(115200);	//定位系统
//	FlashInit();
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

	TIM_Delayms(TIM5, 17000);
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
/*
如果行程开关触发  挂起所有枪 走形任务 
进入自检任务
*/
#ifndef NO_WALK_TASK
	if(KEYSWITCH==1)
	{
		BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
		BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
		BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);	
		
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
#endif
#ifdef NO_WALK_TASK
	OSTaskDel(SELFCHECK_TASK_PRIO);
#endif
//	LeftHold();
//	RightHold();
//////测试使用！！！！！！！！！！！！！！！！
//	MoveY(50.0f);
#ifdef NO_WALK_TASK
	while(!PHOTOSENSORUPGUN)
	{
		//WAIT
	}
//	MoveY(50);
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
	while(1)
	{
	switch(status_check)
	{
		case wheelSpeedCheck:
			
		//正转1s
			ThreeWheelVelControlSelfCheck(1);
			TIM_Delayms(TIM5, 1000);
		//反转1s
			ThreeWheelVelControlSelfCheck(2);
			TIM_Delayms(TIM5, 1000);
		//停止1s
			ThreeWheelVelControlSelfCheck(3);
			TIM_Delayms(TIM5, 1000);
		
			status_check++;
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);	
			break;
		case gpsCheck:
			GyroInit();
			Sendfloat(gRobot.moveBase.actualXPos);
			Sendfloat(gRobot.moveBase.actualYPos);
			Sendfloat(gRobot.moveBase.actualAngle);
			USART_SendData(UART5,(uint8_t)-100);		
			USART_SendData(UART5,(uint8_t)-100);
			USART_SendData(UART5,(uint8_t)-100);
			USART_SendData(UART5,(uint8_t)-100);
			
		
			if(KEYSWITCH==1)
			{
				status_check++;
				BEEP_ON;TIM_Delayms(TIM5, 1000);BEEP_OFF;TIM_Delayms(TIM5, 100);
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
				BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);	
			}
			break;
		case gunCheck:
			//夹子开			
			 ClampOpen();TIM_Delayms(TIM5, 1000);

			//夹子关
			 ClampClose();TIM_Delayms(TIM5, 1000);

//			//夹子翻
//			 ClampRotate();TIM_Delayms(TIM5, 100);

//			//夹子翻转复位
//			 ClampReset();TIM_Delayms(TIM5, 100);

	    LeftPush();TIM_Delayms(TIM5, 500);LeftBack();TIM_Delayms(TIM5, 500);
	    RightPush();TIM_Delayms(TIM5, 500);RightBack();TIM_Delayms(TIM5, 500);		
		LeftShoot();TIM_Delayms(TIM5, 500);LeftShootReset();TIM_Delayms(TIM5, 500);
		RightShoot();TIM_Delayms(TIM5, 500); RightShootReset();TIM_Delayms(TIM5, 500);
		UpperShoot();TIM_Delayms(TIM5, 500);  UpperShootReset();TIM_Delayms(TIM5, 500);
				
		/************下枪左*********/
		VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(50.0f));
		VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(50.0f));

		//横滚向一侧一定角度
		PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(20.0f));TIM_Delayms(TIM5, 1000);	
		//横滚向另一侧一定角度
		PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(-20.0f));TIM_Delayms(TIM5, 1000);
		PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(0.0f));


		
		//左转一定角度
		PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(20.0f));TIM_Delayms(TIM5, 1000);
		//右转一定角度
		PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(-20.0f));TIM_Delayms(TIM5, 1000);
		PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(0.0f));
		
		//俯仰向上一定角度
		PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(40.0f));TIM_Delayms(TIM5, 1000);			
		//俯仰向下一定角度
		PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(10.0f));TIM_Delayms(TIM5, 1000);	
		PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(40.0f));
		
		VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(0.0f));
		VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(0.0f));
		
		/************下枪右*********/
		VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(50.0f));
		VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(50.0f));

		//横滚向一侧一定角度
		PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(20.0f));TIM_Delayms(TIM5, 1000);	
		
		//横滚向另一侧一定角度
		PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(-20.0f));TIM_Delayms(TIM5, 1000);	
		PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(0.0f));	
		
		//左转一定角度
		PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(20.0f));TIM_Delayms(TIM5, 1000);
		//右转一定角度
		PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(-20.0f));TIM_Delayms(TIM5, 1000);
		PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(0.0f));
		
		//俯仰向上一定角度
		PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(40.0f));TIM_Delayms(TIM5, 1000);			

		//俯仰向下一定角度
		PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(10.0f));TIM_Delayms(TIM5, 1000);		
		PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(40.0f));				


		VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(0.0f));
		VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(0.0f));
		/************上枪*********/
		VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(50.0f));

		//左转一定角度
		PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(20.0f));TIM_Delayms(TIM5, 1000);
		//右转一定角度
		PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(-20.0f));TIM_Delayms(TIM5, 1000);
		PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(0.0f));
		
		//俯仰向上一定角度 起始角度
		PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(20.0f));TIM_Delayms(TIM5, 1000);				
		//俯仰向下一定角度	
		PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(-8.0f));TIM_Delayms(TIM5, 1000);
		PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(0.0f));
		
		
		VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(0.0f));

		status_check++;
		BEEP_ON;TIM_Delayms(TIM5, 1000);BEEP_OFF;TIM_Delayms(TIM5, 100);
		BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);
		BEEP_ON;TIM_Delayms(TIM5, 100);BEEP_OFF;TIM_Delayms(TIM5, 100);	
		SendStop2Camera();
			break;
		case cameraCheck:
			
			USART_SendData(UART5,(uint8_t)gRobot.upperGun.targetZone);
			USART_SendData(UART5,(uint8_t)-100);		
			USART_SendData(UART5,(uint8_t)-100);
			USART_SendData(UART5,(uint8_t)-100);
			USART_SendData(UART5,(uint8_t)-100);
		
			Sendfloat(gRobot.moveBase.actualYPos);
			if(KEYSWITCH==1)
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
	
			if(PHOTOSENSORLEFT ||PHOTOSENSORRIGHT||PHOTOSENSORUPGUN)
			{
				GPIOE->ODR^=0X80;
				TIM_Delayms(TIM5, 20);
			}
			if(KEYSWITCH==1)
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
	static uint16_t timeCounter = 0;
	CPU_INT08U  os_err;
	os_err = os_err;	
	int shootPointMsg = MOVEBASE_POS_READY;
    OSSemSet(PeriodSem, 0, &os_err);
	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		if(status >= load)
		{
			ROBOT_CheckGunOpenSafety();
		}
		ReadActualVel(CAN2, MOVEBASE_BROADCAST_ID);
		ReadActualCurrent(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadActualTemperature(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadCurrentLimitFlag(CAN2, MOVEBASE_BROADCAST_ID);
//		ReadVelocityError(CAN2, MOVEBASE_BROADCAST_ID);
		ReadCommandVelocity(CAN2, MOVEBASE_BROADCAST_ID);
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
					GyroInit();
					//等待定位系统信号量
					OSSemSet(GyroSem, 0, &os_err);
					OSSemPend(GyroSem,200, &os_err);
					if(os_err == OS_ERR_TIMEOUT)
					{
						while(1)
						{
							USART_SendData(UART5 ,(uint8_t)99);
							USART_SendData(UART5 ,(uint8_t)66);
							USART_SendData(UART5 ,(uint8_t)99);
							USART_SendData(UART5 ,(uint8_t)66);
							USART_SendData(UART5 ,(uint8_t)-100);
							USART_SendData(UART5 ,(uint8_t)-100);
							USART_SendData(UART5 ,(uint8_t)-100);
							USART_SendData(UART5 ,(uint8_t)-100);							
							BEEP_ON;
							TIM_Delayms(TIM5,500);
							BEEP_OFF;
							TIM_Delayms(TIM5,500);	
						}
					}
					OSSemSet(PeriodSem, 0, &os_err);
					ClampOpen();
					TIM_Delayms(TIM5,20);
					ROBOT_LeftGunHome();
					ROBOT_RightGunHome();
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
				MoveToCenter(13023.14f, 3500.0f, 2000.0f);		
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
					status=goToLaunchingArea;
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
					MoveY(50.0f);
					moveTimFlag = 0;
					status++;
				}
#endif
#ifdef BLUE_FIELD
                MoveToCenter(6459.14f, -3000.0f, 2000.0f);
			    if (GetPosX() <= 6459.14f)
				{
					ClampReset();
					MoveY(50.0f);
					moveTimFlag = 0;
					status++;
				}
#endif
				break;
			
			//发射飞盘
			case launch:
				OSTimeDly(50);
				LockWheel();
				OSMboxPostOpt(LeftGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);
				OSMboxPostOpt(RightGunShootPointMbox , &shootPointMsg , OS_POST_OPT_NONE);		
				OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
//				CameraInit();
//				MoveY(50.0f);
				SendStop2Camera();
				
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
//	OSTimeDly(50);
//	LeftPush();
	gRobot.leftGun.noCommandTimer = 0;
	gRobot.leftGun.mode = GUN_AUTO_MODE;
	//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
	while(1)
	{
		//手自动切换时上弹气缸推到头后收回
		if(gRobot.leftGun.modeChangeFlag == 1)
		{
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

//			if(gRobot.leftGun.shootTimes >=   ROBOT_LeftGunPoint1ShootTimes() + ROBOT_LeftGunPoint3ShootTimes())
//			{
				//自动调度
			//

				shoot_command_t leftGunShootCommand = ROBOT_LeftGunGetShootCommand();

				if(gRobot.leftGun.commandState == GUN_HAVE_COMMAND)
				{
					gRobot.leftGun.noCommandTimer = 0;
					gRobot.leftGun.targetPlant = leftGunShootCommand.plantNum;
					gRobot.leftGun.nextStep = 2;
					gRobot.leftGun.shootParaMode = leftGunShootCommand.shootMethod;
					
					if(gRobot.leftGun.lastPlant == PLANT7 || leftGunShootCommand.plantNum == PLANT3 )
					{
						//获取并更新枪目标姿态  上弹姿态
						gRobot.leftGun.targetPose = gLeftGunReloadPosDatabase[leftGunShootCommand.shootMethod]\
																			[leftGunShootCommand.plantNum];

						ROBOT_LeftGunAim();
						ROBOT_LeftGunCheckAim();
					}
					else
					{
						gRobot.leftGun.targetPose = gLeftGunPosDatabase[leftGunShootCommand.shootMethod]\
																				[leftGunShootCommand.plantNum];
						ROBOT_LeftGunAim();
					}
					gRobot.leftGun.targetPose = gLeftGunPosDatabase[leftGunShootCommand.shootMethod]\
																		[leftGunShootCommand.plantNum];

					ROBOT_LeftGunAim();	
					if(gRobot.leftGun.shootTimes == 0)
					{
						OSTimeDly(30);
						LeftPush();
					}
				
					ROBOT_LeftGunReload();				
					
					//检查上弹是否到位
//					ROBOT_LeftGunCheckReload();
					//获取并更新枪目标姿态  发射姿态

					if(leftGunShootCommand.plantNum == PLANT6)		//PLANT6不检查姿态
					{
						ROBOT_LeftGunCheckAim();
//						OSTimeDly(200);
//						gRobot.leftGun.ready = GUN_AIM_DONE;
					}
					else
					{
							ROBOT_LeftGunCheckAim();
//							OSTimeDly(200);
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
					gRobot.leftGun.lastPlant = leftGunShootCommand.plantNum;
					gRobot.leftGun.lastParaMode = leftGunShootCommand.shootMethod;
					if(gRobot.leftGun.shootParaMode == SHOOT_METHOD2 ||gRobot.leftGun.shootParaMode == SHOOT_METHOD4)
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
					OSTimeDly(6);
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
				gRobot.leftGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.leftGun.shoot==GUN_START_SHOOT)
			{
				if(gRobot.leftGun.targetPlant == PLANT7)
				{
						ROBOT_LeftGunAim();
				}
				ROBOT_LeftGunCheckAim();
				ROBOT_LeftGunShoot();
				if(gRobot.leftGun.targetPlant == PLANT7)
				{
					gRobot.leftGun.reloadPose = gLeftGunReloadPosDatabase[gRobot.leftGun.shootParaMode]\
																		[gRobot.leftGun.targetPlant];

					ROBOT_LeftGunReloadAim();
					ROBOT_LeftGunCheckReloadAim();
					LeftPush();
				}
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
//		LeftGunSendDebugInfo();
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
//	OSTimeDly(50);
//	RightPush();
	gRobot.rightGun.noCommandTimer = 0;
	gRobot.rightGun.mode = GUN_AUTO_MODE;
	//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
	while(1)
	{
		//手自动切换时上弹气缸推到头后收回
		if(gRobot.rightGun.modeChangeFlag == 1)
		{
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
//			if(gRobot.rightGun.shootTimes >= ROBOT_RightGunPoint1ShootTimes() + ROBOT_RightGunPoint3ShootTimes())
//			{


				shoot_command_t rightGunShootCommand = ROBOT_RightGunGetShootCommand();

				if(gRobot.rightGun.commandState == GUN_HAVE_COMMAND)
				{
					gRobot.rightGun.noCommandTimer = 0;
					gRobot.rightGun.nextStep = 2;
					gRobot.rightGun.targetPlant = rightGunShootCommand.plantNum;
					gRobot.rightGun.shootParaMode = rightGunShootCommand.shootMethod;

					if(gRobot.rightGun.lastPlant == PLANT7 || rightGunShootCommand.plantNum == PLANT3)
					{
						//获取并更新枪目标姿态  上弹姿态
						gRobot.rightGun.targetPose = gRightGunReloadPosDatabase[rightGunShootCommand.shootMethod]\
																			[rightGunShootCommand.plantNum];

						//调整枪姿为上弹姿态 need some time
						ROBOT_RightGunAim();
						ROBOT_RightGunCheckAim();
					}
					else
					{
						gRobot.rightGun.targetPose = gRightGunPosDatabase[rightGunShootCommand.shootMethod]\
																		[rightGunShootCommand.plantNum];
						ROBOT_RightGunAim();
					}
					if(gRobot.rightGun.shootTimes == 0)
					{
						OSTimeDly(30);
						RightPush();
					}
					gRobot.rightGun.targetPose = gRightGunPosDatabase[rightGunShootCommand.shootMethod]\
																		[rightGunShootCommand.plantNum];

					//调整枪姿为发射姿态 need some time
					ROBOT_RightGunAim();
					ROBOT_RightGunReload();				

					//检查上弹是否到位
//					ROBOT_RightGunCheckReload();
					//瞄准，此函数最好瞄准完成后再返回

					//获取并更新枪目标姿态  发射姿态

					if(rightGunShootCommand.plantNum == PLANT6)
					{
						ROBOT_RightGunCheckAim();
//						OSTimeDly(200);
//						gRobot.rightGun.ready = GUN_AIM_DONE;
					}
					else
					{
						ROBOT_RightGunCheckAim();
//						OSTimeDly(200);					
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
					gRobot.rightGun.lastPlant = rightGunShootCommand.plantNum;
					gRobot.rightGun.lastParaMode = rightGunShootCommand.shootMethod;
					if(gRobot.rightGun.shootParaMode == SHOOT_METHOD2 ||gRobot.rightGun.shootParaMode == SHOOT_METHOD4)
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
					OSTimeDly(6);
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
				if(gRobot.rightGun.targetPlant == PLANT7)
				{
					ROBOT_RightGunAim();
				}
				ROBOT_RightGunCheckAim();
				ROBOT_RightGunShoot();
				if(gRobot.rightGun.targetPlant == PLANT7)
				{
					gRobot.rightGun.reloadPose = gRightGunReloadPosDatabase[gRobot.rightGun.shootParaMode]\
																[gRobot.rightGun.targetPlant];

					ROBOT_RightGunReloadAim();
					ROBOT_RightGunCheckReloadAim();
					RightPush();
				}
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
//		RightGunSendDebugInfo();
		gRobot.rightGun.checkTimeUsage = 0;
	}

}

void UpperGunShootTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;

	//fix me, if camera send data, this flag = 1
	uint8_t upperGunShootFlag = 0;
	OSTimeDly(150);
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
			while(1) 
			{
				UART5_OUT((uint8_t *)"Upper Gun Mode Error!!!!!!!!!\r\n");
			}
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
			if(gRobot.autoCommand[PLANT1].ball == 0&&\
				gRobot.autoCommand[PLANT2].ball == 0 &&\
				gRobot.autoCommand[PLANT1].plate == 0 &&\
				gRobot.autoCommand[PLANT2].plate == 0)
			{
				for(uint8_t i = 0; i < 7;i++)
				{
					gRobot.autoCommand[i].ball = 1;
				}
				for(uint8_t i = 0; i < 7;i++)
				{
					gRobot.autoCommand[i].plate = 1;
				}			
			}
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

