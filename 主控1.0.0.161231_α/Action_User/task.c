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

/*
===============================================================
                        信号量定义
===============================================================
*/
OS_EVENT *PeriodSem;
//邮箱定义
OS_EVENT *OpenSaftyMail;
OS_EVENT *ShootPointMail;
//定义机器人全局变量
extern robot_t gRobot;

static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];
static  OS_STK  WalkTaskStk[Walk_TASK_STK_SIZE];
static  OS_STK  LeftGunShootTaskStk[LEFT_GUN_AUTO_SHOOT_STK_SIZE];
static  OS_STK  RightGunShootTaskStk[RIGHT_GUN_SHOOT_STK_SIZE];
static  OS_STK  UpperGunShootTaskStk[UPPER_GUN_SHOOT_STK_SIZE];

void LeftGunShootTask(void);
void RightGunShootTask(void);
void UpperGunShootTask(void);

//调试数据发送不能超过30个字节，发送10个字节需要1ms
void sendDebugInfo(void)
{
#define POS_X_OFFSET 50
	USART_SendData(UART5, (int8_t)gRobot.moveBase.targetSpeed.leftWheelSpeed);
	USART_SendData(UART5, (int8_t)gRobot.moveBase.targetSpeed.forwardWheelSpeed);
	USART_SendData(UART5, (int8_t)gRobot.moveBase.targetSpeed.backwardWheelSpeed);
	
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.actualSpeed.leftWheelSpeed);
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.actualSpeed.forwardWheelSpeed);
//	USART_SendData(UART5, (int8_t)gRobot.moveBase.actualSpeed.backwardWheelSpeed);

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
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCommandVelocity.leftDriverCommandVelocity);
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCommandVelocity.forwardDriverCommandVelocity);
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverCommandVelocity.backwardDriverCommandVelocity);
	
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverJoggingVelocity.leftDriverJoggingVelocity);
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverJoggingVelocity.forwardDriverJoggingVelocity);
	USART_SendData(UART5, (uint8_t)gRobot.moveBase.driverJoggingVelocity.backwardDriverJoggingVelocity);
	
	//角度范围【-180，180】，但是实际走行中角度值基本在0度附近，fix me
	USART_SendData(UART5, (int8_t)gRobot.moveBase.actualAngle);

	//X位移分米部分范围是【-140，10】，单位分米
	USART_SendData(UART5, (int8_t)(gRobot.moveBase.actualXPos/100.0f+ POS_X_OFFSET));
	//X位移厘米部分范围是【-100，100】，单位厘米
	USART_SendData(UART5, (uint8_t)((((int)gRobot.moveBase.actualXPos))%100/10));

	//根据场地约束，范围设计为【-130，130】，单位cm
	USART_SendData(UART5, (int8_t)(gRobot.moveBase.actualYPos/10.0f));

	//连续发送4个-100作为结束标识符
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
}


void CameraInit(void)
{
	USART_SendData(USART3, 'a');
	USART_SendData(USART3, 'a');
	USART_SendData(USART3, 'r');
}
void SendStop2Camera(void)
{
	USART_SendData(USART3, 'c');
}
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;          /*防止警告...*/

	/*创建信号量*/
    PeriodSem				=	OSSemCreate(0);
	//创建邮箱
	OpenSaftyMail            =   OSMboxCreate((void *)0);
	ShootPointMail           =   OSMboxCreate((void *)0);
	
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
	
	KeyInit();
	PhotoelectricityInit();
	BeepInit();
	
	//串口初始化
	UART4_Init(115200);     //蓝牙手柄
	UART5_Init(115200);		//调试用wifi
	USART3_Init(115200);    //摄像头
	USART6_Init(115200);	//定位系统
	Flash_Init();
	TIM_Delayms(TIM5, 10000);




	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

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


//	OSTaskSuspend(Walk_TASK_PRIO);

//	OSTaskSuspend(LEFT_GUN_SHOOT_TASK_PRIO);
	OSTaskSuspend(RIGHT_GUN_SHOOT_TASK_PRIO);
//	OSTaskSuspend(UPPER_GUN_SHOOT_TASK_PRIO);

	OSTaskSuspend(OS_PRIO_SELF);
}





//坐标修正量及修正标志位
float amendX = 0.0f;
uint8_t amendXFlag = 0;
//走行移动计时标志位
uint8_t moveTimFlag = 0;
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

void WalkTask(void)
{
	static uint16_t timeCounter = 0;
	CPU_INT08U  os_err;
	os_err = os_err;
	int *shootPointMsg = (int *)MOVEBASE_POS_READY;
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

		
		sendDebugInfo();
		switch (status)
		{
			//准备阶段
			case getReady:				
				if(PHOTOSENSORUPGUN)
				{
					ClampOpen();
					ROBOT_LeftGunAim();
					status ++;
				}
				break;
			//从出发区走向装载区
			case goToLoadingArea:

			    MoveTo(-12776.96f, -1500.0f, 2000.0f);
				if (GetPosX() <= -12650.0f && PHOTOSENSORLEFT)
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
					OSTimeDly(50);
					ClampReset();
					status ++;
				}
				break;
			
			case beginToGo1:
				if (PHOTOSENSORUPGUN)
				{
					status++;
				}
				break;
				
			//四分之三位置停车
			case goToHalfLaunchingArea:
				MoveTo(-9459.14f, 1500.0f, 1200.0f);
			    if (GetPosX() >= -9459.14f)
				{
					LockWheel();
					moveTimFlag = 0;
					status++;					
//					OSTaskResume(LEFT_GUN_SHOOT_TASK_PRIO);
					OSMboxPostOpt(ShootPointMail , shootPointMsg , OS_POST_OPT_BROADCAST);
					OSTaskSuspend(OS_PRIO_SELF);
				}
				break;
			
			case beginToGo2:
				if (PHOTOSENSORUPGUN)
				{
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
//					OSTaskResume(LEFT_GUN_SHOOT_TASK_PRIO);
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
                MoveTo(-6459.14f, 1500.0f, 1200.0f);
			    if (GetPosX() >= -6459.14f)
				{
					LockWheel();
					moveTimFlag = 0;
					OSMboxPostOpt(ShootPointMail , shootPointMsg , OS_POST_OPT_BROADCAST);
					status++;
				}
				break;
			
			//发射飞盘
			case launch:
				LockWheel();
				CameraInit();
				SendStop2Camera();
//				OSTaskResume(LEFT_GUN_SHOOT_TASK_PRIO);
//				OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
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

    OSMboxPend(OpenSaftyMail, 0, &os_err);
	gRobot.leftGun.mode = GUN_AUTO_MODE;
	//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
	int stopAutoFlag = 0;
	while(1)
	{
		//检查手动or自动
		//auto mode用在正式比赛中，平板上位机只会发送枪号和柱子号
		if(ROBOT_GunCheckMode(LEFT_GUN) == GUN_AUTO_MODE)
		{
			//一旦收到发射命令，则停止自动模式
			if(gRobot.leftGun.shoot == GUN_START_SHOOT) stopAutoFlag = 1;

			if(stopAutoFlag || gRobot.leftGun.shootTimes >= LEFT_GUN_POINT1_AUTO_BULLET_NUMBER + LEFT_GUN_POINT2_AUTO_BULLET_NUMBER + LEFT_GUN_POINT3_AUTO_BULLET_NUMBER)
			{
				//自动射击已完成
				if(gRobot.leftGun.shoot == GUN_START_SHOOT)
				{
					//fix me,这里存在的风险是，自动过程中，手动修改柱子命令，这时候有可能结果不一致，要改
					//子弹上膛,第一次上膛默认位置OK
					ROBOT_LeftGunReload();
					//检查并更新子弹状态
					ROBOT_GunCheckBulletState(LEFT_GUN);

					//fix me,此处应该检查着陆台编号是否合法
					shoot_command_t shootCommand = gRobot.leftGun.shootCommand[gRobot.leftGun.shootTimes];
					uint8_t shootPoint = shootCommand.shootPoint;
					uint8_t landId = shootCommand.plantNum;
					uint8_t shootMethod = shootCommand.shootMethod;
					//获取目标位姿
					gun_pose_t pose = gLeftGunPosDatabase[shootPoint][shootMethod][landId];

					//更新枪目标位姿
					gRobot.leftGun.targetPose.pitch =	pose.pitch;
					gRobot.leftGun.targetPose.roll =	pose.roll;
					gRobot.leftGun.targetPose.yaw =		pose.yaw;
					gRobot.leftGun.targetPose.speed1 =	pose.speed1;
					gRobot.leftGun.targetPose.speed2 =	pose.speed2;

					//瞄准，此函数最好瞄准完成后再返回
					//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!
					ROBOT_LeftGunAim();
					ROBOT_LeftGunCheckAim();
					ROBOT_LeftGunShoot();
					//此函数有延迟
//					ROBOT_LeftGunHome();
					gRobot.leftGun.shoot = GUN_STOP_SHOOT;
				}
				else
				{
					OSTaskSuspend(OS_PRIO_SELF);
				}
			}
			else
			{
				//fix me,这里存在的风险是，自动过程中，手动修改柱子命令，这时候有可能结果不一致，要改	
				//获取发射命令
				shoot_command_t shootCommand = gRobot.leftGun.shootCommand[gRobot.leftGun.shootStep];
				uint8_t shootPoint = shootCommand.shootPoint;
				uint8_t landId = shootCommand.plantNum;
				uint8_t shootMethod = shootCommand.shootMethod;
				gRobot.leftGun.targetStepShootTimes = shootCommand.stepTargetShootTime;
				//获取目标位姿
				gun_pose_t pose = gLeftGunPosDatabase[shootPoint][shootMethod][landId];
				//更新枪目标位姿
				gRobot.leftGun.targetPose.pitch = pose.pitch;
				gRobot.leftGun.targetPose.roll = pose.roll;
				gRobot.leftGun.targetPose.yaw = pose.yaw;
				gRobot.leftGun.targetPose.speed1 = pose.speed1;
				gRobot.leftGun.targetPose.speed2 = pose.speed2;

				//瞄准，此函数最好瞄准完成后再返回
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!
				//子弹上膛,第一次上膛默认位置OK
				//7号3号柱子正常参数上弹易卡，需要单独处理 fix me
				if(landId != PLANT7)
				{
					ROBOT_LeftGunAim();
					ROBOT_LeftGunReload();
				}
				else
				{
					ROBOT_LeftGunAim();
					PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(20.0f));
					ROBOT_LeftGunReload();
				}
				//检查上弹是否到位
				ROBOT_LeftGunCheckReload();
				//上弹到位后再次瞄准，并检查枪是否到位
				ROBOT_LeftGunAim();
				ROBOT_LeftGunCheckAim();
				//检查是否进入下一步
				ROBOT_LeftGunCheckStep();
				//检查是否到达发射点
				ROBOT_LeftGunCheckShootPoint();	
				//发射飞盘
				ROBOT_LeftGunShoot();
//				USART_SendData(UART5,(uint8_t)gRobot.leftGun.shootTimes);
//				USART_SendData(UART5,(uint8_t)gRobot.leftGun.shootStep);
//				USART_SendData(UART5, (uint8_t)-100);
//				USART_SendData(UART5, (uint8_t)-100);
//				USART_SendData(UART5, (uint8_t)-100);
//				USART_SendData(UART5, (uint8_t)-100);
				//fix me ,第一个位置发射结束后要走到第二个位置，最好用函数来检查
				if(gRobot.leftGun.shootTimes == LEFT_GUN_POINT1_AUTO_BULLET_NUMBER)
				{
//					gRobot.leftGun.mode = GUN_MANUAL_MODE;
					OSTaskResume(Walk_TASK_PRIO);
				}
				//自动射击结束后进入纯手动模式
				if(gRobot.leftGun.shootTimes >= LEFT_GUN_POINT1_AUTO_BULLET_NUMBER + LEFT_GUN_POINT3_AUTO_BULLET_NUMBER)
				{
					gRobot.leftGun.mode = GUN_MANUAL_MODE;
				}
			}
		}
		//手动模式用于调试过程中，对端设备只会发送枪号和着陆号，枪的姿态
		//调试过程中着陆台信息没有用，根据shoot标志来开枪
		else if(ROBOT_GunCheckMode(LEFT_GUN) == GUN_MANUAL_MODE)
		{
			if(gRobot.leftGun.aim == GUN_START_AIM)
			{
				//获得目标位姿，这里应该由对端设备发送过来，直接更新的gRobot.leftGun中的目标位姿
				//瞄准，此函数最好瞄准完成后再返回
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!					
				ROBOT_LeftGunAim();
				//更新数据库中参数并写入FLASH
				UpdateLeftGunPosDatabaseManulMode();
				FlashWriteGunPosData();
				ROBOT_LeftGunCheckAim();
				gRobot.leftGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.leftGun.shoot==GUN_START_SHOOT)
			{
				ROBOT_LeftGunShoot();
				OSTimeDly(50);
				//更改射击命令标记，此标记在接收到对端设备发生命令时更新
				gRobot.leftGun.shoot = GUN_STOP_SHOOT;
				ROBOT_LeftGunReload();
			}
			else
			{
//				OSTaskResume(Walk_TASK_PRIO);
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

//fix me 右枪还未安装，此部分功能还没有测试
void RightGunShootTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
    OSMboxPend(OpenSaftyMail, 0, &os_err);
	gRobot.rightGun.mode = GUN_AUTO_MODE;
	//自动模式下，如果收到对端设备发送的命令，则停止自动模式进入自动模式中的手动部分，只指定着陆台，不要参数
	int stopAutoFlag = 0;
	while(1)
	{
		//检查手动or自动
		//auto mode用在正式比赛中，平板上位机只会发送枪号和柱子号
		if(ROBOT_GunCheckMode(RIGHT_GUN) == GUN_AUTO_MODE)
		{
			//一旦收到发射命令，则停止自动模式
			if(gRobot.rightGun.shoot == GUN_START_SHOOT) stopAutoFlag = 1;

			if(stopAutoFlag || gRobot.rightGun.shootTimes >= RIGHT_GUN_POINT1_AUTO_BULLET_NUMBER + RIGHT_GUN_POINT2_AUTO_BULLET_NUMBER + RIGHT_GUN_POINT3_AUTO_BULLET_NUMBER)
			{
				//自动射击已完成
				if(gRobot.rightGun.shoot == GUN_START_SHOOT)
				{
					//fix me,这里存在的风险是，自动过程中，手动修改柱子命令，这时候有可能结果不一致，要改
					//子弹上膛,第一次上膛默认位置OK
					ROBOT_RightGunReload();
					//检查并更新子弹状态
					ROBOT_GunCheckBulletState(RIGHT_GUN);

					//fix me,此处应该检查着陆台编号是否合法
					shoot_command_t shootCommand = gRobot.rightGun.shootCommand[gRobot.rightGun.shootTimes];
					uint8_t shootPoint = shootCommand.shootPoint;
					uint8_t landId = shootCommand.plantNum;
					uint8_t shootMethod = shootCommand.shootMethod;
					//获取目标位姿
					gun_pose_t pose = gRightGunPosDatabase[shootPoint][shootMethod][landId];

					//更新枪目标位姿
					gRobot.rightGun.targetPose.pitch =	pose.pitch;
					gRobot.rightGun.targetPose.roll =	pose.roll;
					gRobot.rightGun.targetPose.yaw =	pose.yaw;
					gRobot.rightGun.targetPose.speed1 =	pose.speed1;
					gRobot.rightGun.targetPose.speed2 =	pose.speed2;

					//瞄准，此函数最好瞄准完成后再返回
					//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!
					ROBOT_RightGunAim();
					ROBOT_RightGunCheckAim();
					ROBOT_RightGunShoot();
					//此函数有延迟
//					ROBOT_RightGunHome();
					gRobot.rightGun.shoot = GUN_STOP_SHOOT;
				}
				else
				{
					OSTaskSuspend(OS_PRIO_SELF);
				}
			}
			else
			{
				//fix me,这里存在的风险是，自动过程中，手动修改柱子命令，这时候有可能结果不一致，要改	
				//获取发射命令
				shoot_command_t shootCommand = gRobot.rightGun.shootCommand[gRobot.rightGun.shootStep];
				uint8_t shootPoint = shootCommand.shootPoint;
				uint8_t landId = shootCommand.plantNum;
				uint8_t shootMethod = shootCommand.shootMethod;
				gRobot.rightGun.targetStepShootTimes = shootCommand.stepTargetShootTime;
				//获取目标位姿
				gun_pose_t pose = gRightGunPosDatabase[shootPoint][shootMethod][landId];
				//更新枪目标位姿
				gRobot.rightGun.targetPose.pitch = pose.pitch;
				gRobot.rightGun.targetPose.roll = pose.roll;
				gRobot.rightGun.targetPose.yaw = pose.yaw;
				gRobot.rightGun.targetPose.speed1 = pose.speed1;
				gRobot.rightGun.targetPose.speed2 = pose.speed2;

				//瞄准，此函数最好瞄准完成后再返回
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!
				//子弹上膛,第一次上膛默认位置OK
				//7号3号柱子正常参数上弹易卡，需要单独处理 fix me
				if(landId != PLANT7)
				{
					ROBOT_RightGunAim();
					ROBOT_RightGunReload();
				}
				else
				{
					ROBOT_RightGunAim();
					PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(20.0f));
					ROBOT_RightGunReload();
				}
				//检查上弹是否到位
				ROBOT_RightGunCheckReload();
				//上弹到位后再次瞄准，并检查枪是否到位
				ROBOT_RightGunAim();
				ROBOT_RightGunCheckAim();
				//检查是否进入下一步
				ROBOT_RightGunCheckStep();
				//检查是否到达发射点
				ROBOT_RightGunCheckShootPoint();	
				//发射飞盘
				ROBOT_RightGunShoot();
				//fix me ,第一个位置发射结束后要走到第二个位置，最好用函数来检查
				if(gRobot.rightGun.shootTimes == RIGHT_GUN_POINT1_AUTO_BULLET_NUMBER)
				{
//					gRobot.leftGun.mode = GUN_MANUAL_MODE;
					OSTaskResume(Walk_TASK_PRIO);
				}
				//自动射击结束后进入纯手动模式
				if(gRobot.rightGun.shootTimes >= RIGHT_GUN_POINT1_AUTO_BULLET_NUMBER + RIGHT_GUN_POINT3_AUTO_BULLET_NUMBER)
				{
					gRobot.rightGun.mode = GUN_MANUAL_MODE;
				}
			}
		}
		//手动模式用于调试过程中，对端设备只会发送枪号和着陆号，枪的姿态
		//调试过程中着陆台信息没有用，根据shoot标志来开枪
		else if(ROBOT_GunCheckMode(RIGHT_GUN) == GUN_MANUAL_MODE)
		{
			if(gRobot.rightGun.aim == GUN_START_AIM)
			{
				//获得目标位姿，这里应该由对端设备发送过来，直接更新的gRobot.leftGun中的目标位姿
				//瞄准，此函数最好瞄准完成后再返回
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!					
				ROBOT_RightGunAim();
				//更新数据库中参数并写入FLASH
				UpdateRightGunPosDatabaseManulMode();
				FlashWriteGunPosData();
				ROBOT_RightGunCheckAim();
				gRobot.rightGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.rightGun.shoot==GUN_START_SHOOT)
			{
				ROBOT_RightGunShoot();
				OSTimeDly(50);
				//更改射击命令标记，此标记在接收到对端设备发生命令时更新
				gRobot.rightGun.shoot = GUN_STOP_SHOOT;
				ROBOT_RightGunReload();
			}
			else
			{
//				OSTaskResume(Walk_TASK_PRIO);
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

void UpperGunShootTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;

	//fix me, if camera send data, this flag = 1
	uint8_t upperGunShootFlag = 0;
	while(1)
	{
		//检查手动or自动
		//auto mode用在正式比赛中，与左右两枪不同，通过摄像头的反馈发射飞盘
		gRobot.upperGun.mode = GUN_MANUAL_MODE;
		if(ROBOT_GunCheckMode(UPPER_GUN) == GUN_AUTO_MODE)
		{
			//fix me,此处应该检查目标区域是否合法
			if(gRobot.upperGun.targetZone & 0x0f) upperGunShootFlag = 1;

			if(upperGunShootFlag == 1 && gRobot.upperGun.shootTimes <= MAX_AUTO_BULLET_NUMBER)
			{
				int zoneId = INVALID_ZONE_NUMBER;
				//fix me,此处应该检查着陆台编号是否合法
				if(gRobot.upperGun.targetZone & 0x01)      zoneId = ZONE1;
				else if(gRobot.upperGun.targetZone & 0x02) zoneId = ZONE2;
				else if(gRobot.upperGun.targetZone & 0x04) zoneId = ZONE3;
				else if(gRobot.upperGun.targetZone & 0x08) zoneId = ZONE4;
				//fix me,there may be something wrong
				else                                       continue;

				//获取目标位姿
				gun_pose_t pose = gUpperGunPosDatabase[gRobot.upperGun.shootParaMode][zoneId];
				//fix me,这里存在的风险是，自动过程中，手动修改柱子命令，这时候有可能结果不一致，要改

				//更新枪目标位姿
				gRobot.upperGun.targetPose.pitch = pose.pitch;
				gRobot.upperGun.targetPose.yaw = pose.yaw;
				gRobot.upperGun.targetPose.speed1 = pose.speed1;

				//瞄准，此函数最好瞄准完成后再返回
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!
				ROBOT_UpperGunAim();
				OSTimeDly(5);
				ROBOT_UpperGunCheckAim();
				if (gRobot.upperGun.shoot == GUN_START_SHOOT)
				{
					ROBOT_UpperGunShoot();
					gRobot.upperGun.shoot = GUN_STOP_SHOOT;
					gRobot.upperGun.targetZone = 0x00;
					upperGunShootFlag = 0;
					OSTimeDly(50);
				}
			}
			else
			{
				OSTaskSuspend(OS_PRIO_SELF);
			}
		}
		//手动模式用于调试过程中，对端设备只会发送枪号和着陆号，枪的姿态
		//调试过程中着陆台信息没有用，根据shoot标志来开枪
		else if(ROBOT_GunCheckMode(UPPER_GUN) == GUN_MANUAL_MODE)
		{
			if(gRobot.upperGun.aim == GUN_START_AIM)
			{
				//检查并更新子弹状态，训练时需要记录
				ROBOT_GunCheckBulletState(UPPER_GUN);
				//获得目标位姿，这里应该由对端设备发送过来，直接更新的gRobot.leftGun中的目标位姿
				//瞄准，此函数最好瞄准完成后再返回
				//这个函数使用了CAN，要考虑被其他任务抢占的风险,dangerous!!!					
				ROBOT_UpperGunAim();
				ROBOT_UpperGunCheckAim();
				gRobot.upperGun.aim = GUN_STOP_AIM;
			}
			else if(gRobot.upperGun.shoot==GUN_START_SHOOT)
			{
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

