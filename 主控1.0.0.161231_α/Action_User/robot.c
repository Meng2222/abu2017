#include <math.h>
#include <string.h>
#include "robot.h"
#include "elmo.h"
#include "database.h"
#include "gasvalvecontrol.h"
#include "timer.h"
#include "ucos_ii.h"
#include "gpio.h"
#include "cpu.h"
#include "usart.h"
#include "app_cfg.h"
#include "stdlib.h"
#include "time.h"
#include "rng.h"
robot_t gRobot = {0};
extern OS_EVENT *OpenSaftyMbox;
extern OS_EVENT *LeftGunShootPointMbox;
extern OS_EVENT *RightGunShootPointMbox;
extern OS_EVENT *UpperGunShootPointMbox;

/*
============================================================
						  枪初始化
============================================================
*/


static void LeftGunInit(void)
{
	gRobot.leftGun.actualPose.pitch = 0.0f;
	gRobot.leftGun.actualPose.yaw = 0.0f;
	gRobot.leftGun.actualPose.roll = 0.0f;
	
	//将参数赋值到上弹姿势中
	for(uint8_t k = 0;k < SHOOT_POINT_NUM;k++)
	{
		for(uint8_t i = 0;i < SHOOT_METHOD_NUMBER;i++)
		{
			for(uint8_t j = 0;j < LAND_NUMBER;j++)
			{
				gLeftGunReloadPosDatabase[k][i][j] = gLeftGunPosDatabase[k][i][j];
			}
		}
	}
	//特殊上弹角度
	gLeftGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD1][PLANT3].pitch = 26.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD2][PLANT3].pitch = 26.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD3][PLANT3].pitch = 26.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT3].pitch = 26.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD1][PLANT7].pitch = 20.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD2][PLANT7].pitch = 20.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD3][PLANT7].pitch = 20.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT7].pitch = 20.0f;


	gRobot.leftGun.maxPoseLimit.pitch = 40.0f;
	gRobot.leftGun.maxPoseLimit.yaw = 50.0f;
	gRobot.leftGun.maxPoseLimit.roll = 46.54f;
	gRobot.leftGun.maxPoseLimit.speed1=200.0f;
	gRobot.leftGun.maxPoseLimit.speed2=200.0f;


	gRobot.leftGun.minPoseLimit.pitch = 7.0f;
	gRobot.leftGun.minPoseLimit.yaw = -50.0f;
	gRobot.leftGun.minPoseLimit.roll = -43.46f;
	gRobot.leftGun.minPoseLimit.speed1=-200.0f;
	gRobot.leftGun.minPoseLimit.speed2=-200.0f;

	//枪未进行瞄准
	gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;
	//自动模式
	gRobot.leftGun.mode = GUN_AUTO_MODE;
	//子弹数
	gRobot.leftGun.bulletNumber = MAX_BULLET_NUMBER_LEFT;
	//认为第一发上弹没有问题
	gRobot.leftGun.champerErrerState = GUN_RELOAD_OK;
	//枪停止射击
	gRobot.leftGun.shoot = GUN_STOP_SHOOT;
	//左枪姿态数据库
	gRobot.leftGun.gunPoseDatabase = (gun_pose_t **)gLeftGunPosDatabase;
	//左枪自动发射命令集合，里面为投射柱子的顺序
	gRobot.leftGun.shootCommand = (shoot_command_t *)gLeftGunShootCmds;
	//目标着陆台设置为无效台
	gRobot.leftGun.targetPlant = INVALID_PLANT_NUMBER;
	//防守区设置为无敌方盘
	gRobot.leftGun.defendZone1 = NO_ENEMY_DISK;
	gRobot.leftGun.defendZone2 = NO_ENEMY_DISK;
	//当前打盘区设为无效区
	gRobot.leftGun.presentDefendZoneId = INVALID_ZONE_NUMBER;
	//上一打盘区设为无效区
	gRobot.leftGun.lastDefendZoneId = INVALID_ZONE_NUMBER;
	//射击次数为0
	gRobot.leftGun.shootTimes = 0;
	//初始化时命令指向自动命令
	gRobot.leftGun.gunCommand = (plant_t *)gRobot.autoCommand;
	gRobot.leftGun.lastPlant = INVALID_PLANT_NUMBER;
	gRobot.leftGun.lastParaMode = INVALID_SHOOT_METHOD;

	elmo_Enable(CAN1, LEFT_GUN_LEFT_ID);
	elmo_Enable(CAN1, LEFT_GUN_RIGHT_ID);
	elmo_Enable(CAN1, LEFT_GUN_PITCH_ID);
	elmo_Enable(CAN1, LEFT_GUN_ROLL_ID);
	elmo_Enable(CAN1, LEFT_GUN_YAW_ID);


	Vel_cfg(CAN1, LEFT_GUN_LEFT_ID, 300000,300000);
	Vel_cfg(CAN1, LEFT_GUN_RIGHT_ID, 300000,300000);

	Pos_cfg(CAN1, LEFT_GUN_PITCH_ID, 50000,50000,80000);//俯仰
	Pos_cfg(CAN1, LEFT_GUN_ROLL_ID, 50000,50000,80000);//翻滚
	Pos_cfg(CAN1, LEFT_GUN_YAW_ID,50000,50000,80000);//航向

//	ROBOT_LeftGunHome();
//	PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(5.0f));
	VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(0.0f));
	VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(0.0f));


}

static void RightGunInit(void)
{
	gRobot.rightGun.actualPose.pitch = 0.0f;
	gRobot.rightGun.actualPose.yaw = 0.0f;
	gRobot.rightGun.actualPose.roll = 0.0f;
	
	//将参数赋值到上弹姿势中
	for(uint8_t k = 0;k < SHOOT_POINT_NUM;k++)
	{
		for(uint8_t i = 0;i < SHOOT_METHOD_NUMBER;i++)
		{
			for(uint8_t j = 0;j < LAND_NUMBER;j++)
			{
				gRightGunReloadPosDatabase[k][i][j] = gRightGunPosDatabase[k][i][j];
			}
		}
	}
	gRightGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD1][PLANT3].pitch = 26.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD2][PLANT3].pitch = 26.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD3][PLANT3].pitch = 26.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT3].pitch = 26.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD1][PLANT7].pitch = 20.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD2][PLANT7].pitch = 20.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD3][PLANT7].pitch = 20.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT7].pitch = 20.0f;

	gRobot.rightGun.maxPoseLimit.pitch = 40.0f;
	gRobot.rightGun.maxPoseLimit.yaw = 50.0f;
	gRobot.rightGun.maxPoseLimit.roll = 46.54f;
	gRobot.rightGun.maxPoseLimit.speed1=200.0f;
	gRobot.rightGun.maxPoseLimit.speed2=200.0f;

	gRobot.rightGun.minPoseLimit.pitch = 7.0f;
	gRobot.rightGun.minPoseLimit.yaw = -50.0f;
	gRobot.rightGun.minPoseLimit.roll = -43.46f;
	gRobot.rightGun.minPoseLimit.speed1=-200.0f;
	gRobot.rightGun.minPoseLimit.speed2=-200.0f;

	//枪未进行瞄准
	gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
	//自动模式
	gRobot.rightGun.mode = GUN_AUTO_MODE;
	//最大子弹数
	gRobot.rightGun.bulletNumber = MAX_BULLET_NUMBER_RIGHT;
	//默认第一发初始上弹没有问题
	gRobot.rightGun.champerErrerState = GUN_RELOAD_OK;
	//枪停止射击
	gRobot.rightGun.shoot = GUN_STOP_SHOOT;
	//右枪姿态数据库
	gRobot.rightGun.gunPoseDatabase = (gun_pose_t **)gRightGunPosDatabase;
	//右枪自动发射命令集合，里面为投射柱子的顺序
	gRobot.rightGun.shootCommand = (shoot_command_t *)gRightGunShootCmds;
	//目标着陆台设置为无效台
	gRobot.rightGun.targetPlant = INVALID_PLANT_NUMBER;
	//防守区设置为无敌方盘
	gRobot.rightGun.defendZone1 = NO_ENEMY_DISK;
	gRobot.rightGun.defendZone2 = NO_ENEMY_DISK;
	//当前打盘区设为无效区
	gRobot.rightGun.presentDefendZoneId = INVALID_ZONE_NUMBER;
	//上一打盘区设为无效区
	gRobot.rightGun.lastDefendZoneId = INVALID_ZONE_NUMBER;
	//射击次数为0
	gRobot.rightGun.shootTimes = 0;
	//初始化时命令指向自动命令
	gRobot.rightGun.gunCommand = (plant_t *)gRobot.autoCommand;
	gRobot.rightGun.lastPlant = INVALID_PLANT_NUMBER;
	gRobot.rightGun.lastParaMode = INVALID_SHOOT_METHOD;

	elmo_Enable(CAN1, RIGHT_GUN_LEFT_ID);
	elmo_Enable(CAN1, RIGHT_GUN_RIGHT_ID);
	elmo_Enable(CAN1, RIGHT_GUN_PITCH_ID);
	elmo_Enable(CAN1, RIGHT_GUN_ROLL_ID);
	elmo_Enable(CAN1, RIGHT_GUN_YAW_ID);


	Vel_cfg(CAN1, RIGHT_GUN_LEFT_ID, 300000,300000);
	Vel_cfg(CAN1, RIGHT_GUN_RIGHT_ID, 300000,300000);

	Pos_cfg(CAN1, RIGHT_GUN_PITCH_ID, 50000,50000,80000);//俯仰
	Pos_cfg(CAN1, RIGHT_GUN_ROLL_ID, 50000,50000,80000);//翻滚
	Pos_cfg(CAN1, RIGHT_GUN_YAW_ID,50000,50000,80000);//航向

//	ROBOT_RightGunHome();
//	PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(-5.0f));
	VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(0.0f));
	VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(0.0f));

}

static void UpperGunInit(void)
{
	gRobot.upperGun.actualPose.pitch = 0.0f;
	gRobot.upperGun.actualPose.yaw = 0.0f;
	gRobot.upperGun.actualPose.roll = 0.0f;

	gRobot.upperGun.targetPose.pitch = 0.0f;
	gRobot.upperGun.targetPose.yaw = 0.0f;
	gRobot.upperGun.targetPose.roll = 0.0f;

	gRobot.upperGun.maxPoseLimit.pitch = 40.0f;
	gRobot.upperGun.maxPoseLimit.yaw = 50.0f;
	gRobot.upperGun.maxPoseLimit.roll = 0.0f;
	gRobot.upperGun.maxPoseLimit.speed1=200.0f;
	gRobot.upperGun.maxPoseLimit.speed2=200.0f;

	gRobot.upperGun.minPoseLimit.pitch = -10.0f;
	gRobot.upperGun.minPoseLimit.yaw = -50.0f;
	gRobot.upperGun.minPoseLimit.roll = 0.0f;
	gRobot.upperGun.minPoseLimit.speed1=-200.0f;
	gRobot.upperGun.minPoseLimit.speed2=-200.0f;

	gRobot.upperGun.targetPose.pitch = gUpperGunPosDatabase[SHOOT_POINT2][gUpperGunShootCmds[0].plantNum]\
															[gUpperGunShootCmds[0].shootMethod][ZONE1].pitch;
	gRobot.upperGun.targetPose.yaw = gUpperGunPosDatabase[SHOOT_POINT2][gUpperGunShootCmds[0].plantNum]\
														[gUpperGunShootCmds[0].shootMethod][ZONE1].yaw;
	gRobot.upperGun.targetPose.speed1 = gUpperGunPosDatabase[SHOOT_POINT2][gUpperGunShootCmds[0].plantNum]\
														[gUpperGunShootCmds[0].shootMethod][ZONE1].speed1;


	//枪未进行瞄准
	gRobot.upperGun.ready = GUN_AIM_IN_PROCESS;
	//自动模式
	gRobot.upperGun.mode = GUN_MANUAL_MODE;
	//最大子弹数
	gRobot.upperGun.bulletNumber = MAX_BULLET_NUMBER_UPPER;
	//fix me
	gRobot.upperGun.champerErrerState = 0;
	//枪停止射击
	gRobot.upperGun.shoot = GUN_STOP_SHOOT;
	//上枪姿态数据库
	gRobot.upperGun.gunPoseDatabase = (gun_pose_t **)gUpperGunPosDatabase;
	//上面枪自动发射命令集合，里面为投射柱子的顺序
	gRobot.upperGun.shootCommand = (shoot_command_t *)gUpperGunShootCmds;
	//目标着陆台设置为无效台
	gRobot.upperGun.targetPlant = INVALID_PLANT_NUMBER;
	//防守区设置为无敌方盘
	gRobot.upperGun.defendZone1 = NO_ENEMY_DISK;
	gRobot.upperGun.defendZone2 = NO_ENEMY_DISK;
	//当前打盘区设为无效区
	gRobot.upperGun.presentDefendZoneId = INVALID_ZONE_NUMBER;
	//上一打盘区设为无效区
	gRobot.upperGun.lastDefendZoneId = INVALID_ZONE_NUMBER;
	//射击次数为0
	gRobot.upperGun.shootTimes = 0;
	//初始化时命令指向自动命令
	gRobot.upperGun.gunCommand = (plant_t *)gRobot.autoCommand;

	gRobot.upperGun.lastPlant = INVALID_PLANT_NUMBER;
	gRobot.upperGun.lastParaMode = INVALID_SHOOT_METHOD;

	elmo_Enable(CAN1, UPPER_GUN_LEFT_ID);
	elmo_Enable(CAN1, UPPER_GUN_RIGHT_ID);
	elmo_Enable(CAN1, UPPER_GUN_YAW_ID);
	elmo_Enable(CAN1, UPPER_GUN_PITCH_ID);
	

	Vel_cfg(CAN1, UPPER_GUN_LEFT_ID,350000,350000);
	Vel_cfg(CAN1, UPPER_GUN_RIGHT_ID,350000,350000);
	Pos_cfg(CAN1, UPPER_GUN_YAW_ID,50000,50000,80000);//航向
	Pos_cfg(CAN1, UPPER_GUN_PITCH_ID,50000,50000,80000);//俯仰

//	PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(0.0f));
//	PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(-10.0f));
	VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(0.0f));
	VelCrl(CAN1, UPPER_GUN_RIGHT_ID, UpperGunRightSpeedTransform(0.0f));
}
/*
*名称：ROBOT_Init
*功能：机器人初始化，初始化底盘，初始化枪，初始化
*参数：none
*注意：上面的枪不需要上子弹，因为是手动上弹
*/
status_t ROBOT_Init(void)
{
	gRobot.stage = ROBOT_STAGE_POWER_ON;
	gRobot.isLeaveSZ = ROBOT_IN_SZ;
	gRobot.shootTimes = 0;
	gRobot.status = ROBOT_STATUS_OK;

	gRobot.moveBase.targetPoint = SHOOT_POINT_MOVING;
	gRobot.moveBase.actualStopPoint = SHOOT_POINT_MOVING;

	gRobot.isReset = ROBOT_NOT_RESET;

	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.plantState[i].ballState = COMMAND_DONE;
	}
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.plantState[i].plateState = COMMAND_DONE;
	}
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.autoCommand[i].ballState = COMMAND_DONE;
	}
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.autoCommand[i].plateState = COMMAND_DONE;
	}
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.upperLeftCommand[i].ballState = COMMAND_DONE;
	}
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.upperLeftCommand[i].plateState = COMMAND_DONE;
	}
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.upperRightCommand[i].ballState = COMMAND_DONE;
	}
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.upperRightCommand[i].plateState = COMMAND_DONE;
	}
	
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.cameraInfo[i].ball = 1;
	}
	for(uint8_t i = PLANT1; i < LAND_NUMBER;i++)
	{
		gRobot.cameraInfo[i].plate = 1;
	}

	
//	gRobot.upperLeftCommand[PLANT2].ball = 1;
//	gRobot.upperRightCommand[PLANT4].ball = 1;

#ifdef AUTO_MODE	
//	InitQueue();
#endif	
	LeftGunInit();
	RightGunInit();
	UpperGunInit();

	MOVEBASE_Init();

	gRobot.stage = ROBOT_STAGE_INIT;

	return GUN_NO_ERROR;
}

/**
  * @brief	Get left gun shoot command
  * @note
  * @param
  *     @arg
  * @param	`
  * @retval
  */
shoot_command_t ROBOT_LeftGunGetShootCommand(void)
{
	#define LEFT_NEW_PLATE_NUM 10u
	shoot_command_t shootCommand = {SHOOT_POINT3, INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	uint8_t searchRange = 2;
	//防止同一个枪连续执行命令
	OSTimeDly(1);
	gRobot.leftGun.commandState = GUN_NO_COMMAND;
//	if(gRobot.leftGun.shootTimes%2==0)
//	{
//		gRobot.plantState[PLANT1].plate = 1;
//	}
//	else
//	{
//		gRobot.plantState[PLANT2].plate = 1;
//	}
	if(gRobot.leftGun.shootTimes >= LEFT_AUTO_NUMBER)
	{
		searchRange = 7;
		RED_LED_ON;
		gRobot.leftGun.gunCommand = (plant_t *)gRobot.plantState;
	}
	if(gRobot.leftGun.shootTimes >= LEFT_BULLET_NUM || gRobot.leftGun.bulletNumber == GUN_NO_BULLET_ERROR)
	{
		gRobot.leftGun.commandState = GUN_NO_COMMAND;
	}
	else
	{
		for(uint8_t i = 0;i < searchRange;i++)
		{
			//有球
			if(gRobot.leftGun.gunCommand[LeftGunPriority[i]].ball >= 1)
			{
				shootCommand.plantNum = LeftGunPriority[i];
				if(gRobot.leftGun.shootTimes < LEFT_AUTO_NUMBER)
					shootCommand.shootMethod = SHOOT_METHOD1;
				else
					shootCommand.shootMethod = SHOOT_METHOD3;
//				else if(gRobot.leftGun.shootTimes < LEFT_NEW_PLATE_NUM)
//					shootCommand.shootMethod = SHOOT_METHOD5;
//				else
//					shootCommand.shootMethod = SHOOT_METHOD1;

				//不连续打同一组参数
				if(gRobot.leftGun.shootTimes >= LEFT_AUTO_NUMBER)
				{
					if(shootCommand.plantNum == gRobot.leftGun.lastPlant &&\
						gRobot.leftGun.lastParaMode == shootCommand.shootMethod)
					{
						if(gRobot.rightGun.lastPlant != shootCommand.plantNum ||\
							gRobot.rightGun.lastParaMode != shootCommand.shootMethod)
						{
							if(gRobot.rightGun.mode != GUN_MANUAL_MODE)
								continue;
						}
					}
				}
				gRobot.leftGun.commandState = GUN_HAVE_COMMAND;
				if(shootCommand.plantNum == gRobot.leftGun.lastPlant)
				{
//					continue;
				}
				break;
			}
			//没盘
			if(gRobot.leftGun.gunCommand[LeftGunPriority[i]].plate >= 1)
			{
				shootCommand.plantNum = LeftGunPriority[i];
				if(gRobot.leftGun.shootTimes < LEFT_AUTO_NUMBER)
					shootCommand.shootMethod = SHOOT_METHOD2;
				else
					shootCommand.shootMethod = SHOOT_METHOD4;
//				else if(gRobot.leftGun.shootTimes < LEFT_NEW_PLATE_NUM)
//					shootCommand.shootMethod = SHOOT_METHOD6;
//				else
//					shootCommand.shootMethod = SHOOT_METHOD4;

				//不连续打同一组参数
				if(gRobot.leftGun.shootTimes >= LEFT_AUTO_NUMBER)
				{
					if(shootCommand.plantNum!=PLANT6)
					{
						if(shootCommand.plantNum == gRobot.leftGun.lastPlant &&\
							gRobot.leftGun.lastParaMode == shootCommand.shootMethod)
						{
							if(gRobot.rightGun.lastPlant != shootCommand.plantNum ||\
								gRobot.rightGun.lastParaMode != shootCommand.shootMethod)
							{
								if(gRobot.rightGun.mode != GUN_MANUAL_MODE)
								continue;
							}
						}
					}
				}
				gRobot.leftGun.commandState = GUN_HAVE_COMMAND;
				if(shootCommand.plantNum == gRobot.leftGun.lastPlant)
				{
//					continue;
				}
				break;
			}
		}
		if(gRobot.leftGun.commandState == GUN_HAVE_COMMAND)
		{
			if(shootCommand.shootMethod%2)
			{
				gRobot.leftGun.gunCommand[shootCommand.plantNum].plate -= 1;
				gRobot.leftGun.gunCommand[shootCommand.plantNum].plateState = COMMAND_IN_PROCESS;
			}
			else
			{
				gRobot.leftGun.gunCommand[shootCommand.plantNum].ball -= 1;
				gRobot.leftGun.gunCommand[shootCommand.plantNum].ballState = COMMAND_IN_PROCESS;
			}
		}
		//左右枪交叉打1#和5#柱子时机械上会有干涉
		if(gRobot.rightGun.targetPlant == PLANT1 && gRobot.rightGun.commandState == GUN_HAVE_COMMAND)
		{
			if(shootCommand.plantNum == PLANT5)
			{
				gRobot.leftGun.commandState = GUN_NO_COMMAND;
				if(shootCommand.shootMethod%2)
					gRobot.leftGun.gunCommand[shootCommand.plantNum].plate += 1;
				else
					gRobot.leftGun.gunCommand[shootCommand.plantNum].ball += 1;
			}
		}
	}
	return shootCommand;
}

/**
  * @brief	Get left gun shoot command FIFO
  * @note
  * @param
  *     @arg
  * @param	`
  * @retval
  */
shoot_command_t ROBOT_LeftGunGetShootCommandFIFO(void)
{

	#define LEFT_NEW_PLATE_NUM 10u
	shoot_command_t shootCommand = {SHOOT_POINT2, INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	cmd_t manualCmd;
	//防止同一个枪连续执行命令
	OSTimeDly(1);
	gRobot.leftGun.commandState = GUN_NO_COMMAND;
	//判断是否没弹
	if(gRobot.leftGun.shootTimes >= LEFT_BULLET_NUM || gRobot.leftGun.bulletNumber <= GUN_NO_BULLET_ERROR)
	{
		gRobot.leftGun.commandState = GUN_NO_COMMAND;
	}
	else		//有弹
	{
		manualCmd = LeftGunOutQueue();
		if(manualCmd.plantNum == INVALID_PLANT_NUMBER)
		{
			gRobot.leftGun.commandState = GUN_NO_COMMAND;		
		}
		else
		{
			shootCommand.plantNum = manualCmd.plantNum;
			shootCommand.shootMethod = manualCmd.method;
			gRobot.leftGun.commandState = GUN_HAVE_COMMAND;
		}
		//左右枪交叉打1#和5#柱子时机械上会有干涉
		if(gRobot.rightGun.targetPlant == PLANT1 && gRobot.rightGun.commandState == GUN_HAVE_COMMAND)
		{
			if(shootCommand.plantNum == PLANT5)
			{
				manualCmd = ReplaceHeadQueue(manualCmd);
				if(manualCmd.plantNum == INVALID_PLANT_NUMBER)
				{
					gRobot.leftGun.commandState = GUN_NO_COMMAND;		
				}
				else
				{
					shootCommand.plantNum = manualCmd.plantNum;
					shootCommand.shootMethod = manualCmd.method;
					gRobot.leftGun.commandState = GUN_HAVE_COMMAND;
				}
			}
		}
		if(gRobot.moveBase.actualStopPoint == SHOOT_POINT2)
		{
			//左枪优先打1#、2#;右枪优先打4#、5#
			if(gRobot.rightGun.commandState == GUN_NO_COMMAND && gRobot.rightGun.bulletNumber > GUN_NO_BULLET_ERROR)
			{
				if((shootCommand.plantNum == PLANT5)||(shootCommand.plantNum == PLANT4))
				{
					manualCmd = ReplaceHeadQueue(manualCmd);
					if(manualCmd.plantNum == INVALID_PLANT_NUMBER)
					{
						gRobot.leftGun.commandState = GUN_NO_COMMAND;		
					}
					else
					{
						shootCommand.plantNum = manualCmd.plantNum;
						shootCommand.shootMethod = manualCmd.method;
						gRobot.leftGun.commandState = GUN_HAVE_COMMAND;
					}
				}
			}
		}
		
		//标记命令开始执行
		if(shootCommand.shootMethod%2)
		{
			gRobot.leftGun.gunCommand[shootCommand.plantNum].plateState = COMMAND_IN_PROCESS;
		}
		else
		{
			gRobot.leftGun.gunCommand[shootCommand.plantNum].ballState = COMMAND_IN_PROCESS;		
		}
	}
	return shootCommand;
}


/**
  * @brief	Get right gun shoot command
  * @note
  * @param
  *     @arg
  * @param
  * @retval
  */

shoot_command_t ROBOT_RightGunGetShootCommand(void)
{
	#define RIGHT_NEW_PLATE_NUM 10u
	shoot_command_t shootCommand = {SHOOT_POINT3, INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	uint8_t searchRange = 2;
	//使同一个枪不连续获得命令
	OSTimeDly(2);
	gRobot.rightGun.commandState = GUN_NO_COMMAND;
	if(gRobot.rightGun.shootTimes >= RIGHT_AUTO_NUMBER)
	{
		searchRange = 7;
		BLUE_LED_ON;
		gRobot.rightGun.gunCommand = (plant_t *)gRobot.plantState;
	}
	if(gRobot.rightGun.shootTimes >= RIGHT_BULLET_NUM || gRobot.rightGun.bulletNumber == GUN_NO_BULLET_ERROR)
	{
		gRobot.rightGun.commandState = GUN_NO_COMMAND;
	}
	else
	{
		for(uint8_t i = 0;i < searchRange;i++)
		{
			//有球
			if(gRobot.rightGun.gunCommand[RightGunPriority[i]].ball >= 1)
			{
				shootCommand.plantNum = RightGunPriority[i];
				if(gRobot.rightGun.shootTimes < RIGHT_AUTO_NUMBER)
					shootCommand.shootMethod = SHOOT_METHOD1;
				else
					shootCommand.shootMethod = SHOOT_METHOD2;
//				else if(gRobot.rightGun.shootTimes < RIGHT_NEW_PLATE_NUM)
//					shootCommand.shootMethod = SHOOT_METHOD5;
//				else
//					shootCommand.shootMethod = SHOOT_METHOD1;

				//不连续同一组参数
				if(gRobot.rightGun.shootTimes >= RIGHT_AUTO_NUMBER)
				{
					if(shootCommand.plantNum == gRobot.rightGun.lastPlant &&\
						gRobot.rightGun.lastParaMode == shootCommand.shootMethod)
					{
						if(gRobot.leftGun.lastPlant != shootCommand.plantNum ||\
							gRobot.leftGun.lastParaMode != shootCommand.shootMethod)
						{
							if(gRobot.leftGun.mode != GUN_MANUAL_MODE)
							continue;
						}
					}
				}
				gRobot.rightGun.commandState = GUN_HAVE_COMMAND;
				if(shootCommand.plantNum == gRobot.rightGun.lastPlant)
				{
//					continue;
				}
				break;
			}
			//没盘
			if(gRobot.rightGun.gunCommand[RightGunPriority[i]].plate >= 1)
			{
				shootCommand.plantNum = RightGunPriority[i];
				if(gRobot.rightGun.shootTimes < RIGHT_AUTO_NUMBER)
					shootCommand.shootMethod = SHOOT_METHOD2;
				else
					shootCommand.shootMethod = SHOOT_METHOD4;
//				else if(gRobot.rightGun.shootTimes < RIGHT_NEW_PLATE_NUM)
//					shootCommand.shootMethod = SHOOT_METHOD6;
//				else
//					shootCommand.shootMethod = SHOOT_METHOD4;
				//不连续打同一组参数
				if(gRobot.rightGun.shootTimes >= RIGHT_AUTO_NUMBER)
				{
					if(RightGunPriority[i]!=PLANT6)
					{
						if(RightGunPriority[i] == gRobot.rightGun.lastPlant &&\
							gRobot.rightGun.lastParaMode == shootCommand.shootMethod)
						{
							if(gRobot.leftGun.lastPlant != RightGunPriority[i] ||\
								gRobot.leftGun.lastParaMode != shootCommand.shootMethod)
							{
								if(gRobot.leftGun.mode != GUN_MANUAL_MODE)
								continue;
							}
						}
					}
				}
				gRobot.rightGun.commandState = GUN_HAVE_COMMAND;
				if(shootCommand.plantNum == gRobot.rightGun.lastPlant)
				{
//					continue;
				}
				break;
			}
		}
		if(gRobot.rightGun.commandState == GUN_HAVE_COMMAND)
		{
			if(shootCommand.shootMethod%2)
			{
				gRobot.rightGun.gunCommand[shootCommand.plantNum].plate -= 1;
				gRobot.rightGun.gunCommand[shootCommand.plantNum].plateState = COMMAND_IN_PROCESS;
			}
			else
			{
				gRobot.rightGun.gunCommand[shootCommand.plantNum].ball -= 1;
				gRobot.rightGun.gunCommand[shootCommand.plantNum].ballState = COMMAND_IN_PROCESS;
			}
		}
		//左右枪交叉打1#和5#柱子时机械上会有干涉
		if(gRobot.leftGun.targetPlant == PLANT5 && gRobot.leftGun.commandState == GUN_HAVE_COMMAND)
		{
			if(shootCommand.plantNum == PLANT1)
			{
				gRobot.rightGun.commandState = GUN_NO_COMMAND;
				if(shootCommand.shootMethod%2)
					gRobot.rightGun.gunCommand[shootCommand.plantNum].plate += 1;
				else
					gRobot.rightGun.gunCommand[shootCommand.plantNum].ball += 1;
			}
		}
	}
	return shootCommand;
}

/**
  * @brief	Get right gun shoot command FIFO
  * @note
  * @param
  *     @arg
  * @param	`
  * @retval
  */
shoot_command_t ROBOT_RightGunGetShootCommandFIFO(void)
{
	#define RIGHT_NEW_PLATE_NUM 10u
	shoot_command_t shootCommand = {SHOOT_POINT3, INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	cmd_t manualCmd;
	//防止同一个枪连续执行命令
	OSTimeDly(1);
	gRobot.rightGun.commandState = GUN_NO_COMMAND;
	//判断是否没弹
	if(gRobot.rightGun.shootTimes >= RIGHT_BULLET_NUM || gRobot.rightGun.bulletNumber <= GUN_NO_BULLET_ERROR)
	{
		gRobot.rightGun.commandState = GUN_NO_COMMAND;
	}
	else		//有弹
	{
		manualCmd = RightGunOutQueue();
		if(manualCmd.plantNum == INVALID_PLANT_NUMBER)
		{
			gRobot.rightGun.commandState = GUN_NO_COMMAND;		
		}
		else
		{
			shootCommand.plantNum = manualCmd.plantNum;
			shootCommand.shootMethod = manualCmd.method;
			gRobot.rightGun.commandState = GUN_HAVE_COMMAND;
		}
		//左右枪交叉打1#和5#柱子时机械上会有干涉
		if(gRobot.leftGun.targetPlant == PLANT5 && gRobot.leftGun.commandState == GUN_HAVE_COMMAND)
		{
			if(shootCommand.plantNum == PLANT1)
			{
				manualCmd = ReplaceHeadQueue(manualCmd);
				if(manualCmd.plantNum == INVALID_PLANT_NUMBER)
				{
					gRobot.rightGun.commandState = GUN_NO_COMMAND;		
				}
				else
				{
					shootCommand.plantNum = manualCmd.plantNum;
					shootCommand.shootMethod = manualCmd.method;
					gRobot.rightGun.commandState = GUN_HAVE_COMMAND;
				}
			}
		}
		
		if(gRobot.moveBase.actualStopPoint == SHOOT_POINT2)
		{
			//左枪优先打1#、2#;右枪优先打4#、5#
			if(gRobot.leftGun.commandState == GUN_NO_COMMAND && gRobot.leftGun.bulletNumber > GUN_NO_BULLET_ERROR)
			{
				if((shootCommand.plantNum == PLANT1)||(shootCommand.plantNum == PLANT2))
				{
					manualCmd = ReplaceHeadQueue(manualCmd);
					if(manualCmd.plantNum == INVALID_PLANT_NUMBER)
					{
						gRobot.rightGun.commandState = GUN_NO_COMMAND;		
					}
					else
					{
						shootCommand.plantNum = manualCmd.plantNum;
						shootCommand.shootMethod = manualCmd.method;
						gRobot.rightGun.commandState = GUN_HAVE_COMMAND;
					}
				}
			}
		}
		//标记命令开始执行
		if(shootCommand.shootMethod%2)
		{
			gRobot.rightGun.gunCommand[shootCommand.plantNum].plateState = COMMAND_IN_PROCESS;
		}
		else
		{
			gRobot.rightGun.gunCommand[shootCommand.plantNum].ballState = COMMAND_IN_PROCESS;		
		}
	}
	return shootCommand;
}




/**
  * @brief	Get upper gun shoot command
  * @note
  * @param
  *     @arg
  * @param
  * @retval
  */

shoot_command_t ROBOT_UpperGunGetShootCommand(void)
{
	#define UPPER_AUTO_NUM 7u
	uint8_t i = 0u;
	uint8_t searchRange = 4;
	gRobot.upperGun.gunCommand = (plant_t *)gRobot.autoCommand;
	shoot_command_t shootCommand = {SHOOT_POINT3, INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	//为使上枪接收命令更难
	OSTimeDly(5);
	if(gRobot.upperGun.shootTimes >= UPPER_AUTO_NUM)
	{
		searchRange = 4;
		gRobot.upperGun.gunCommand = (plant_t *)gRobot.plantState;
	}
	if(gRobot.moveBase.actualStopPoint == SHOOT_POINT1)
	{
		searchRange = 5;
		gRobot.upperGun.gunCommand = (plant_t *)gRobot.upperLeftCommand;
	}
	if(gRobot.moveBase.actualStopPoint == SHOOT_POINT3)
	{
		searchRange = 5;
		gRobot.upperGun.gunCommand = (plant_t *)gRobot.upperRightCommand;		
	}
	for( i = 0;i < searchRange;i++)
	{
		//有球
		if(gRobot.upperGun.gunCommand[UpperGunPriority[i]].ball >= 1 && UpperGunPriority[i]!=PLANT6)
		{
			shootCommand.plantNum = UpperGunPriority[i];
			shootCommand.shootMethod = SHOOT_METHOD1;
			if(gRobot.upperGun.shootTimes >= UPPER_AUTO_NUM)
			{
				//shootCommand.shootMethod = SHOOT_METHOD4;
			}
			gRobot.upperGun.gunCommand[UpperGunPriority[i]].ball -= 1;
			gRobot.upperGun.commandState = GUN_HAVE_COMMAND;
			gRobot.upperGun.gunCommand[UpperGunPriority[i]].ballState = COMMAND_IN_PROCESS;
			break;
		}
		//没盘
		if(gRobot.upperGun.gunCommand[UpperGunPriority[i]].plate >= 1)
		{
			shootCommand.plantNum = UpperGunPriority[i];
			shootCommand.shootMethod = SHOOT_METHOD2;
			if(gRobot.upperGun.shootTimes >= UPPER_AUTO_NUM)
			{
				//shootCommand.shootMethod = SHOOT_METHOD5;
			}
			gRobot.upperGun.gunCommand[UpperGunPriority[i]].plate -= 1;
			gRobot.upperGun.commandState = GUN_HAVE_COMMAND;
			gRobot.upperGun.gunCommand[UpperGunPriority[i]].plateState = COMMAND_IN_PROCESS;
			break;

		}
		if(i==searchRange-1)
		{
			gRobot.upperGun.ready = GUN_AIM_IN_PROCESS;
			gRobot.upperGun.commandState = GUN_NO_COMMAND;
		}
	}
	//上枪不连续打同一组参数（六号柱除外）
//	if(gRobot.upperGun.lastPlant == shootCommand.plantNum && gRobot.upperGun.lastParaMode == shootCommand.shootMethod \
//		&&shootCommand.plantNum != PLANT6)
//	{
//		if(gRobot.leftGun.bulletNumber > 0 || gRobot.rightGun.bulletNumber > 0)
//		{
//			if(shootCommand.shootMethod%3)
//			{
//				gRobot.upperGun.gunCommand[shootCommand.plantNum].plate += 1;
//			}
//			else
//			{
//				gRobot.upperGun.gunCommand[shootCommand.plantNum].ball += 1;
//			}
//			gRobot.upperGun.commandState = GUN_NO_COMMAND;
//		}
//	}

	return shootCommand;
}
/*
============================================================
				   枪参数变换与逆变换
============================================================
*/

/*
*名称：LeftGunYawTransform
*功能：左枪yaw轴角度转换到位置，结果将发送给其位置环
*参数：
*
*注意：
*/
int32_t LeftGunYawTransform(float yaw)
{
	if(yaw > gRobot.leftGun.maxPoseLimit.yaw)
	{
		gRobot.leftGun.targetPose.yaw = gRobot.leftGun.maxPoseLimit.yaw;
		yaw = gRobot.leftGun.maxPoseLimit.yaw;
	}
	if(yaw < gRobot.leftGun.minPoseLimit.yaw)
	{
		gRobot.leftGun.targetPose.yaw = gRobot.leftGun.minPoseLimit.yaw;
		yaw = gRobot.leftGun.minPoseLimit.yaw;
	}
	return (int32_t)((50.0f + yaw) * 102.4f);
}

/*
*名称：LeftGunYawInverseTransform
*功能：左枪yaw轴位置转换到角度
*参数：
*position:轴的绝对位置pulse
*注意：
*/
float LeftGunYawInverseTransform(int32_t position)
{
	return (float)position / 102.4f - 50.0f;
}

/*
*名称：LeftGunPitchTransform
*功能：左枪pitch轴角度转换到位置，结果将发送给其位置环
*参数：
*
*注意：
*/
int32_t RightGunPitchTransform(float pitch)
{
	if(pitch > gRobot.rightGun.maxPoseLimit.pitch)
	{
		gRobot.rightGun.targetPose.pitch = gRobot.rightGun.maxPoseLimit.pitch;
		pitch = gRobot.rightGun.maxPoseLimit.pitch;
	}
	if(pitch < gRobot.rightGun.minPoseLimit.pitch)
	{
		gRobot.rightGun.targetPose.pitch = gRobot.rightGun.minPoseLimit.pitch;
		pitch = gRobot.rightGun.minPoseLimit.pitch;
	}
	return (int32_t)((pitch - 7.0f) * 141.0844f);
}

/*
*名称：LeftGunPitchInverseTransform
*功能：左枪pitch轴位置转换到角度
*参数：
*position:轴的绝对位置pulse
*注意：
*/
float RightGunPitchInverseTransform(int32_t position)
{
	return (float)position / 141.0844f + 7.0f;
}

/*
*名称：LeftGunRollTransform
*功能：左枪roll轴角度转换到位置，结果将发送给其位置环
*参数：
*
*注意：
*/
int32_t RightGunRollTransform(float roll)
{
	if(roll > gRobot.rightGun.maxPoseLimit.roll)
	{
		gRobot.rightGun.targetPose.roll = gRobot.rightGun.maxPoseLimit.roll;
		roll = gRobot.rightGun.maxPoseLimit.roll;
	}
	if(roll < gRobot.rightGun.minPoseLimit.roll)
	{
		gRobot.rightGun.targetPose.roll = gRobot.rightGun.minPoseLimit.roll;
		roll = gRobot.rightGun.minPoseLimit.roll;
	}
	return (int32_t)((roll - 46.54f) * 141.0844f);
}

/*
*名称：LeftGunRollInverseTransform
*功能：左枪roll轴位置转换到角度
*参数：
*position:轴的绝对位置pulse
*注意：
*/
float RightGunRollInverseTransform(int32_t position)
{
	return (float)position/141.0844f + 46.54f;
}

/*
*名称：LeftGunLeftSpeedTransform
*功能：左枪左传送带速度转换，m/s 到pulse/s
*参数：
*
*注意：
*/
int32_t RightGunLeftSpeedTransform(float speed)
{

	if(speed > gRobot.rightGun.maxPoseLimit.speed1)
	{
		gRobot.rightGun.targetPose.speed1 = gRobot.rightGun.maxPoseLimit.speed1;
		speed = gRobot.rightGun.maxPoseLimit.speed1;
	}
	if(speed < gRobot.rightGun.minPoseLimit.speed1)
	{
		gRobot.rightGun.targetPose.speed1 = gRobot.rightGun.minPoseLimit.speed1;
		speed = gRobot.rightGun.minPoseLimit.speed1;
	}
	return -4096*(int32_t)speed;
}

/*
*名称：LeftGunLeftSpeedInverseTransform
*功能：左枪左传送带速度逆变换，pulse/s到m/s
*参数：
*
*注意：
*/
float RightGunLeftSpeedInverseTransform(int32_t speed)
{
	//fix me, 添加参数合法性检测
	return -(float)speed/4096;
}

/*
*名称：LeftGunRightSpeedTransform
*功能：左枪右传送带速度转换，m/s 到pulse/s
*参数：
*
*注意：
*/
int32_t RightGunRightSpeedTransform(float speed)
{
	if(speed > gRobot.rightGun.maxPoseLimit.speed2)
	{
		gRobot.rightGun.targetPose.speed2 = gRobot.rightGun.maxPoseLimit.speed2;
		speed = gRobot.rightGun.maxPoseLimit.speed2;
	}
	if(speed < gRobot.rightGun.minPoseLimit.speed2)
	{
		gRobot.rightGun.targetPose.speed2 = gRobot.rightGun.minPoseLimit.speed2;
		speed = gRobot.rightGun.minPoseLimit.speed2;
	}
	return 4096*(int32_t)speed;
}

/*
*名称：LeftGunRightSpeedInverseTransform
*功能：左枪右传送带速度逆变换，pulse/s到m/s
*参数：
*
*注意：
*/
float RightGunRightSpeedInverseTransform(int32_t speed)
{
	//fix me, 添加参数合法性检测
	return (float)speed/4096;
}

/*
*名称：RightGunYawTransform
*功能：右枪yaw轴角度转换到位置，结果将发送给其位置环
*参数：
*
*注意：
*/
int32_t RightGunYawTransform(float yaw)
{
	if(yaw > gRobot.rightGun.maxPoseLimit.yaw)
	{
		gRobot.rightGun.targetPose.yaw = gRobot.rightGun.maxPoseLimit.yaw;
		yaw = gRobot.rightGun.maxPoseLimit.yaw;
	}
	if(yaw < gRobot.rightGun.minPoseLimit.yaw)
	{
		gRobot.rightGun.targetPose.yaw = gRobot.rightGun.minPoseLimit.yaw;
		yaw = gRobot.rightGun.minPoseLimit.yaw;
	}
	return (int32_t)((yaw - 50.0f) * 102.4f);
}

/*
*名称：RightGunYawInverseTransform
*功能：右枪yaw轴角度反变换，由脉冲转化为角度
*参数：
*
*注意：
*/
float RightGunYawInverseTransform(int32_t position)
{
	return (float)(position) / 102.4f + 50.0f;
}

/*
*名称：RightGunPitchTransform
*功能：右枪pitch轴角度转换到位置，结果将发送给其位置环
*参数：
*
*注意：
*/
int32_t LeftGunPitchTransform(float pitch)
{
	if(pitch > gRobot.leftGun.maxPoseLimit.pitch)
	{
		gRobot.leftGun.targetPose.pitch = gRobot.leftGun.maxPoseLimit.pitch;
		pitch = gRobot.leftGun.maxPoseLimit.pitch;
	}
	if(pitch < gRobot.leftGun.minPoseLimit.pitch)
	{
		gRobot.leftGun.targetPose.pitch = gRobot.leftGun.minPoseLimit.pitch;
		pitch = gRobot.leftGun.minPoseLimit.pitch;
	}
	return -(int32_t)((pitch - 7.0f) * 141.0844f);
}

/*
*名称：RightGunPitchInverseTransform
*功能：右枪pitch轴角度反变换，由脉冲转化为角度
*参数：
*
*注意：
*/
float LeftGunPitchInverseTransform(int32_t position)
{
	return (float)(-position)/141.0844f + 7.0f;
}

/*
*名称：RightGunRollTransform
*功能：右枪roll轴角度转换到位置，结果将发送给其位置环
*参数：
*
*注意：
*/
int32_t LeftGunRollTransform(float roll)
{
	if(roll > gRobot.leftGun.maxPoseLimit.roll)
	{
		gRobot.leftGun.targetPose.roll = gRobot.leftGun.maxPoseLimit.roll;
		roll = gRobot.leftGun.maxPoseLimit.roll;
	}
	if(roll < gRobot.leftGun.minPoseLimit.roll)
	{
		gRobot.leftGun.targetPose.roll = gRobot.leftGun.minPoseLimit.roll;
		roll = gRobot.leftGun.minPoseLimit.roll;
	}
	return -(int32_t)((roll - 46.54f) * 141.0844f);
}

/*
*名称：RightGunRollInverseTransform
*功能：右枪roll轴角度反变换，由脉冲转化为角度
*参数：
*
*注意：
*/
float LeftGunRollInverseTransform(int32_t position)
{
	return (float)(-position)/141.0844f + 46.54f;
}

/*
*名称：RightGunLeftSpeedTransform
*功能：右枪左传送带速度转换，由转每秒转化为脉冲/s
*参数：
*
*注意：
*/
int32_t LeftGunLeftSpeedTransform(float speed)
{
	if(speed > gRobot.leftGun.maxPoseLimit.speed1)
	{
		gRobot.leftGun.targetPose.speed1 = gRobot.leftGun.maxPoseLimit.speed1;
		speed = gRobot.leftGun.maxPoseLimit.speed1;
	}
	if(speed < gRobot.leftGun.minPoseLimit.speed1)
	{
		gRobot.leftGun.targetPose.speed1 = gRobot.leftGun.minPoseLimit.speed1;
		speed = gRobot.leftGun.minPoseLimit.speed1;
	}
	return -4096*(int32_t)speed;
}

/*
*名称：RightGunLeftSpeedInverseTransform
*功能：右枪左传送带速度反变换，由脉冲转化为转每秒
*参数：
*
*注意：
*/
float LeftGunLeftSpeedInverseTransform(int32_t speed)
{
	return -(float)speed / 4096.0f;
}

/*
*名称：RightGunRightSpeedTransform
*功能：右枪右传送带速度转换， 由转每秒转化为脉冲/s
*
*注意：
*/
int32_t LeftGunRightSpeedTransform(float speed)
{
	if(speed > gRobot.leftGun.maxPoseLimit.speed2)
	{
		gRobot.leftGun.targetPose.speed2 = gRobot.leftGun.maxPoseLimit.speed2;
		speed = gRobot.leftGun.maxPoseLimit.speed2;
	}
	if(speed < gRobot.leftGun.minPoseLimit.speed2)
	{
		gRobot.leftGun.targetPose.speed2 = gRobot.leftGun.minPoseLimit.speed2;
		speed = gRobot.leftGun.minPoseLimit.speed2;
	}
	return 4096 * (int32_t)speed;
}

/*
*名称：RightGunRightSpeedInverseTransform
*功能：右枪左传送带速度反变换，由脉冲转化为转每秒
*参数：
*
*注意：
*/
float LeftGunRightSpeedInverseTransform(int32_t speed)
{
	return (float)speed / 4096.0f;
}
/*
*名称：UpperGunYawTransform
*功能：上面枪yaw轴角度转换到位置，结果将发送给其位置环
*参数：
*
*注意：
*/
int32_t UpperGunYawTransform(float yaw)
{
	if(yaw > gRobot.upperGun.maxPoseLimit.yaw)
	{
		gRobot.upperGun.targetPose.yaw =  gRobot.upperGun.maxPoseLimit.yaw;
		yaw = gRobot.upperGun.maxPoseLimit.yaw;
	}
	if(yaw < gRobot.upperGun.minPoseLimit.yaw)
	{
		gRobot.upperGun.targetPose.yaw =  gRobot.upperGun.minPoseLimit.yaw;
		yaw = gRobot.upperGun.minPoseLimit.yaw;
	}
	return (int32_t)((50.0f + yaw) * 102.4f);
}

/*
*名称：UpperGunYawInverseTransform
*功能：上枪yaw轴角度反变换，由脉冲转化为角度
*参数：
*
*注意：
*/
float UpperGunYawInverseTransform(int32_t position)
{
	return (float)position / 102.4f - 50.0f;
}

/*
*名称：UpperGunPitchTransform
*功能：上面枪pitch轴角度转换到位置，结果将发送给其位置环
*参数：
*
*注意：
*/
int32_t UpperGunPitchTransform(float pitch)
{
	if(pitch > gRobot.upperGun.maxPoseLimit.pitch)
	{
		gRobot.upperGun.targetPose.pitch = gRobot.upperGun.maxPoseLimit.pitch;
		pitch = gRobot.upperGun.maxPoseLimit.pitch;
	}
	if(pitch < gRobot.upperGun.minPoseLimit.pitch)
	{
		gRobot.upperGun.targetPose.pitch = gRobot.upperGun.minPoseLimit.pitch;
		pitch = gRobot.upperGun.minPoseLimit.pitch;
	}
	return (int32_t)(-(10.0f + pitch) * 141.0844f);
}

/*
*名称：RightGunPitchInverseTransform
*功能：上枪pitch轴角度反变换，由脉冲转化为角度
*参数：
*
*注意：
*/
float UpperGunPitchInverseTransform(int32_t position)
{
	return (float)-position/141.0844f - 10.0f;
}

/*
*名称：UpperGunLeftSpeedTransform
*功能：上枪左传送带速度转化函数
*参数：
*
*注意：
*/
int32_t UpperGunLeftSpeedTransform(float speed)
{
	if(speed > gRobot.upperGun.maxPoseLimit.speed1)
	{
		gRobot.upperGun.targetPose.speed1 = gRobot.upperGun.maxPoseLimit.speed1;
		speed = gRobot.upperGun.maxPoseLimit.speed1;
	}
	if(speed < gRobot.upperGun.minPoseLimit.speed1)
	{
		gRobot.upperGun.targetPose.speed1 = gRobot.upperGun.minPoseLimit.speed1;
		speed = gRobot.upperGun.minPoseLimit.speed1;
	}
	return -4096*(int32_t)speed;
}

/*
*名称：UpperGunLeftSpeedInverseTransform
*功能：上枪左传送带速度逆变换，pulse/s到m/s
*参数：
*
*注意：
*/
float UpperGunLeftSpeedInverseTransform(int32_t speed)
{
	return -(float)speed/4096;
}
/*
*名称：UpperGunRightSpeedTransform
*功能：上枪左传送带速度转化函数
*参数：
*
*注意：
*/
int32_t UpperGunRightSpeedTransform(float speed)
{
	if(speed > gRobot.upperGun.maxPoseLimit.speed2)
	{
		gRobot.upperGun.targetPose.speed2 = gRobot.upperGun.maxPoseLimit.speed2;
		speed = gRobot.upperGun.maxPoseLimit.speed2;
	}
	if(speed < gRobot.upperGun.minPoseLimit.speed2)
	{
		gRobot.upperGun.targetPose.speed2 = gRobot.upperGun.minPoseLimit.speed2;
		speed = gRobot.upperGun.minPoseLimit.speed2;
	}
	return 4096*(int32_t)speed;
}

/*
*名称：UpperGunRightSpeedInverseTransform
*功能：上枪左传送带速度逆变换，pulse/s到m/s
*参数：
*
*注意：
*/
float UpperGunRightSpeedInverseTransform(int32_t speed)
{
	return (float)speed/4096;
}
/*
============================================================
				   机器人动作流程函数
============================================================
*/



/*
*名称：ROBOT_GunLoad
*功能：安装弹夹，即抓取子弹过程
*参数：
*status:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹，因为是手动上弹
*/
status_t ROBOT_GunLoad(void)
{
	ClampClose();

	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_GunOpenSafety
*功能：拉开枪保险，子弹安装好后才能进行此步骤
*参数：
*status:GUN_NO_ERROR，GUN_OPEN_SAFETY_ERROR
*注意：上面的枪不需要
*/
status_t ROBOT_GunOpenSafety(void)
{
	ClampRotate();
	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_CheckGunOpenSafety
*功能：检查拉开枪保险
*参数：
*status:GUN_NO_ERROR，GUN_OPEN_SAFETY_ERROR
*注意：上面的枪不需要
*/
status_t ROBOT_CheckGunOpenSafety(void)
{
	int *msg = (int *)GUN_OPENSAFTY_READY;
	if(KEYSWITCH)
	{
		OSMboxPostOpt(OpenSaftyMbox , msg , OS_POST_OPT_BROADCAST);
	}
	return GUN_NO_ERROR;
}

/**
*名称：ROBOT_LeftGunReload
*功能：左枪上弹，每次射击前需要上弹一次
*		通过发送CAN命令给间隔一段时间后
*@param None
*@retval status_t:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹
*/
status_t ROBOT_LeftGunReload(void)
{
//	uint8_t pushTimes = 10;
	if(gRobot.leftGun.reloadState == GUN_NOT_RELOAD)
	{
		if(gRobot.leftGun.lastPlant == PLANT7)
		{
			if(!PHOTOSENSORLEFTGUN)
			{
				LeftPush();
				OSTimeDly(6);
			}
		}
		if(gRobot.leftGun.shootTimes == 0)
		{
//			while(pushTimes--)
//			{
//				LeftPush();
//				OSTimeDly(2);
//				LeftHold();
//				OSTimeDly(8);
//			}
//			OSTimeDly(50);
		}
		else
		{
			OSTimeDly(24);
		}
//		if(gRobot.leftGun.shootTimes == 0)
//		{
//			pushTimes = 7;
//		}

		LeftBack();
//		OSTimeDly(10);
//		OSTimeDly(50);
//		LeftHold();
		gRobot.leftGun.reloadState = GUN_ALREADY_RELOAD;
	}
	return GUN_NO_ERROR;
}
/**
*名称：ROBOT_RightGunReload
*功能：右枪上弹，每次射击前需要上弹一次
*		通过发送CAN命令给间隔一段时间后
*@param None
*@retval status_t:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹
*/
status_t ROBOT_RightGunReload(void)
{
//	uint8_t pushTimes = 10;
	if(gRobot.rightGun.reloadState == GUN_NOT_RELOAD)
	{
		if(gRobot.rightGun.lastPlant ==PLANT7)
		{
			if(!PHOTOSENSORRIGHTGUN)
			{
				RightPush();
				OSTimeDly(6);
			}
		}
		if(gRobot.rightGun.shootTimes == 0)
		{
//			while(pushTimes--)
//			{
//				RightPush();
//				OSTimeDly(2);
//				RightHold();
//				OSTimeDly(8);
//			}
//			OSTimeDly(50);
		}
		else
		{
			OSTimeDly(24);
		}

		RightBack();
//		OSTimeDly(10);//看似无用

//		OSTimeDly(50);
//		RightHold();
		gRobot.rightGun.reloadState = GUN_ALREADY_RELOAD;
	}
	return GUN_NO_ERROR;

}
/**
*名称：ROBOT_LeftGunCheckReload
*功能：检查左枪上弹情况
*@param None
*@retval status_t:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹
*/
status_t ROBOT_LeftGunCheckReload(void)
{
	static int8_t reloadErrorTimes = 0;
	uint8_t noPlateTimes = 0;
	uint8_t checkTimes = 10;
	while(checkTimes--)
	{
		if(!PHOTOSENSORLEFTGUN)
		{
			noPlateTimes ++;
		}
		OSTimeDly(1);
	}
	if(noPlateTimes>=8)
	{
		checkTimes = 10;
		noPlateTimes  = 0;
		while(checkTimes--)
		{
			if(!PHOTOSENSORLEFTGUN)
			{
				noPlateTimes ++;
			}
			OSTimeDly(1);
		}
		if(noPlateTimes >= 8)
		{
			gRobot.leftGun.champerErrerState = GUN_RELOAD_ERROR;
			reloadErrorTimes++;
		}
		else
		{
			gRobot.leftGun.champerErrerState = GUN_RELOAD_OK;			
		}
	}
	else
	{
		gRobot.leftGun.champerErrerState = GUN_RELOAD_OK;
	}
	if(reloadErrorTimes >= 2)
	{
		gRobot.leftGun.champerErrerState = GUN_RELOAD_OK;
		reloadErrorTimes = 0;
	}
	return GUN_NO_ERROR;
}

/**
*名称：ROBOT_LeftGunCheckReload
*功能：检查右枪上弹情况
*@param None
*@retval status_t:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹
*/
status_t ROBOT_RightGunCheckReload(void)
{
	static int8_t reloadErrorTimes = 0;
	uint8_t noPlateTimes = 0;
	uint8_t checkTimes = 10;
	while(checkTimes--)
	{
		if(!PHOTOSENSORRIGHTGUN)
		{
			noPlateTimes ++;
		}
		OSTimeDly(1);
	}
	if(noPlateTimes>=8)
	{
		checkTimes = 10;
		noPlateTimes = 0;
		while(checkTimes--)
		{
			if(!PHOTOSENSORRIGHTGUN)
			{
				noPlateTimes ++;
			}
			OSTimeDly(1);
		}
		if(noPlateTimes >= 8)
		{
			gRobot.rightGun.champerErrerState = GUN_RELOAD_ERROR;
			reloadErrorTimes++;
		}
		else
		{
			gRobot.rightGun.champerErrerState = GUN_RELOAD_OK;		
		}
	}
	else
	{
		gRobot.rightGun.champerErrerState = GUN_RELOAD_OK;
	}
	if(reloadErrorTimes >= 2)
	{
		gRobot.rightGun.champerErrerState = GUN_RELOAD_OK;
		reloadErrorTimes = 0;
	}
	return GUN_NO_ERROR;
}

/** @defgroup Left_Gun_Shoot_Tragedy
  * @brief
  * @{
  */



/**
  * @}
  */
/**
*名称： ROBOT_LeftGunAim
*功能： 瞄准，目标改变后需要先调用此接口来重新瞄准,此函数将发送CAN命令
*	上面的枪目前机械上没有roll，没有右侧传送带speed2
*	故 ROBOT_LeftGunAim 和 ROBOT_LeftGunAim 各发送5条CAN命令
*	ROBOT_UpperGunAim仅发送3条CAN命令
*参数： None
* @retval status:GUN_NO_ERROR
*/
status_t ROBOT_LeftGunAim(void)
{

	gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;

	PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(gRobot.leftGun.targetPose.yaw));
	PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(gRobot.leftGun.targetPose.pitch));
	PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(gRobot.leftGun.targetPose.roll));

	VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(gRobot.leftGun.targetPose.speed1));
	VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(gRobot.leftGun.targetPose.speed2));

	return GUN_NO_ERROR;
}
status_t ROBOT_LeftGunReloadAim(void)
{

	gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;

	PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(gRobot.leftGun.reloadPose.yaw));
	PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(gRobot.leftGun.reloadPose.pitch));
	PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(gRobot.leftGun.reloadPose.roll));

	VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(gRobot.leftGun.reloadPose.speed1));
	VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(gRobot.leftGun.reloadPose.speed2));

	return GUN_NO_ERROR;
}
/** @defgroup Right_Gun_Shoot_Tragedy
  * @brief
  * @{
  */




/**
  * @}
  */

/**
  *名称： ROBOT_RightGunAim
  *功能： 瞄准，目标改变后需要先调用此接口来重新瞄准,此函数将发送CAN命令
  *	上面的枪目前机械上没有roll，没有右侧传送带speed2
  *	故 ROBOT_LeftGunAim 和 ROBOT_LeftGunAim 各发送5条CAN命令
  *	ROBOT_UpperGunAim仅发送3条CAN命令
  *参数： None
  * @retval status:GUN_NO_ERROR
  */
status_t ROBOT_RightGunAim(void)
{
	//这里应该保证枪膛里有子弹！！！,fix me，检测参数合法性
			gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
			PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(gRobot.rightGun.targetPose.yaw));
			PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(gRobot.rightGun.targetPose.pitch));
			PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(gRobot.rightGun.targetPose.roll));

			VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(gRobot.rightGun.targetPose.speed1));
			VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(gRobot.rightGun.targetPose.speed2));

	return GUN_NO_ERROR;
}
status_t ROBOT_RightGunReloadAim(void)
{
	//这里应该保证枪膛里有子弹！！！,fix me，检测参数合法性
			gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
			PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(gRobot.rightGun.reloadPose.yaw));
			PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(gRobot.rightGun.reloadPose.pitch));
			PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(gRobot.rightGun.reloadPose.roll));

			VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(gRobot.rightGun.reloadPose.speed1));
			VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(gRobot.rightGun.reloadPose.speed2));

	return GUN_NO_ERROR;
}

/**
*名称： ROBOT_UpperGunAim
*功能： 瞄准，目标改变后需要先调用此接口来重新瞄准,此函数将发送CAN命令
*	上面的枪目前机械上没有roll，没有右侧传送带speed2
*	故 ROBOT_LeftGunAim 和 ROBOT_LeftGunAim 各发送5条CAN命令
*	ROBOT_UpperGunAim仅发送3条CAN命令
*参数： None
* @retval status:GUN_NO_ERROR
*/
status_t ROBOT_UpperGunAim(void)
{
	//这里应该保证枪膛里有子弹！！！,fix me，检测参数合法性
	gRobot.upperGun.ready = GUN_AIM_IN_PROCESS;
	PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(gRobot.upperGun.targetPose.yaw));
	PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(gRobot.upperGun.targetPose.pitch));

	VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(gRobot.upperGun.targetPose.speed1));
	VelCrl(CAN1, UPPER_GUN_RIGHT_ID, UpperGunRightSpeedTransform(gRobot.upperGun.targetPose.speed2));
	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_LeftGunCheckAim
*功能：检查瞄准是否已完成，不同枪分开检测为了防止重入，
*因为此函数中需要设计超时
*参数：
*none
*status:GUN_AIM_IN_PROCESS， GUN_AIM_DONE
*注意：
*/
status_t ROBOT_LeftGunCheckAim(void)
{
	//左枪到位标准
	#define LEFT_READY_STANDARD (5u)
	//左枪超时时间 单位为LEFT_SAMPLING_PERIOD
	#define LEFT_TIME_OUT (50u)
	//左枪位置检测采样周期 单位为系统tick
	#define LEFT_SAMPLIING_PERIOD (4u)
	int checkTime = 0;
	//超时时间为50*4*10ms，2秒
	int timeout = LEFT_TIME_OUT;
	uint8_t leftGunReadyTimes = 0;
	
	BEEP_OFF;
	while(timeout--)
	{
		//每次检测前对之前的数据复位
		gRobot.leftGun.actualPose.pitch = 0.0f;
		gRobot.leftGun.actualPose.roll = 0.0f;
		gRobot.leftGun.actualPose.yaw = 0.0f;
		gRobot.leftGun.actualPose.speed1 = 0.0f;
		gRobot.leftGun.actualPose.speed2 = 0.0f;
		ReadActualPos(CAN1, LEFT_GUN_GROUP_ID);
		ReadActualVel(CAN1, LEFT_GUN_VEL_GROUP_ID);
		//检查命令状态是否发生改变
		if(gRobot.leftGun.gunCommand == gRobot.autoCommand || gRobot.isBleOk.noBleFlag == BLE_LOST)
		{
			if(gRobot.leftGun.targetPlant==PLANT1 ||gRobot.leftGun.targetPlant==PLANT2||\
				gRobot.leftGun.targetPlant==PLANT4 ||gRobot.leftGun.targetPlant==PLANT5)
			{
				if(gRobot.leftGun.shootParaMode%2)
				{
					if(gRobot.cameraInfo[gRobot.leftGun.targetPlant].plate == 0)
					{
						gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;
						break;
					}
				}
				else
				{
					if(gRobot.cameraInfo[gRobot.leftGun.targetPlant].ball == 0)
					{
						gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;
						break;
					}
				}
			}
		}
		OSTimeDly(LEFT_SAMPLIING_PERIOD);
		//减少了发送的数据
//		LeftGunSendDebugInfo();
		//fix me,检查枪位姿是否到位，后面需要在枪结构体中增加可容忍误差，然后封装成函数检测
		if(gRobot.leftGun.actualPose.pitch > gRobot.leftGun.targetPose.pitch + 0.5f ||\
			gRobot.leftGun.actualPose.pitch < gRobot.leftGun.targetPose.pitch - 0.5f)
		{
			continue;
		}

		if(gRobot.leftGun.actualPose.roll > gRobot.leftGun.targetPose.roll + 0.5f ||\
			gRobot.leftGun.actualPose.roll < gRobot.leftGun.targetPose.roll - 0.5f)
		{
			continue;
		}

		if(gRobot.leftGun.actualPose.yaw > gRobot.leftGun.targetPose.yaw + 0.5f ||\
			gRobot.leftGun.actualPose.yaw < gRobot.leftGun.targetPose.yaw - 0.5f)
		{
			continue;
		}
		if(gRobot.leftGun.actualPose.speed1 > gRobot.leftGun.targetPose.speed1 + 1.0f||\
			gRobot.leftGun.actualPose.speed1 < gRobot.leftGun.targetPose.speed1 - 1.0f)
		{
			continue;
		}
		if(gRobot.leftGun.actualPose.speed2 > gRobot.leftGun.targetPose.speed2 + 1.0f ||\
			gRobot.leftGun.actualPose.speed2 < gRobot.leftGun.targetPose.speed2 - 1.0f)
		{
			continue;
		}
		//运行到这里，表示都满足指标，跳出循环
		leftGunReadyTimes++;
		if(leftGunReadyTimes > LEFT_READY_STANDARD)
		{
			break;
		}
	}
	checkTime = (LEFT_TIME_OUT-timeout)*LEFT_SAMPLIING_PERIOD;
	if(checkTime > (LEFT_TIME_OUT * LEFT_SAMPLIING_PERIOD))
	{
		BEEP_ON;
		UART5_OUT((uint8_t *)"Left Gun Check Time Out !!!\r\n");
	}
	gRobot.leftGun.checkTimeUsage = checkTime;
	if(leftGunReadyTimes > LEFT_READY_STANDARD || checkTime > (LEFT_TIME_OUT * LEFT_SAMPLIING_PERIOD))
	{
		gRobot.leftGun.ready = GUN_AIM_DONE;
	}
	return GUN_NO_ERROR;
}


status_t ROBOT_LeftGunCheckReloadAim(void)
{
	//超时时间为20*5*10ms，1秒
	uint8_t checkTimes = 2;
	int checkTime = 0;
	while(checkTimes--)
	{
		int timeout = 20;
		while(timeout--)
		{
			ReadActualPos(CAN1, LEFT_GUN_GROUP_ID);
			ReadActualVel(CAN1, LEFT_GUN_VEL_GROUP_ID);
			OSTimeDly(5);
			LeftGunSendDebugInfo();
			//fix me,检查枪位姿是否到位，后面需要在枪结构体中增加可容忍误差，然后封装成函数检测
			if(gRobot.leftGun.actualPose.pitch > gRobot.leftGun.reloadPose.pitch + 0.5f ||\
				gRobot.leftGun.actualPose.pitch < gRobot.leftGun.reloadPose.pitch - 0.5f)
			{
				continue;
			}

			if(gRobot.leftGun.actualPose.roll > gRobot.leftGun.reloadPose.roll + 0.5f ||\
				gRobot.leftGun.actualPose.roll < gRobot.leftGun.reloadPose.roll - 0.5f)
			{
				continue;
			}

			if(gRobot.leftGun.actualPose.yaw > gRobot.leftGun.reloadPose.yaw + 0.5f ||\
				gRobot.leftGun.actualPose.yaw < gRobot.leftGun.reloadPose.yaw - 0.5f)
			{
				continue;
			}

			if(gRobot.leftGun.actualPose.speed1 > gRobot.leftGun.reloadPose.speed1 + 1.0f||\
				gRobot.leftGun.actualPose.speed1 < gRobot.leftGun.reloadPose.speed1 - 1.0f)
			{
				continue;
			}
			if(gRobot.leftGun.actualPose.speed2 > gRobot.leftGun.reloadPose.speed2 + 1.0f ||\
				gRobot.leftGun.actualPose.speed2 < gRobot.leftGun.reloadPose.speed2 - 1.0f)
			{
				continue;
			}

			//运行到这里，表示都满足指标，跳出循环
			break;
		}
		checkTime += (20-timeout)*5;
	}
	if(checkTime > 200)
	{
		UART5_OUT((uint8_t *)"Left Gun  Reload Check Time Out !!!\r\n");
	}
	gRobot.leftGun.checkTimeUsage = checkTime;
	gRobot.leftGun.ready = GUN_AIM_DONE;
	return GUN_NO_ERROR;
}
/*
*名称：ROBOT_RightGunCheckAim
*功能：检查瞄准是否已完成，不同枪分开检测为了防止重入，
*此函数中需要设计超时因为防止过长等待
*参数：
*none
*status:GUN_AIM_IN_PROCESS， GUN_AIM_DONE
*注意：
*/
 status_t ROBOT_RightGunCheckAim(void)
{
	//右枪到位标准
	#define RIGHT_READY_STANDARD (5u)
	//右枪超时时间 单位为RIGHT_SAMPLING_PERIOD
	#define RIGHT_TIME_OUT (50u)
	//右枪位置检测采样周期 单位为系统tick
	#define RIGHT_SAMPLIING_PERIOD (4u)
	int checkTime = 0;
	//超时时间为50*4*10ms，2秒
	int timeout = RIGHT_TIME_OUT;
	uint8_t rightGunReadyTimes = 0;

	BEEP_OFF;
	while(timeout--)
	{
		//每次检测前对之前的数据复位
		gRobot.rightGun.actualPose.pitch = 0.0f;
		gRobot.rightGun.actualPose.roll = 0.0f;
		gRobot.rightGun.actualPose.yaw = 0.0f;
		gRobot.rightGun.actualPose.speed1 = 0.0f;
		gRobot.rightGun.actualPose.speed2 = 0.0f;
		ReadActualPos(CAN1, RIGHT_GUN_GROUP_ID);
		ReadActualVel(CAN1, RIGHT_GUN_VEL_GROUP_ID);
		//检查命令状态是否发生改变
		if(gRobot.rightGun.gunCommand == gRobot.autoCommand||gRobot.isBleOk.noBleFlag == BLE_LOST)
		{
			if(gRobot.rightGun.targetPlant==PLANT1 ||gRobot.rightGun.targetPlant==PLANT2||\
				gRobot.rightGun.targetPlant==PLANT4 ||gRobot.rightGun.targetPlant==PLANT5)
			{
				if(gRobot.rightGun.shootParaMode%2)
				{
					if(gRobot.cameraInfo[gRobot.rightGun.targetPlant].plate == 0)
					{
						gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
						break;
					}
				}
				else
				{
					if(gRobot.cameraInfo[gRobot.rightGun.targetPlant].ball == 0)
					{
						gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
						break;
					}
				}
			}
		}
		OSTimeDly(RIGHT_SAMPLIING_PERIOD);
		//减少了发送的数据
//		RightGunSendDebugInfo();
		//fix me,检查枪位姿是否到位，后面需要在枪结构体中增加可容忍误差，然后封装成函数检测
		if(gRobot.rightGun.actualPose.pitch > gRobot.rightGun.targetPose.pitch + 0.5f ||\
			gRobot.rightGun.actualPose.pitch < gRobot.rightGun.targetPose.pitch - 0.5f)
		{
			continue;
		}

		if(gRobot.rightGun.actualPose.roll > gRobot.rightGun.targetPose.roll + 0.5f ||\
			gRobot.rightGun.actualPose.roll < gRobot.rightGun.targetPose.roll - 0.5f)
		{
			continue;
		}

		if(gRobot.rightGun.actualPose.yaw > gRobot.rightGun.targetPose.yaw + 0.5f ||\
			gRobot.rightGun.actualPose.yaw < gRobot.rightGun.targetPose.yaw - 0.5f)
		{
			continue;
		}
		if(gRobot.rightGun.actualPose.speed1 > gRobot.rightGun.targetPose.speed1 + 1.0f||\
			gRobot.rightGun.actualPose.speed1 < gRobot.rightGun.targetPose.speed1 - 1.0f)
		{
			continue;
		}
		if(gRobot.rightGun.actualPose.speed2 > gRobot.rightGun.targetPose.speed2 + 1.0f ||\
			gRobot.rightGun.actualPose.speed2 < gRobot.rightGun.targetPose.speed2 - 1.0f)
		{
			continue;
		}
		//运行到这里，表示都满足指标，跳出循环
		rightGunReadyTimes++;
		if(rightGunReadyTimes > RIGHT_READY_STANDARD)
		{
			break;
		}
	}
	checkTime = (RIGHT_TIME_OUT-timeout)*RIGHT_SAMPLIING_PERIOD;
	if(checkTime > (RIGHT_TIME_OUT * RIGHT_SAMPLIING_PERIOD))
	{
		BEEP_ON;
		UART5_OUT((uint8_t *)"Right Gun Check Time Out !!!\r\n");
	}
	gRobot.rightGun.checkTimeUsage = checkTime;
	if(rightGunReadyTimes > RIGHT_READY_STANDARD || checkTime > (RIGHT_TIME_OUT * RIGHT_SAMPLIING_PERIOD))
	{
		gRobot.rightGun.ready = GUN_AIM_DONE;
	}
	return GUN_NO_ERROR;
}
status_t ROBOT_RightGunCheckReloadAim(void)
{
	//超时时间为20*5*10ms，1秒
	uint8_t checkTimes = 2;
	int checkTime = 0;
	while(checkTimes--)
	{
		int timeout = 20;
		while(timeout--)
		{
			//fix me 三轴位置已经支持组ID，组ID在robot.h中定义
			ReadActualPos(CAN1,RIGHT_GUN_GROUP_ID);
			ReadActualVel(CAN1,RIGHT_GUN_VEL_GROUP_ID);
			OSTimeDly(5);
			RightGunSendDebugInfo();
			//fix me,检查枪位姿是否到位，后面需要在枪结构体中增加可容忍误差，然后封装成函数检测
			if(gRobot.rightGun.actualPose.pitch > gRobot.rightGun.reloadPose.pitch + 0.5f ||\
				gRobot.rightGun.actualPose.pitch < gRobot.rightGun.reloadPose.pitch - 0.5f)
			{
				continue;
			}

			if(gRobot.rightGun.actualPose.roll > gRobot.rightGun.reloadPose.roll + 0.5f ||\
				gRobot.rightGun.actualPose.roll < gRobot.rightGun.reloadPose.roll - 0.5f)
			{
				continue;
			}

			if(gRobot.rightGun.actualPose.yaw > gRobot.rightGun.reloadPose.yaw + 0.5f ||\
				gRobot.rightGun.actualPose.yaw < gRobot.rightGun.reloadPose.yaw - 0.5f)
			{
				continue;
			}

			if(gRobot.rightGun.actualPose.speed1 > gRobot.rightGun.reloadPose.speed1 + 1.0f ||\
				gRobot.rightGun.actualPose.speed1 < gRobot.rightGun.reloadPose.speed1 - 1.0f)
			{
				continue;
			}
			if(gRobot.rightGun.actualPose.speed2 > gRobot.rightGun.reloadPose.speed2 + 1.0f ||\
				gRobot.rightGun.actualPose.speed2 < gRobot.rightGun.reloadPose.speed2 - 1.0f)
			{
				continue;
			}

			//运行到这里，表示都满足指标，跳出循环
			break;
		}
		checkTime+=(20-timeout)*5;
	}
	if(checkTime > 200)
	{
		UART5_OUT((uint8_t *)"Right Gun Reload Check Time Out !!!\r\n");
	}
	gRobot.rightGun.checkTimeUsage = checkTime;
	gRobot.rightGun.ready = GUN_AIM_DONE;
	return GUN_NO_ERROR;
}
/*
*名称：ROBOT_UpperGunCheckAim
*功能：检查瞄准是否已完成，不同枪分开检测为了防止重入，
*因为此函数中需要设计超时
*参数：
*none
*status:GUN_AIM_IN_PROCESS， GUN_AIM_DONE
*注意：
*/
status_t ROBOT_UpperGunCheckAim(void)
{

	uint8_t checkTimes = 5;
	int checkTime = 0;
	BEEP_OFF;
	if(gRobot.upperGun.mode==GUN_DEFEND_MODE)checkTimes = 1;
	if(gRobot.upperGun.targetPlant == PLANT6)
	{
		checkTimes = 10;
	}
	while(checkTimes--)
	{
		//超时时间为20*5*10ms，1秒
		int timeout = 20;
		while(timeout--)
		{
			//检查防守台上盘状态的变化，如果有改变立即跳出循环重新打盘
			//fix me 耦合太高
			if(gRobot.upperGun.mode == GUN_ATTACK_MODE)
			{
				if(gRobot.upperGun.defendZone1 & 0x0f)
				{
					break;
				}
			}
			if(gRobot.upperGun.mode == GUN_DEFEND_MODE)
			{
				if (gRobot.upperGun.presentDefendZoneId != gRobot.upperGun.defendZone1 - 0x01 &&
					gRobot.upperGun.presentDefendZoneId != gRobot.upperGun.defendZone2 - 0x01)
				{
					gRobot.upperGun.shoot = GUN_STOP_SHOOT;
					return GUN_NO_READY_ERROR;
				}
			}
			//fix me 三轴位置已经支持组ID，组ID在robot.h中定义
			ReadActualPos(CAN1, UPPER_GUN_GROUP_ID);
			ReadActualVel(CAN1, UPPER_GUN_VEL_GROUP_ID);
			OSTimeDly(5);
			//减少了瞄准时发送的数据
//			UpperGunSendDebugInfo();

			//fix me,检查枪位姿是否到位，后面需要在枪结构体中增加可容忍误差，然后封装成函数检测
			if(gRobot.upperGun.actualPose.pitch > gRobot.upperGun.targetPose.pitch + 0.5f ||\
				gRobot.upperGun.actualPose.pitch < gRobot.upperGun.targetPose.pitch - 0.5f)
			{
				continue;
			}

			if(gRobot.upperGun.actualPose.yaw > gRobot.upperGun.targetPose.yaw + 0.2f ||\
				gRobot.upperGun.actualPose.yaw < gRobot.upperGun.targetPose.yaw - 0.2f)
			{
				continue;
			}
			if(gRobot.upperGun.actualPose.speed1 > gRobot.upperGun.targetPose.speed1 +1.5f ||\
				gRobot.upperGun.actualPose.speed1 < gRobot.upperGun.targetPose.speed1 -1.5f)
			{
				continue;
			}
			if(gRobot.upperGun.actualPose.speed2 > gRobot.upperGun.targetPose.speed2 +1.5f ||\
				gRobot.upperGun.actualPose.speed2 < gRobot.upperGun.targetPose.speed2 -1.5f)
			{
				continue;
			}			
			break;
		}
		checkTime += (20 - timeout) * 5;
	}
	if(gRobot.upperGun.mode == GUN_DEFEND_MODE)
	{
		if(checkTime > 100)
		{
			BEEP_ON;
			UART5_OUT((uint8_t *)"Upper Gun Check Time Out !!!\r\n");
		}
	}
	else
	{
		if(checkTime >= 500)
		{
			BEEP_ON;
			UART5_OUT((uint8_t *)"Upper Gun Check Time Out !!!\r\n");
		}
	}
	if(gRobot.upperGun.mode == GUN_DEFEND_MODE)
	{
		gRobot.upperGun.shoot = GUN_START_SHOOT;
		if (gRobot.upperGun.shoot == GUN_START_SHOOT)
		{
			gRobot.upperGun.ready = GUN_AIM_DONE;
		}
	}
	else
	{
		gRobot.upperGun.ready = GUN_AIM_DONE;
	}

	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_LeftGunCheckShootPoint
*功能：检查底盘是否走到位
*参数：
*none
*status:
*注意：
*/
 status_t ROBOT_LeftGunCheckShootPoint(void)
{
	CPU_INT08U  os_err;
	if((gRobot.leftGun.shootTimes == 0 && gRobot.leftGun.champerErrerState == GUN_RELOAD_OK)\
		||gRobot.isReset == ROBOT_RESET||gRobot.isReload == ROBOT_RELOAD||gRobot.moveBase.actualStopPoint == SHOOT_POINT_MOVING)
	{
		OSMboxPend(LeftGunShootPointMbox,0,&os_err);
		OSTimeDly(20);
		return MOVEBASE_POS_READY;
	}

	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_RightGunCheckShootPoint
*功能：检查底盘是否走到位
*参数：
*none
*status:
*注意：
*/
 status_t ROBOT_RightGunCheckShootPoint(void)
{
	CPU_INT08U  os_err;

	if((gRobot.rightGun.shootTimes == 0 && gRobot.rightGun.champerErrerState == GUN_RELOAD_OK)\
		||gRobot.isReset == ROBOT_RESET||gRobot.isReload == ROBOT_RELOAD||gRobot.moveBase.actualStopPoint == SHOOT_POINT_MOVING)
	{
		OSMboxPend(RightGunShootPointMbox,0,&os_err);
		OSTimeDly(20);
		return MOVEBASE_POS_READY;
	}
	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_UpperGunCheckShootPoint
*功能：检查底盘是否走到位
*参数：
*none
*status:
*注意：
*/
 status_t ROBOT_UpperGunCheckShootPoint(void)
{
	CPU_INT08U  os_err;

	if(((gRobot.upperGun.shootTimes == 0)&&(gRobot.upperGun.mode != GUN_MANUAL_MODE))\
		||(gRobot.isReset == ROBOT_RESET)||(gRobot.isReload == ROBOT_RELOAD)||gRobot.moveBase.actualStopPoint == SHOOT_POINT_MOVING)
	{
		OSMboxPend(UpperGunShootPointMbox,0,&os_err);
		OSTimeDly(20);
		return MOVEBASE_POS_READY;
	}
	return GUN_NO_ERROR;
}
/**
*名称：ROBOT_LeftGunShoot
*功能：左枪开枪，开枪前需要确保子弹上膛，拉开保险，枪支架已经就绪
*@param None
*@retval status:GUN_NO_ERROR，GUN_CHAMPER_ERROR， GUN_NO_BULLET_ERROR， GUN_NO_READY_ERROR
*/
status_t ROBOT_LeftGunShoot(void)
{
	if(gRobot.leftGun.mode == GUN_AUTO_MODE)
	{
		if(gRobot.leftGun.ready == GUN_AIM_DONE)
		{
				ROBOT_LeftGunCheckConflict();
				gRobot.leftGun.shoot = GUN_START_SHOOT;

				LeftShoot();
				OSTimeDly(22);
				if(gRobot.leftGun.targetPlant!= PLANT7)
				{
					//如果上弹正常时上下一发弹，否则不上下一发弹
					if(gRobot.leftGun.champerErrerState == GUN_RELOAD_OK)
					{
						LeftPush();
						USART_SendData(USART3, 'x');
					}
				}
				OSTimeDly(6);
				LeftShootReset();
				gRobot.leftGun.shoot = GUN_STOP_SHOOT;
				if(gRobot.leftGun.champerErrerState == GUN_RELOAD_OK)
				{
					gRobot.leftGun.reloadState = GUN_NOT_RELOAD;
					gRobot.leftGun.shootTimes++;
					gRobot.leftGun.bulletNumber--;
				}
				else
				{
					//给一定延时让发射装置收回，因为上弹失败时没有上弹的时间让装置收回
					OSTimeDly(10);
				}
		}
	}
	if(gRobot.leftGun.mode == GUN_MANUAL_MODE)
	{
		LeftShoot();
		OSTimeDly(22);
		if(gRobot.leftGun.targetPlant!= PLANT7)
		{
			LeftPush();
		}
		OSTimeDly(6);
		LeftShootReset();
		gRobot.leftGun.shootTimes++;
		gRobot.leftGun.bulletNumber--;
		gRobot.leftGun.reloadState = GUN_NOT_RELOAD;
	}

	return GUN_NO_ERROR;

}

/**
*名称：ROBOT_RightGunShoot
*功能：右枪开枪，开枪前需要确保子弹上膛，拉开保险，枪支架已经就绪
*@param None
*@retval status:GUN_NO_ERROR，GUN_CHAMPER_ERROR， GUN_NO_BULLET_ERROR， GUN_NO_READY_ERROR
*/
status_t ROBOT_RightGunShoot(void)
{
	if(gRobot.rightGun.mode == GUN_AUTO_MODE)
	{
		if(gRobot.rightGun.ready == GUN_AIM_DONE)
		{
				ROBOT_RightGunCheckConflict();
				gRobot.rightGun.shoot=GUN_START_SHOOT;
				RightShoot();
				OSTimeDly(22);
				if(gRobot.rightGun.targetPlant != PLANT7)
				{
					//如果上弹正常时上下一发弹，否则不上下一发弹
					if(gRobot.rightGun.champerErrerState == GUN_RELOAD_OK)
					{
						RightPush();
						USART_SendData(USART3, 'y');

					}
				}
				OSTimeDly(6);
				RightShootReset();
				gRobot.rightGun.shoot = GUN_STOP_SHOOT;
				if(gRobot.rightGun.champerErrerState == GUN_RELOAD_OK)
				{
					gRobot.rightGun.reloadState = GUN_NOT_RELOAD;
					gRobot.rightGun.shootTimes++;
					gRobot.rightGun.bulletNumber--;
				}
				else
				{
					//给一定延时让发射装置收回，因为上弹失败时没有上弹的时间让装置收回
					OSTimeDly(10);
				}
		}
	}
	if(gRobot.rightGun.mode == GUN_MANUAL_MODE)
	{
		RightShoot();
		OSTimeDly(20);
		if(gRobot.rightGun.targetPlant != PLANT7)
		{
			RightPush();
		}
		OSTimeDly(8);
		RightShootReset();
		gRobot.rightGun.shootTimes++;
		gRobot.rightGun.bulletNumber--;
		gRobot.rightGun.reloadState = GUN_NOT_RELOAD;
	}
	return GUN_NO_ERROR;
}

/**
*名称：ROBOT_UpperGunShoot
*功能：上枪开枪，开枪前需要确保子弹足够
*@param None
*@retval status:GUN_NO_ERROR，GUN_CHAMPER_ERROR， GUN_NO_BULLET_ERROR， GUN_NO_READY_ERROR
*/
status_t ROBOT_UpperGunShoot(void)
{
	if(gRobot.upperGun.ready == GUN_AIM_DONE)
	{
		UpperShoot();
		OSTimeDly(20);
		if(gRobot.upperGun.shootTimes < 3)
		{
			OSTimeDly(20);
		}
		UpperShootReset();
		gRobot.upperGun.shootTimes++;
		//fix me, 应该检查子弹是否用完
		gRobot.upperGun.bulletNumber--;
	}
	return GUN_NO_ERROR;
}

/**
*@name ROBOT_LeftGunHome
*功能：左枪归位，开枪后为了更好的上膛需要归位
*@param None
*@retval status:GUN_NO_ERROR
*@note fix me, 此处发出命令后等待两秒以确保其能够归位，应加位置检测
*/
status_t ROBOT_LeftGunHome(void)
{
	PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(5.0f));
	PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(35.0f));
	PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(0.0f));

	return GUN_NO_ERROR;
}
/**
*@name ROBOT_RightGunHome
*功能：右枪归位，开枪后为了更好的上膛需要归位
*@param None
*@retval status:GUN_NO_ERROR
*@note fix me, 此处发出命令后等待两秒以确保其能够归位，应加位置检测
*/
status_t ROBOT_RightGunHome(void)
{
	PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(-5.0f));
	PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(35.0f));
	PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(0.0f));

	return GUN_NO_ERROR;
}

/**
*@name ROBOT_UpperGunHome
*功能:上枪归位，完成攻击任务后回到接近防守的姿态，做好防守的准备
*@param None
*@retval status:GUN_NO_ERROR
*@note fix me, 此处发出命令后等待两秒以确保其能够归位，应加位置检测
*/
status_t ROBOT_UpperGunHome(void)
{
	PosCrl(CAN1, UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(gUpperGunPosDatabase[SHOOT_POINT2][PLANT7][SHOOT_METHOD3][ZONE3].yaw));
	PosCrl(CAN1, UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(gUpperGunPosDatabase[SHOOT_POINT2][PLANT7][SHOOT_METHOD3][ZONE3].pitch));
	VelCrl(CAN1, UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(gUpperGunPosDatabase[SHOOT_POINT2][PLANT7][SHOOT_METHOD3][ZONE3].speed1));
	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_GunCheckMode
*功能：检查枪的模式
*参数：
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*status:
*/
status_t ROBOT_GunCheckMode(unsigned char gun)
{
	switch(gun)
	{
		case LEFT_GUN:
			return gRobot.leftGun.mode;
//			break;
		case RIGHT_GUN:
			return gRobot.rightGun.mode;
//			break;
		case UPPER_GUN:
			return gRobot.upperGun.mode;
//			break;
		default:
			break;
	}
	return -1;
}


/*
*名称：ROBOT_LeftGunCheckConflict
*功能：左枪检测两个枪是否冲突
*参数：
*status:
*/
status_t ROBOT_LeftGunCheckConflict(void)
{
	if(gRobot.leftGun.targetPlant == gRobot.rightGun.targetPlant)
	{
		if(gRobot.rightGun.shoot == GUN_START_SHOOT)
		{
			OSTimeDly(30);
		}
	}
	else if(gRobot.leftGun.targetPlant <= PLANT3)
	{
		if(gRobot.rightGun.targetPlant < gRobot.leftGun.targetPlant && gRobot.rightGun.shoot == GUN_START_SHOOT)
		{
			OSTimeDly(50);
		}
	}
	else if(gRobot.leftGun.targetPlant >= PLANT6)
	{
		if(gRobot.rightGun.targetPlant <= PLANT2 && gRobot.rightGun.shoot == GUN_START_SHOOT)
		{
			OSTimeDly(50);
		}
	}
	else
	{
		if((gRobot.rightGun.targetPlant < gRobot.leftGun.targetPlant || gRobot.leftGun.targetPlant > PLANT5)\
			&&gRobot.rightGun.shoot == GUN_START_SHOOT)
		{
			OSTimeDly(50);
		}
	}
	return GUN_NO_ERROR;
}
/*
*名称：ROBOT_RightGunCheckConflict
*功能：右枪检测两个枪是否冲突
*参数：
*status:
*/
status_t ROBOT_RightGunCheckConflict(void)
{
	if(gRobot.leftGun.targetPlant == gRobot.rightGun.targetPlant)
	{
		if(gRobot.leftGun.shoot == GUN_START_SHOOT)
		{
			OSTimeDly(30);
		}
	}
	else if(gRobot.leftGun.targetPlant <= PLANT3)
	{
		if(gRobot.rightGun.targetPlant < gRobot.leftGun.targetPlant && gRobot.leftGun.shoot == GUN_START_SHOOT)
		{
			OSTimeDly(50);
		}
	}
	else if(gRobot.leftGun.targetPlant >= PLANT6)
	{
		if(gRobot.rightGun.targetPlant <= PLANT2 && gRobot.leftGun.shoot == GUN_START_SHOOT)
		{
			OSTimeDly(50);
		}
	}
	else
	{
		if((gRobot.rightGun.targetPlant < gRobot.leftGun.targetPlant || gRobot.leftGun.targetPlant > PLANT5)\
			&&gRobot.leftGun.shoot == GUN_START_SHOOT)
		{
			OSTimeDly(50);
		}
	}
	return GUN_NO_ERROR;
}
/*
*名称：ROBOT_LeftGunReturn
*功能：左枪没有命令时复位
*参数：
*status:
*/
status_t ROBOT_LeftGunReturn(void)
{
	PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(gLeftGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].yaw));
	PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(gLeftGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].pitch));
	PosCrl(CAN1, LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(gLeftGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].roll));

	VelCrl(CAN1, LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(gLeftGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].speed1));
	VelCrl(CAN1, LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(gLeftGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].speed2));

	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_RightGunReturn
*功能：右枪没有命令时复位
*参数：
*status:
*/
status_t ROBOT_RightGunReturn(void)
{
	PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(gRightGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].yaw));
	PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(gRightGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].pitch));
	PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(gRightGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].roll));

	VelCrl(CAN1, RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(gRightGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].speed1));
	VelCrl(CAN1, RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(gRightGunPosDatabase[SHOOT_POINT2][SHOOT_METHOD4][PLANT6].speed2));

	return GUN_NO_ERROR;

}
