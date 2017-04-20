#include "robot.h"
#include "elmo.h"
#include "database.h"
#include "gasvalvecontrol.h"
#include "timer.h"
#include "ucos_ii.h"
#include "gpio.h"
#include "cpu.h"
#include "usart.h"
robot_t gRobot = {0};
extern OS_EVENT *OpenSaftyMbox;
extern OS_EVENT *LeftGunShootPointMbox;
extern OS_EVENT *RightGunShootPointMbox;

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

//	gRobot.leftGun.targetPose.pitch = 0.0f;
//	gRobot.leftGun.targetPose.yaw = 0.0f;
//	gRobot.leftGun.targetPose.roll = 0.0f;
	//特殊上弹角度
	gLeftGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD1][PLANT3].pitch = 26.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD2][PLANT3].pitch = 26.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD1][PLANT7].pitch = 20.0f;
	gLeftGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD2][PLANT7].pitch = 20.0f;
	
	
	gRobot.leftGun.targetPose.pitch = gLeftGunPosDatabase[gLeftGunShootCmds[0].shootPoint][gLeftGunShootCmds[0].shootMethod][gLeftGunShootCmds[0].plantNum].pitch;
	gRobot.leftGun.targetPose.yaw = gLeftGunPosDatabase[gLeftGunShootCmds[0].shootPoint][gLeftGunShootCmds[0].shootMethod][gLeftGunShootCmds[0].plantNum].yaw;
	gRobot.leftGun.targetPose.roll = gLeftGunPosDatabase[gLeftGunShootCmds[0].shootPoint][gLeftGunShootCmds[0].shootMethod][gLeftGunShootCmds[0].plantNum].roll;
	
	gRobot.leftGun.maxPoseLimit.pitch = 40.0f;
	gRobot.leftGun.maxPoseLimit.yaw = 50.0f;
	gRobot.leftGun.maxPoseLimit.roll = 46.54f;
	gRobot.leftGun.maxPoseLimit.speed1=200.0f;
	gRobot.leftGun.maxPoseLimit.speed2=200.0f;

	
	gRobot.leftGun.minPoseLimit.pitch = 7.0f;
	gRobot.leftGun.minPoseLimit.yaw = -50.0f;
	gRobot.leftGun.minPoseLimit.roll = -43.46f;
	gRobot.leftGun.minPoseLimit.speed1=0.0f;
	gRobot.leftGun.minPoseLimit.speed2=0.0f;	
	
	//枪未进行瞄准
	gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;
	//自动模式
	gRobot.leftGun.mode = GUN_AUTO_MODE;
	//子弹数
	gRobot.leftGun.bulletNumber = MAX_BULLET_NUMBER_LEFT;
	//fix me
	gRobot.leftGun.champerErrerState = 0;
	//枪停止射击
	gRobot.leftGun.shoot = GUN_STOP_SHOOT;
	//左枪姿态数据库
	gRobot.leftGun.gunPoseDatabase = (gun_pose_t **)gLeftGunPosDatabase;
	//左枪自动发射命令集合，里面为投射柱子的顺序
	gRobot.leftGun.shootCommand = (shoot_command_t *)gLeftGunShootCmds;
	//目标着陆台设置为无效台
	gRobot.leftGun.targetPlant = INVALID_PLANT_NUMBER;
	//目标打盘区设置为无效区
	gRobot.leftGun.targetZone = INVALID_ZONE_NUMBER;
	//射击次数为0
	gRobot.leftGun.shootTimes = 0;
	
	elmo_Enable(CAN1, LEFT_GUN_LEFT_ID);
	elmo_Enable(CAN1, LEFT_GUN_RIGHT_ID);
	elmo_Enable(CAN1, LEFT_GUN_PITCH_ID);
	elmo_Enable(CAN1, LEFT_GUN_ROLL_ID);
	elmo_Enable(CAN1, LEFT_GUN_YAW_ID);


	Vel_cfg(CAN1, LEFT_GUN_LEFT_ID, 300000,300000);	
	Vel_cfg(CAN1, LEFT_GUN_RIGHT_ID, 300000,300000);	

	Pos_cfg(CAN1, LEFT_GUN_PITCH_ID, 30000,30000,80000);//俯仰
	Pos_cfg(CAN1, LEFT_GUN_ROLL_ID, 30000,30000,80000);//翻滚
	Pos_cfg(CAN1, LEFT_GUN_YAW_ID,30000,30000,80000);//航向
	

}

static void RightGunInit(void)
{
	gRobot.rightGun.actualPose.pitch = 0.0f;
	gRobot.rightGun.actualPose.yaw = 0.0f;
	gRobot.rightGun.actualPose.roll = 0.0f;
	
//	gRightGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD1][PLANT5].roll = -10.0f;
//	gRightGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD2][PLANT5].roll = -10.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD1][PLANT3].pitch = 26.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD2][PLANT3].pitch = 26.0f;

	gRightGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD1][PLANT7].pitch = 20.0f;
	gRightGunReloadPosDatabase[SHOOT_POINT3][SHOOT_METHOD2][PLANT7].pitch = 20.0f;

	
	gRobot.rightGun.targetPose.pitch = gRightGunPosDatabase[gRightGunShootCmds[0].shootPoint][gRightGunShootCmds[0].shootMethod][gRightGunShootCmds[0].plantNum].pitch;
	gRobot.rightGun.targetPose.yaw = gRightGunPosDatabase[gRightGunShootCmds[0].shootPoint][gRightGunShootCmds[0].shootMethod][gRightGunShootCmds[0].plantNum].yaw;
	gRobot.rightGun.targetPose.roll = gRightGunPosDatabase[gRightGunShootCmds[0].shootPoint][gRightGunShootCmds[0].shootMethod][gRightGunShootCmds[0].plantNum].roll;
	
	gRobot.rightGun.maxPoseLimit.pitch = 40.0f;
	gRobot.rightGun.maxPoseLimit.yaw = 50.0f;
	gRobot.rightGun.maxPoseLimit.roll = 46.54f;
	gRobot.rightGun.maxPoseLimit.speed1=200.0f;
	gRobot.rightGun.maxPoseLimit.speed2=200.0f;
	
	gRobot.rightGun.minPoseLimit.pitch = 7.0f;
	gRobot.rightGun.minPoseLimit.yaw = -50.0f;
	gRobot.rightGun.minPoseLimit.roll = -43.46f;
	gRobot.rightGun.minPoseLimit.speed1=0.0f;
	gRobot.rightGun.minPoseLimit.speed2=0.0f;
	
	//枪未进行瞄准
	gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
	//自动模式
	gRobot.rightGun.mode = GUN_AUTO_MODE;
	//最大子弹数
	gRobot.rightGun.bulletNumber = MAX_BULLET_NUMBER_RIGHT;
	//fix me
	gRobot.rightGun.champerErrerState = 0;
	//枪停止射击
	gRobot.rightGun.shoot = GUN_STOP_SHOOT;
	//右枪姿态数据库
	gRobot.rightGun.gunPoseDatabase = (gun_pose_t **)gRightGunPosDatabase;
	//右枪自动发射命令集合，里面为投射柱子的顺序
	gRobot.rightGun.shootCommand = (shoot_command_t *)gRightGunShootCmds;
	//目标着陆台设置为无效台
	gRobot.rightGun.targetPlant = INVALID_PLANT_NUMBER;
	//目标打盘区设置为无效区
	gRobot.rightGun.targetZone = INVALID_ZONE_NUMBER;
	//射击次数为0
	gRobot.rightGun.shootTimes = 0;
	
	elmo_Enable(CAN1, RIGHT_GUN_LEFT_ID);
	elmo_Enable(CAN1, RIGHT_GUN_RIGHT_ID);
	elmo_Enable(CAN1, RIGHT_GUN_PITCH_ID);
	elmo_Enable(CAN1, RIGHT_GUN_ROLL_ID);
	elmo_Enable(CAN1, RIGHT_GUN_YAW_ID);


	Vel_cfg(CAN1, RIGHT_GUN_LEFT_ID, 300000,300000);	
	Vel_cfg(CAN1, RIGHT_GUN_RIGHT_ID, 300000,300000);	

	Pos_cfg(CAN1, RIGHT_GUN_PITCH_ID, 30000,30000,80000);//俯仰
	Pos_cfg(CAN1, RIGHT_GUN_ROLL_ID, 30000,30000,80000);//翻滚
	Pos_cfg(CAN1, RIGHT_GUN_YAW_ID,30000,30000,80000);//航向
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
	gRobot.upperGun.maxPoseLimit.yaw = 20.0f;
	gRobot.upperGun.maxPoseLimit.roll = 0.0f;
	gRobot.upperGun.maxPoseLimit.speed1=200.0f;
	gRobot.upperGun.maxPoseLimit.speed2=0.0f;
	
	gRobot.upperGun.minPoseLimit.pitch = -10.0f;
	gRobot.upperGun.minPoseLimit.yaw = -20.0f;
	gRobot.upperGun.minPoseLimit.roll = 0.0f;
	gRobot.upperGun.minPoseLimit.speed1=0.0f;
	gRobot.upperGun.minPoseLimit.speed2=0.0f;
	
	gRobot.upperGun.targetPose.pitch = gUpperGunPosDatabase[gUpperGunShootCmds[0].plantNum][gUpperGunShootCmds[0].shootMethod][ZONE1].pitch;
	gRobot.upperGun.targetPose.yaw = gUpperGunPosDatabase[gUpperGunShootCmds[0].plantNum][gUpperGunShootCmds[0].shootMethod][ZONE1].yaw;
	gRobot.upperGun.targetPose.speed1 = gUpperGunPosDatabase[gUpperGunShootCmds[0].plantNum][gUpperGunShootCmds[0].shootMethod][ZONE1].speed1;

	
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
	//目标打盘区设置为无效区
	gRobot.upperGun.targetZone = INVALID_ZONE_NUMBER;
	//射击次数为0
	gRobot.upperGun.shootTimes = 0;
	
	elmo_Enable(CAN1, UPPER_GUN_LEFT_ID);
	elmo_Enable(CAN1, UPPER_GUN_YAW_ID);
	elmo_Enable(CAN1, UPPER_GUN_PITCH_ID);
	
	Vel_cfg(CAN1, UPPER_GUN_LEFT_ID,350000,350000);
	Pos_cfg(CAN1, UPPER_GUN_YAW_ID,30000,30000,50000);//航向
	Pos_cfg(CAN1, UPPER_GUN_PITCH_ID,30000,30000,50000);//俯仰
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
	if(yaw > gRobot.leftGun.maxPoseLimit.yaw) yaw = gRobot.leftGun.maxPoseLimit.yaw;	
	if(yaw < gRobot.leftGun.minPoseLimit.yaw) yaw = gRobot.leftGun.minPoseLimit.yaw;
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
	if(pitch > gRobot.leftGun.maxPoseLimit.pitch) pitch = gRobot.leftGun.maxPoseLimit.pitch;	
	if(pitch < gRobot.leftGun.minPoseLimit.pitch) pitch = gRobot.leftGun.minPoseLimit.pitch;
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
	if(roll > gRobot.leftGun.maxPoseLimit.roll) roll = gRobot.leftGun.maxPoseLimit.roll;	
	if(roll < gRobot.leftGun.minPoseLimit.roll) roll = gRobot.leftGun.minPoseLimit.roll;
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
	
	if(speed > gRobot.leftGun.maxPoseLimit.speed1) speed = gRobot.leftGun.maxPoseLimit.speed1;	
	if(speed < gRobot.leftGun.minPoseLimit.speed1) speed = gRobot.leftGun.minPoseLimit.speed1;
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
	if(speed > gRobot.leftGun.maxPoseLimit.speed2) speed = gRobot.leftGun.maxPoseLimit.speed2;	
	if(speed < gRobot.leftGun.minPoseLimit.speed2) speed = gRobot.leftGun.minPoseLimit.speed2;
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
	if(yaw > gRobot.rightGun.maxPoseLimit.yaw) yaw = gRobot.rightGun.maxPoseLimit.yaw;	
	if(yaw < gRobot.rightGun.minPoseLimit.yaw) yaw = gRobot.rightGun.minPoseLimit.yaw;
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
	if(pitch > gRobot.rightGun.maxPoseLimit.pitch) pitch = gRobot.rightGun.maxPoseLimit.pitch;	
	if(pitch < gRobot.rightGun.minPoseLimit.pitch) pitch = gRobot.rightGun.minPoseLimit.pitch;
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
	if(roll > gRobot.rightGun.maxPoseLimit.roll) roll = gRobot.rightGun.maxPoseLimit.roll;	
	if(roll < gRobot.rightGun.minPoseLimit.roll) roll = gRobot.rightGun.minPoseLimit.roll;
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
	if(speed > gRobot.rightGun.maxPoseLimit.speed1) speed = gRobot.rightGun.maxPoseLimit.speed1;	
	if(speed < gRobot.rightGun.minPoseLimit.speed1) speed = gRobot.rightGun.minPoseLimit.speed1;
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
	if(speed > gRobot.rightGun.maxPoseLimit.speed2) speed = gRobot.rightGun.maxPoseLimit.speed2;	
	if(speed < gRobot.rightGun.minPoseLimit.speed2) speed = gRobot.rightGun.minPoseLimit.speed2;
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
	if(yaw > gRobot.upperGun.maxPoseLimit.yaw) yaw = gRobot.upperGun.maxPoseLimit.yaw;	
	if(yaw < gRobot.upperGun.minPoseLimit.yaw) yaw = gRobot.upperGun.minPoseLimit.yaw;
	return (int32_t)((20.0f + yaw) * 102.4f);
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
	return (float)position / 102.4f - 20.0f;
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
	if(pitch > gRobot.upperGun.maxPoseLimit.pitch) pitch = gRobot.upperGun.maxPoseLimit.pitch;	
	if(pitch < gRobot.upperGun.minPoseLimit.pitch) pitch = gRobot.upperGun.minPoseLimit.pitch;
	return (int32_t)((10.0f + pitch) * 141.0844f);	
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
	return (float)position/141.0844f - 10.0f;
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
	if(speed > gRobot.upperGun.maxPoseLimit.speed1) speed = gRobot.upperGun.maxPoseLimit.speed1;	
	if(speed < gRobot.upperGun.minPoseLimit.speed1) speed = gRobot.upperGun.minPoseLimit.speed1;
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
============================================================
				   机器人动作流程函数			
============================================================
*/

/*
*名称：ROBOT_Init
*功能：机器人初始化，初始化底盘，初始化枪，初始化
*参数：none
*注意：上面的枪不需要上子弹，因为是手动上弹
*/
status_t ROBOT_Init(void)
{
	gRobot.stage = ROBOT_STAGE_POWER_ON;
	gRobot.shootTimes = 0;
	gRobot.status = ROBOT_STATUS_OK;
	gRobot.moveBase.targetPoint = 2;
	for(uint8_t i = 0; i < 7;i++)
	{
		gRobot.plantState[i].ball = 1;
	}
    gRobot.plantState[PLANT6].ball = 1;
	
	for(uint8_t i = 0; i < 7;i++)
	{
		gRobot.plantState[i].plate = 1;
	}
  gRobot.plantState[PLANT6].plate = 1;

	LeftGunInit();
	RightGunInit();
	UpperGunInit();

	MOVEBASE_Init();
	
	gRobot.stage = ROBOT_STAGE_INIT;

	return GUN_NO_ERROR;
}

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
//	uint8_t pushTimes = 5;
	if(gRobot.leftGun.reloadState == GUN_NOT_RELOAD)
	{
//		LeftPush();
		if(gRobot.leftGun.shootTimes == 0)
		{
			OSTimeDly(60);
		}
		else
		{
			OSTimeDly(45);
		}
//		if(gRobot.leftGun.shootTimes == 0)
//		{
//			pushTimes = 7;
//		}
//		while(pushTimes--)
//		{
//			LeftPush();
//			OSTimeDly(2);
//			LeftHold();
//			OSTimeDly(7);
//		}
		LeftBack();
		OSTimeDly(10);
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
//	uint8_t pushTimes = 8;
	if(gRobot.rightGun.reloadState == GUN_NOT_RELOAD)
	{
//		RightPush();
		if(gRobot.rightGun.shootTimes == 0)
		{
			OSTimeDly(60);
		}
		else
		{
			OSTimeDly(45);
		}
//		while(pushTimes--)
//		{
//			RightPush();
//			OSTimeDly(2);
//			RightHold();
//			OSTimeDly(8);
//		}
		RightBack();
		OSTimeDly(10);//看似无用

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
	int timeOut = 20;
	while(timeOut--)
	{
		if(PHOTOSENSORLEFTGUN)
		{
			gRobot.leftGun.champerErrerState = GUN_RELOAD_OK;
			break;
		}
		else
		{
			ROBOT_LeftGunHome();
		}
		OSTimeDly(5);
	}
	gRobot.leftGun.champerErrerState = GUN_RELOAD_OK;
	return GUN_RELOAD_ERROR;
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
	int timeOut = 20;
	while(timeOut--)
	{
		if(!PHOTOSENSORRIGHTGUN)
		{
			gRobot.rightGun.champerErrerState = GUN_RELOAD_OK;
			break;
		}
		else
		{
			ROBOT_RightGunHome();
		}
		OSTimeDly(5);
	}
	gRobot.rightGun.champerErrerState = GUN_RELOAD_OK;
	return GUN_RELOAD_ERROR;
}

/*
*名称：ROBOT_GunCheckBulletState
*功能：根据枪膛传感器，检测子弹状态，决定后面开枪的具体参数
*参数：
*gun :LEFT_GUN, RIGHT_GUN
*status:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹，因为是手动上弹
*/
status_t ROBOT_GunCheckBulletState(unsigned char gun)
{
//	switch(gun)
//	{
//		case LEFT_GUN:
//			//fix me, should be replaced by senser results
//			gRobot.leftGun.champerBulletState = CHAMPER_BULLET_FEATURE0_STATE;
//			break;
//		case RIGHT_GUN:
//			gRobot.rightGun.champerBulletState = CHAMPER_BULLET_FEATURE0_STATE;
//			break;
//		case UPPER_GUN:
//			gRobot.upperGun.champerBulletState = CHAMPER_BULLET_FEATURE0_STATE;
//			break;
//		default:
//			break;
//	}
	return GUN_NO_ERROR;
}
/** @defgroup Left_Gun_Shoot_Tragedy
  * @brief 
  * @{
  */

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
	#define LEFT_AUTO_NUMBER 2
	shoot_command_t shootCommand = {SHOOT_POINT3, PLANT6, SHOOT_METHOD2};
	uint8_t searchRange = 3;
//	gRobot.plantState[PLANT6].plate = 1;
//	if(gRobot.leftGun.shootTimes >= LEFT_AUTO_NUMBER)searchRange = 7;
	for(uint8_t i = 0;i < searchRange;i++)
	{
		//有球
		if(gRobot.plantState[LeftGunPriority[i]].ball >= 1)
		{
			shootCommand.plantNum = LeftGunPriority[i];
			shootCommand.shootMethod = SHOOT_METHOD1;
			gRobot.plantState[LeftGunPriority[i]].ball -= 1;
			gRobot.leftGun.commandState = GUN_HAVE_COMMAND;
			break;
		}
		//没盘
		if(gRobot.plantState[LeftGunPriority[i]].plate >= 1)
		{
			shootCommand.plantNum = LeftGunPriority[i];
			shootCommand.shootMethod = SHOOT_METHOD2;
			gRobot.plantState[LeftGunPriority[i]].plate -= 1;
			gRobot.leftGun.commandState = GUN_HAVE_COMMAND;

			break;		
		}
		if(i == (searchRange-1))
		{
			gRobot.leftGun.commandState = GUN_NO_COMMAND;
		}
	}
	
	return shootCommand;
}


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

/** @defgroup Right_Gun_Shoot_Tragedy
  * @brief 
  * @{
  */

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
	#define RIGHT_AUTO_NUMBER 2
	shoot_command_t shootCommand = {SHOOT_POINT3, PLANT6, SHOOT_METHOD2};
	uint8_t searchRange = 3;
//	gRobot.plantState[PLANT6].plate = 1;
//	if(gRobot.rightGun.shootTimes >= RIGHT_AUTO_NUMBER)searchRange = 7;
	for(uint8_t i = 0;i < searchRange;i++)
	{
		//有球
		if(gRobot.plantState[RightGunPriority[i]].ball >= 1)
		{
			shootCommand.plantNum = RightGunPriority[i];
			shootCommand.shootMethod = SHOOT_METHOD1;
			gRobot.plantState[RightGunPriority[i]].ball -= 1;
			gRobot.rightGun.commandState = GUN_HAVE_COMMAND;
			break;
		}
		//没盘
		if(gRobot.plantState[RightGunPriority[i]].plate >= 1)
		{
			shootCommand.plantNum = RightGunPriority[i];
			shootCommand.shootMethod = SHOOT_METHOD2;
			gRobot.plantState[RightGunPriority[i]].plate -= 1;
			gRobot.rightGun.commandState = GUN_HAVE_COMMAND;
			break;
		
		}
		if(i==(searchRange-1))
		{
			gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
			gRobot.rightGun.commandState = GUN_NO_COMMAND;
		}
	}
	
	return shootCommand;
}


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
	uint8_t i = 0u;
	shoot_command_t shootCommand = {SHOOT_POINT3, PLANT6, SHOOT_METHOD2};
	//为使上枪接收命令更难
	OSTimeDly(5);
	for( i = 0;i < 2;i++)
	{
		//有球
		if(gRobot.plantState[UpperGunPriority[i]].ball >= 1 && UpperGunPriority[i]!=PLANT6)
		{
			shootCommand.plantNum = UpperGunPriority[i];
			shootCommand.shootMethod = SHOOT_METHOD1;
			gRobot.plantState[UpperGunPriority[i]].ball -= 1;
			gRobot.upperGun.commandState = GUN_HAVE_COMMAND;
			break;
		}
		//没盘
		if(gRobot.plantState[UpperGunPriority[i]].plate >= 1)
		{
			shootCommand.plantNum = UpperGunPriority[i];
			shootCommand.shootMethod = SHOOT_METHOD2;
			gRobot.plantState[UpperGunPriority[i]].plate -= 1;
			gRobot.upperGun.commandState = GUN_HAVE_COMMAND;
			break;
		
		}
		if(i==1)
		{
			gRobot.upperGun.ready = GUN_AIM_IN_PROCESS;
			gRobot.upperGun.commandState = GUN_NO_COMMAND;
		}
	}
	
	return shootCommand;
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
	//超时时间为20*5*10ms，1秒
	uint8_t checkTimes = 5;
	int checkTime = 0;
	while(checkTimes--)
	{
		int timeout = 20;
		while(timeout--)
		{
			ReadActualPos(CAN1, LEFT_GUN_GROUP_ID);		
			ReadActualVel(CAN1, LEFT_GUN_VEL_GROUP_ID);
			OSTimeDly(5);
			USART_SendData(UART5, 0);
			LeftGunSendDebugInfo();
			//fix me,检查枪位姿是否到位，后面需要在枪结构体中增加可容忍误差，然后封装成函数检测
			if(gRobot.leftGun.actualPose.pitch > gRobot.leftGun.targetPose.pitch + 0.0f || \
				gRobot.leftGun.actualPose.pitch < gRobot.leftGun.targetPose.pitch - 0.0f)
			{
				continue;
			}
			
			if(gRobot.leftGun.actualPose.roll > gRobot.leftGun.targetPose.roll + 0.0f || \
				gRobot.leftGun.actualPose.roll < gRobot.leftGun.targetPose.roll - 0.0f)
			{
				continue;
			}
			
			if(gRobot.leftGun.actualPose.yaw > gRobot.leftGun.targetPose.yaw + 0.0f || \
				gRobot.leftGun.actualPose.yaw < gRobot.leftGun.targetPose.yaw - 0.0f)
			{
				continue;
			}
			if(gRobot.leftGun.actualPose.speed1 > gRobot.leftGun.targetPose.speed1 +0.0f|| \
				gRobot.leftGun.actualPose.speed1 < gRobot.leftGun.targetPose.speed1 - 0.0f)
			{
				continue;
			}
			if(gRobot.leftGun.actualPose.speed2 > gRobot.leftGun.targetPose.speed2 + 0.0f || \
				gRobot.leftGun.actualPose.speed2 < gRobot.leftGun.targetPose.speed2 - 0.0f)
			{
				continue;
			}	
			//运行到这里，表示都满足指标，跳出循环
			break;
		}
		checkTime += (50-timeout);	
	}
	gRobot.leftGun.checkTimeUsage = checkTime;
	gRobot.leftGun.ready = GUN_AIM_DONE;
	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_RightGunCheckAim
*功能：检查瞄准是否已完成，不同枪分开检测为了防止重入，
*因为此函数中需要设计超时
*参数：
*none
*status:GUN_AIM_IN_PROCESS， GUN_AIM_DONE
*注意：
*/
status_t ROBOT_RightGunCheckAim(void)
{
	//超时时间为20*5*10ms，1秒
	uint8_t checkTimes = 5;
	int checkTime = 0;
	while(checkTimes--)
	{
		int timeout = 20;
		while(timeout--)
		{
			//fix me,发送3组命令需要200us*3，加上返回的5帧数据，会达到2ms，这里最好使用组ID实现，需要驱动器支持
			//fix me 三轴位置已经支持组ID，组ID在robot.h中定义
			ReadActualPos(CAN1,RIGHT_GUN_GROUP_ID);		
			ReadActualVel(CAN1,RIGHT_GUN_VEL_GROUP_ID);
			OSTimeDly(5);
			USART_SendData(UART5, 0);
			RightGunSendDebugInfo();
			//fix me,检查枪位姿是否到位，后面需要在枪结构体中增加可容忍误差，然后封装成函数检测
			if(gRobot.rightGun.actualPose.pitch > gRobot.rightGun.targetPose.pitch + 0.0f || \
				gRobot.rightGun.actualPose.pitch < gRobot.rightGun.targetPose.pitch - 0.0f)
			{
				continue;
			}
			
			if(gRobot.rightGun.actualPose.roll > gRobot.rightGun.targetPose.roll + 0.0f || \
				gRobot.rightGun.actualPose.roll < gRobot.rightGun.targetPose.roll - 0.0f)
			{
				continue;
			}
			
			if(gRobot.rightGun.actualPose.yaw > gRobot.rightGun.targetPose.yaw + 0.0f || \
				gRobot.rightGun.actualPose.yaw < gRobot.rightGun.targetPose.yaw - 0.0f)
			{
				continue;
			}
			
			if(gRobot.rightGun.actualPose.speed1 > gRobot.rightGun.targetPose.speed1 + 0.0f || \
				gRobot.rightGun.actualPose.speed1 < gRobot.rightGun.targetPose.speed1 - 0.0f)
			{
				continue;
			}
			if(gRobot.rightGun.actualPose.speed2 > gRobot.rightGun.targetPose.speed2 + 0.0f || \
				gRobot.rightGun.actualPose.speed2 < gRobot.rightGun.targetPose.speed2 - 0.0f)
			{
				continue;
			}	
			//运行到这里，表示都满足指标，跳出循环
			break;
		}
		checkTime+=(50-timeout);	
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
	//超时时间为100*5*10ms，5秒
	uint8_t checkTimes = 5;
	uint8_t lastTargetZone = gRobot.upperGun.targetZone;
	if(gRobot.upperGun.mode==GUN_DEFEND_MODE)checkTimes = 2;
	while(checkTimes--)
	{
		int timeout = 20;
		while(timeout--)
		{
			//检查防守台上盘状态的变化，如果有改变立即跳出循环重新打盘
			//fix me 耦合太高
			if(gRobot.upperGun.mode == GUN_ATTACK_MODE)
			{
				if(gRobot.upperGun.targetZone &0x0f)
				{
					break;
				}
			}
			if(gRobot.upperGun.mode == GUN_DEFEND_MODE)
			{
				if (lastTargetZone != gRobot.upperGun.targetZone)
				{
					gRobot.upperGun.shoot = GUN_STOP_SHOOT;
					return GUN_NO_READY_ERROR;
				}
			}
			//fix me 三轴位置已经支持组ID，组ID在robot.h中定义
			ReadActualPos(CAN1, UPPER_GUN_GROUP_ID);
			ReadActualVel(CAN1, UPPER_GUN_VEL_GROUP_ID);
			OSTimeDly(5);
			
			//fix me,检查枪位姿是否到位，后面需要在枪结构体中增加可容忍误差，然后封装成函数检测
			if(gRobot.upperGun.actualPose.pitch > gRobot.upperGun.targetPose.pitch + 1.5f || \
				gRobot.upperGun.actualPose.pitch < gRobot.upperGun.targetPose.pitch - 1.5f)
			{
				continue;
			}
			
			if(gRobot.upperGun.actualPose.yaw > gRobot.upperGun.targetPose.yaw + 0.5f || \
				gRobot.upperGun.actualPose.yaw < gRobot.upperGun.targetPose.yaw - 0.5f)
			{
				continue;
			}
			if(gRobot.upperGun.actualPose.speed1 > gRobot.upperGun.targetPose.speed1 +1.0f || \
				gRobot.upperGun.actualPose.speed1 < gRobot.upperGun.targetPose.speed1 -1.0f)
			{
				continue;
			}
			
			//这里检查传送带的速度，暂时没有加
			break;
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
	if(gRobot.leftGun.shootTimes == 0||gRobot.leftGun.shootTimes ==  ROBOT_LeftGunPoint1ShootTimes() ||\
		gRobot.leftGun.shootTimes ==  ROBOT_LeftGunPoint1ShootTimes() + LEFT_GUN_POINT2_AUTO_BULLET_NUMBER)
	{
		OSMboxPend(LeftGunShootPointMbox,0,&os_err);
		OSTimeDly(50);
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

	if(gRobot.rightGun.shootTimes == 0||gRobot.rightGun.shootTimes ==  ROBOT_RightGunPoint1ShootTimes() ||\
		gRobot.rightGun.shootTimes == ROBOT_RightGunPoint1ShootTimes() + RIGHT_GUN_POINT2_AUTO_BULLET_NUMBER)
	{
		OSMboxPend(RightGunShootPointMbox,0,&os_err);
		OSTimeDly(50);
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

				if(gRobot.leftGun.targetPlant == gRobot.rightGun.targetPlant && \
					gRobot.rightGun.shoot == GUN_START_SHOOT)
				{
					OSTimeDly(100);
				}
				gRobot.leftGun.shoot = GUN_START_SHOOT;
				LeftShoot();
				OSTimeDly(20);
				LeftPush();
				OSTimeDly(8);
				LeftShootReset();
				gRobot.leftGun.shoot = GUN_STOP_SHOOT;
				gRobot.leftGun.reloadState = GUN_NOT_RELOAD;
				gRobot.leftGun.shootTimes++;
				gRobot.leftGun.actualStepShootTimes[gRobot.leftGun.shootCommand[gRobot.leftGun.shootStep].plantNum]\
																						[gRobot.leftGun.shootCommand[gRobot.leftGun.shootStep].shootMethod]++;			
				gRobot.leftGun.bulletNumber--;
		}
	}
	if(gRobot.leftGun.mode == GUN_MANUAL_MODE)
	{
		LeftShoot();
		OSTimeDly(20);
		LeftPush();
		OSTimeDly(8);
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

				if(gRobot.leftGun.targetPlant == gRobot.rightGun.targetPlant
					&& 	gRobot.leftGun.shoot == GUN_START_SHOOT)
				{
					OSTimeDly(100);
				}
				gRobot.rightGun.shoot=GUN_START_SHOOT;
				RightShoot();	
				OSTimeDly(20);
				RightPush();
				OSTimeDly(8);
				RightShootReset();
				gRobot.rightGun.shoot = GUN_STOP_SHOOT;
				gRobot.rightGun.reloadState = GUN_NOT_RELOAD;
				gRobot.rightGun.shootTimes++;
				gRobot.rightGun.actualStepShootTimes[gRobot.rightGun.shootCommand[gRobot.rightGun.shootStep].plantNum][gRobot.rightGun.shootCommand[gRobot.rightGun.shootStep].shootMethod]++;			
				//fix me, 应该检查子弹是否用完
				gRobot.rightGun.bulletNumber--;
		}
	}
	if(gRobot.rightGun.mode == GUN_MANUAL_MODE)
	{
		RightShoot();	
		OSTimeDly(20);
		RightPush();
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
			USART_SendData(USART3,'b');
			OSTimeDly(40);
			UpperShootReset();
			gRobot.upperGun.shootTimes++;
			gRobot.upperGun.actualStepShootTimes[gRobot.upperGun.shootCommand[gRobot.upperGun.shootStep].plantNum][gRobot.upperGun.shootCommand[gRobot.upperGun.shootStep].shootMethod]++;			
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
	PosCrl(CAN1, LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(0.0f));
	PosCrl(CAN1, LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(40.0f));			
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
	PosCrl(CAN1, RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(0.0f));
	PosCrl(CAN1, RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(40.0f));			
	PosCrl(CAN1, RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(0.0f));	
	
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
*名称：ROBOT_LeftGunCheckPlantState
*功能：左枪检查各个柱子状态并更新指令
*参数：
*status:
*/
status_t ROBOT_LeftGunCheckPlantState(void)
{
	uint8_t i = 0;
	for(i = 0;i < LEFT_GUN_AUTO_SHOOT_STEP_NUMBER;i++)
	{
		//着陆台上有球有盘
		if(gRobot.plantState[gRobot.leftGun.shootCommand[i].plantNum].ball == 0&& \
			gRobot.plantState[gRobot.leftGun.shootCommand[i].plantNum].plate == 1)
		{
			if(gRobot.leftGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.leftGun.shootCommand[i].stepTargetShootTime = 1;
			}
			if(gRobot.leftGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.leftGun.shootCommand[i].stepTargetShootTime = gRobot.leftGun.actualStepShootTimes[gRobot.leftGun.shootCommand[i].plantNum][gRobot.leftGun.shootCommand[i].shootMethod];
			}				
		}
		//着陆台上有球没盘
		if(gRobot.plantState[gRobot.leftGun.shootCommand[i].plantNum].ball == 0&& \
			gRobot.plantState[gRobot.leftGun.shootCommand[i].plantNum].plate == 0)
		{
			if(gRobot.leftGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.leftGun.shootCommand[i].stepTargetShootTime = 1;
			}
			if(gRobot.leftGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.leftGun.shootCommand[i].stepTargetShootTime = 2;
			}				
		}
		//着陆台上没球有盘
		if(gRobot.plantState[gRobot.leftGun.shootCommand[i].plantNum].ball == 1&& \
			gRobot.plantState[gRobot.leftGun.shootCommand[i].plantNum].plate == 1)
		{
			if(gRobot.leftGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.leftGun.shootCommand[i].stepTargetShootTime = gRobot.leftGun.actualStepShootTimes[gRobot.leftGun.shootCommand[i].plantNum][gRobot.leftGun.shootCommand[i].shootMethod];
			}
			if(gRobot.leftGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.leftGun.shootCommand[i].stepTargetShootTime = gRobot.leftGun.actualStepShootTimes[gRobot.leftGun.shootCommand[i].plantNum][gRobot.leftGun.shootCommand[i].shootMethod];
			}				
		}
		//着陆台上没球没盘
		if(gRobot.plantState[gRobot.leftGun.shootCommand[i].plantNum].ball == 1&& \
			gRobot.plantState[gRobot.leftGun.shootCommand[i].plantNum].plate == 0)
		{
			if(gRobot.leftGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.leftGun.shootCommand[i].stepTargetShootTime = gRobot.leftGun.actualStepShootTimes[gRobot.leftGun.shootCommand[i].plantNum][gRobot.leftGun.shootCommand[i].shootMethod];
			}
			if(gRobot.leftGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.leftGun.shootCommand[i].stepTargetShootTime = 2;
			}				
		}		
	}
	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_RightGunCheckPlantState
*功能：左枪检查各个柱子状态并更新指令
*参数：
*status:
*/
status_t ROBOT_RightGunCheckPlantState(void)
{
	uint8_t i = 0;
	for(i = 0;i < RIGHT_GUN_AUTO_SHOOT_STEP_NUMBER;i++)
	{
		//着陆台上有球有盘
		if(gRobot.plantState[gRobot.rightGun.shootCommand[i].plantNum].ball == 0&& \
			gRobot.plantState[gRobot.rightGun.shootCommand[i].plantNum].plate == 1)
		{
			if(gRobot.rightGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.rightGun.shootCommand[i].stepTargetShootTime = 1;
			}
			if(gRobot.rightGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.rightGun.shootCommand[i].stepTargetShootTime = gRobot.rightGun.actualStepShootTimes[gRobot.rightGun.shootCommand[i].plantNum][gRobot.rightGun.shootCommand[i].shootMethod];
			}				
		}
		//着陆台上有球没盘
		if(gRobot.plantState[gRobot.rightGun.shootCommand[i].plantNum].ball == 0&& \
			gRobot.plantState[gRobot.rightGun.shootCommand[i].plantNum].plate == 0)
		{
			if(gRobot.rightGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.rightGun.shootCommand[i].stepTargetShootTime = 1;
			}
			if(gRobot.rightGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.rightGun.shootCommand[i].stepTargetShootTime = 2;
			}				
		}
		//着陆台上没球有盘
		if(gRobot.plantState[gRobot.rightGun.shootCommand[i].plantNum].ball == 1&& \
			gRobot.plantState[gRobot.rightGun.shootCommand[i].plantNum].plate == 1)
		{
			if(gRobot.rightGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.rightGun.shootCommand[i].stepTargetShootTime = gRobot.rightGun.actualStepShootTimes[gRobot.rightGun.shootCommand[i].plantNum][gRobot.rightGun.shootCommand[i].shootMethod];
			}
			if(gRobot.rightGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.rightGun.shootCommand[i].stepTargetShootTime = gRobot.rightGun.actualStepShootTimes[gRobot.rightGun.shootCommand[i].plantNum][gRobot.rightGun.shootCommand[i].shootMethod];
			}				
		}
		//着陆台上没球没盘
		if(gRobot.plantState[gRobot.rightGun.shootCommand[i].plantNum].ball == 1&& \
			gRobot.plantState[gRobot.rightGun.shootCommand[i].plantNum].plate == 0)
		{
			if(gRobot.rightGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.rightGun.shootCommand[i].stepTargetShootTime = gRobot.rightGun.actualStepShootTimes[gRobot.rightGun.shootCommand[i].plantNum][gRobot.rightGun.shootCommand[i].shootMethod];
			}
			if(gRobot.rightGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.rightGun.shootCommand[i].stepTargetShootTime = 2;
			}				
		}		
	}
	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_UpperGunCheckPlantState
*功能：左枪检查各个柱子状态并更新指令
*参数：
*status:
*/
status_t ROBOT_UpperGunCheckPlantState(void)
{
	uint8_t i = 0;
	for(i = 0;i < 3;i++)
	{
		//着陆台上有球有盘
		if(gRobot.plantState[gRobot.upperGun.shootCommand[i].plantNum].ball == 0&& \
			gRobot.plantState[gRobot.upperGun.shootCommand[i].plantNum].plate == 1)
		{
			if(gRobot.upperGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.upperGun.shootCommand[i].stepTargetShootTime = 1;
			}
			if(gRobot.upperGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.upperGun.shootCommand[i].stepTargetShootTime = \
				gRobot.upperGun.actualStepShootTimes[gRobot.upperGun.shootCommand[i].plantNum][gRobot.upperGun.shootCommand[i].shootMethod];
			}				
		}
		//着陆台上有球没盘
		if(gRobot.plantState[gRobot.upperGun.shootCommand[i].plantNum].ball == 0&& \
			gRobot.plantState[gRobot.upperGun.shootCommand[i].plantNum].plate == 0)
		{
			if(gRobot.upperGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.upperGun.shootCommand[i].stepTargetShootTime = 1;
			}
			if(gRobot.upperGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.upperGun.shootCommand[i].stepTargetShootTime = 1;
			}				
		}
		//着陆台上没球有盘
		if(gRobot.plantState[gRobot.upperGun.shootCommand[i].plantNum].ball == 1&& \
			gRobot.plantState[gRobot.upperGun.shootCommand[i].plantNum].plate == 1)
		{
			if(gRobot.upperGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.upperGun.shootCommand[i].stepTargetShootTime = \
				gRobot.upperGun.actualStepShootTimes[gRobot.upperGun.shootCommand[i].plantNum][gRobot.upperGun.shootCommand[i].shootMethod];
			}
			if(gRobot.upperGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.upperGun.shootCommand[i].stepTargetShootTime = \
				gRobot.upperGun.actualStepShootTimes[gRobot.upperGun.shootCommand[i].plantNum][gRobot.upperGun.shootCommand[i].shootMethod];
			}				
		}
		//着陆台上没球没盘
		if(gRobot.plantState[gRobot.upperGun.shootCommand[i].plantNum].ball == 1&& \
			gRobot.plantState[gRobot.upperGun.shootCommand[i].plantNum].plate == 0)
		{
			if(gRobot.upperGun.shootCommand[i].shootMethod == SHOOT_METHOD1)
			{
				gRobot.upperGun.shootCommand[i].stepTargetShootTime = \
				gRobot.upperGun.actualStepShootTimes[gRobot.upperGun.shootCommand[i].plantNum][gRobot.upperGun.shootCommand[i].shootMethod];
			}
			if(gRobot.upperGun.shootCommand[i].shootMethod == SHOOT_METHOD2)
			{
				gRobot.upperGun.shootCommand[i].stepTargetShootTime = 1;
			}				
		}		
	}
	return GUN_NO_ERROR;
}

/*
*名称：ROBOT_LeftGunPoint1ShootTimes
*功能：计算左枪第一个发射位置的目标发射次数
*参数：
*status:
*/
unsigned char ROBOT_LeftGunPoint1ShootTimes(void)
{
	uint8_t i = 0;
	uint8_t point1ShootTimes = 0;
	for(i = 0;i < LEFT_GUN_AUTO_SHOOT_STEP_NUMBER;i++)
	{
		if(gRobot.leftGun.shootCommand[i].shootPoint == SHOOT_POINT1)
		{
			point1ShootTimes+=gRobot.leftGun.shootCommand[i].stepTargetShootTime;
		}	
	}
	return point1ShootTimes;
}
/*
*名称：ROBOT_RightGunPoint1ShootTimes
*功能：计算右枪第一个发射位置的目标发射次数
*参数：
*status:
*/
unsigned char ROBOT_RightGunPoint1ShootTimes(void)
{
	uint8_t i = 0;
	uint8_t point1ShootTimes = 0;
	for(i = 0;i < RIGHT_GUN_AUTO_SHOOT_STEP_NUMBER;i++)
	{
		if(gRobot.rightGun.shootCommand[i].shootPoint == SHOOT_POINT1)
		{
			point1ShootTimes+=gRobot.rightGun.shootCommand[i].stepTargetShootTime;
		}	
	}
	return point1ShootTimes;
}
/*
*名称：ROBOT_LeftGunPoint3ShootTimes
*功能：计算左枪第一个发射位置的目标发射次数
*参数：
*status:
*/
unsigned char ROBOT_LeftGunPoint3ShootTimes(void)
{
	uint8_t i = 0;
	uint8_t point3ShootTimes = 0;
	for(i = 0;i < LEFT_GUN_AUTO_SHOOT_STEP_NUMBER;i++)
	{
		if(gRobot.leftGun.shootCommand[i].shootPoint == SHOOT_POINT3)
		{
			point3ShootTimes+=gRobot.leftGun.shootCommand[i].stepTargetShootTime;
		}	
	}
	return point3ShootTimes;
}
/*
*名称：ROBOT_RightGunPoint3ShootTimes
*功能：计算右枪第一个发射位置的目标发射次数
*参数：
*status:
*/
unsigned char ROBOT_RightGunPoint3ShootTimes(void)
{
	uint8_t i = 0;
	uint8_t point3ShootTimes = 0;
	for(i = 0;i < RIGHT_GUN_AUTO_SHOOT_STEP_NUMBER;i++)
	{
		if(gRobot.rightGun.shootCommand[i].shootPoint == SHOOT_POINT3)
		{
			point3ShootTimes+=gRobot.rightGun.shootCommand[i].stepTargetShootTime;
		}	
	}
	return point3ShootTimes;
}


