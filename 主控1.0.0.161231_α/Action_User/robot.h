#ifndef __ROBOT_H
#define __ROBOT_H
#include "stdint.h"
/**************************************************************************************
 宏定义
**************************************************************************************/


#define LEFT_GUN				1
#define RIGHT_GUN				2
#define UPPER_GUN				3

//左枪组ID号
#define RIGHT_GUN_GROUP_ID		101
//右枪组ID号
#define LEFT_GUN_GROUP_ID		102
//上面枪组ID号
#define UPPER_GUN_GROUP_ID		103
//左枪速度组ID
#define RIGHT_GUN_VEL_GROUP_ID   104
//右枪速度组ID
#define LEFT_GUN_VEL_GROUP_ID  105
//上枪速度组ID
#define UPPER_GUN_VEL_GROUP_ID  106


//左枪支架roll轴CAN ID
#define RIGHT_GUN_ROLL_ID 7
//左枪支架pitch轴CAN ID
#define RIGHT_GUN_PITCH_ID 6
//左枪支架yaw轴CAN ID
#define LEFT_GUN_YAW_ID 8
//左枪左侧传送带轴CAN ID
#define RIGHT_GUN_LEFT_ID 4
//左枪右侧传送带轴CAN ID
#define RIGHT_GUN_RIGHT_ID 5

//右枪支架roll轴CAN ID
#define LEFT_GUN_ROLL_ID 15
//右枪支架pitch轴CAN ID
#define LEFT_GUN_PITCH_ID 14
//右枪支架yaw轴CAN ID
#define RIGHT_GUN_YAW_ID 16
//右枪左侧传送带轴CAN ID
#define LEFT_GUN_LEFT_ID 12
//右枪右侧传送带轴CAN ID
#define LEFT_GUN_RIGHT_ID 13

//上面枪支架pitch轴CAN ID
#define UPPER_GUN_PITCH_ID 11
//上面枪支架yaw轴CAN ID
#define UPPER_GUN_YAW_ID 10
//上面枪左侧传送带轴CAN ID
#define UPPER_GUN_LEFT_ID 9



/**************************************************************************************
 类型定义
**************************************************************************************/
typedef struct
{
	uint32_t canSendId;
	uint8_t message[8];
}canMsg_t;

typedef int status_t;

typedef struct
{
	//枪航向角度
	float yaw;
	//枪俯仰角度
	float pitch;
	//枪横滚角度
	float roll;

	//左传送带转速，单位转/秒
	float speed1;
	//右传送带转速，单位转/秒
	float speed2;
}gun_pose_t;
/*
* 枪结构体
*/
typedef struct
{
	//枪的目标姿态
	gun_pose_t targetPose;
	//枪的目标姿态
	gun_pose_t actualPose;

}gun_t;


//机器人结构体封装了机器的关键数据，为全局数据，此结构体暂时放在此处
typedef struct 
{	
	//机器人左边枪
	gun_t leftGun;
	//机器人右边枪
	gun_t rightGun;
	//机器人上边枪
	gun_t upperGun;
	
}robot_t;





/*
============================================================
                   枪参数变换与逆变换            
============================================================
*/



float LeftGunYawInverseTransform(int32_t position);
float LeftGunPitchInverseTransform(int32_t position);
float LeftGunRollInverseTransform(int32_t position);
float LeftGunLeftSpeedInverseTransform(int32_t speed);
float LeftGunRightSpeedInverseTransform(int32_t speed);


float RightGunYawInverseTransform(int32_t position);
float RightGunPitchInverseTransform(int32_t position);
float RightGunRollInverseTransform(int32_t position);
float RightGunLeftSpeedInverseTransform(int32_t speed);
float RightGunRightSpeedInverseTransform(int32_t speed);


float UpperGunYawInverseTransform(int32_t position);
float UpperGunPitchInverseTransform(int32_t position);
float UpperGunLeftSpeedInverseTransform(int32_t speed);


#endif
