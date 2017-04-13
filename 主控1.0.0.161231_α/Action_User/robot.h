#ifndef __ROBOT_H
#define __ROBOT_H
#include "stdint.h"
#include "movebase.h"
/**************************************************************************************
 宏定义
**************************************************************************************/

//着陆台个数
#define LAND_NUMBER 7
//机器人枪的个数
#define GUN_NUMBER 3
//子弹在枪膛里状态种类，由光线传感器
#define BULLET_TYPE_NUMBER 10
//参数种类，打球，落盘
#define SHOOT_METHOD_NUMBER		2
//上枪参数种类
#define UPPER_SHOOT_METHOD_NUMBER  3
//发射位置个数
#define SHOOT_POINT_NUMBER 3
//防守台分区数
#define ZONE_NUMBER 4

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

//机器人拉保险完成
#define GUN_OPENSAFTY_READY     1
//底盘已经到达发射点位
#define MOVEBASE_POS_READY      1

#define GUN_AUTO_MODE			0
#define GUN_MANUAL_MODE			1

#define GUN_ATTACK_MODE         2
#define GUN_DEFEND_MODE         3

#define GUN_START_SHOOT 1
#define GUN_STOP_SHOOT 0

#define GUN_START_AIM			1
#define GUN_STOP_AIM			0

//是否已经上弹标志位
#define GUN_ALREADY_RELOAD  1
#define GUN_NOT_RELOAD  0

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

//上弹成功
#define GUN_RELOAD_OK 1
//射击完成
#define GUN_NO_ERROR 0
//枪膛卡弹
#define GUN_CHAMPER_ERROR -1
//弹夹空
#define GUN_NO_BULLET_ERROR -2
//枪架未就位， fix me
#define GUN_NO_READY_ERROR -3
//上弹失败
#define GUN_RELOAD_ERROR -4
//打开保险失败
#define GUN_OPEN_SAFETY_ERROR -5

//枪最大子弹数
#define MAX_BULLET_NUMBER_LEFT 20
#define MAX_BULLET_NUMBER_RIGHT 20
#define MAX_BULLET_NUMBER_UPPER 10
//枪最大自动发射子弹发数
#define MAX_AUTO_BULLET_NUMBER 12

//左枪1点（靠近装载区）自动发射子弹数目
#define LEFT_GUN_POINT1_AUTO_BULLET_NUMBER  4
//左枪2点（靠近出发区）自动发射子弹数目
#define LEFT_GUN_POINT2_AUTO_BULLET_NUMBER  0
//左枪3点（中点）自动发射子弹数目
#define LEFT_GUN_POINT3_AUTO_BULLET_NUMBER  10//18
//右枪1点（靠近装载区）自动发射子弹数目
#define RIGHT_GUN_POINT1_AUTO_BULLET_NUMBER  4//8
//右枪2点（靠近装载区）自动发射子弹数目
#define RIGHT_GUN_POINT2_AUTO_BULLET_NUMBER  0
//右枪3点（中点）自动发射子弹数目
#define RIGHT_GUN_POINT3_AUTO_BULLET_NUMBER  11

//瞄准未完成
#define GUN_AIM_IN_PROCESS 1
//瞄准完成
#define GUN_AIM_DONE 2

//进入下一步
#define GUN_NEXT_STEP  1
//不进入下一步
#define GUN_PRESENT_STEP 0


//机器人已上电
#define ROBOT_STAGE_POWER_ON 0
//机器人已经初始化
#define ROBOT_STAGE_INIT 1
//机器人处于加速过程中
#define ROBOT_STAGE_RUN_ACC 2
//机器人处于匀速运动阶段
#define ROBOT_STAGE_RUN_UNIFORM 3
//机器人进入减速阶段
#define ROBOT_STAGE_RUN_DEC 4
//机器人抓取子弹
#define ROBOT_STAGE_LOAD_GUN 5
//机器人将弹夹转向上，相当于拉保险
#define ROBOT_STAGE_OPEN_GUN_SAFETY 6
//机器人停止所有走行运动
#define ROBOT_END_RUNNING 7
//机器人开始射击
#define ROBOT_START_SHOOT 8

#define ROBOT_STATUS_OK 0
#define ROBOT_STATUS_OVER_LOAD 0x01
#define ROBOT_STATUS_OVER_TEMPERATURE 0x02
#define ROBOT_STATUS_UNDER_VOLTAGE 0x04
#define ROBOT_STATUS_GAS_LOW  0x08

//自动模式下总步数
#define LEFT_GUN_AUTO_SHOOT_STEP_NUMBER 10
#define RIGHT_GUN_AUTO_SHOOT_STEP_NUMBER 10//14
//上枪自动射击步数
#define UPPER_GUN_AUTO_STEP_NUMBER 4


//着陆台编号
#define INVALID_PLANT_NUMBER 7
#define PLANT1 0
#define PLANT2 1
#define PLANT3 2
#define PLANT4 3
#define PLANT5 4
#define PLANT6 5
#define PLANT7 6

//发射参数模式，对应打球和落盘
//打球模式
#define SHOOT_METHOD1  0
//落盘模式
#define SHOOT_METHOD2  1
//打球模式
#define SHOOT_METHOD3  2

//发射点位置
//靠近装载区发射点
#define SHOOT_POINT1 0
//靠近出发区发射点
#define SHOOT_POINT2 1
//场地中央发射点
#define SHOOT_POINT3 2

#define INVALID_ZONE_NUMBER 0u
//7#着陆台左侧区域
#define ZONE1 0u
//7#着陆台中后区域
#define ZONE2 1u
//7#着陆台中前区域
#define ZONE3 2u
//7#着陆台右侧区域
#define ZONE4 3u

//枪自动射击时命令结构体
typedef struct
{
	uint8_t shootPoint;
	uint8_t plantNum;
	uint8_t shootMethod;
	uint8_t stepTargetShootTime;
}shoot_command_t;

/**************************************************************************************
 类型定义
**************************************************************************************/

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
	
	//枪姿态上限
	gun_pose_t maxPoseLimit;
	
	//枪姿态下限
	gun_pose_t minPoseLimit;
	
	//枪支架是否准备好：1就绪0准备中
	unsigned char ready;
	
	//枪的模式：0自动，1手动
	unsigned char mode;
	
	//射击参数模式：1 打球 2 打盘 3 扔
	unsigned char shootParaMode;
	
	//弹夹内子弹数，因为最多23发，所以用8位
	signed char bulletNumber;
	
	//枪膛是否卡弹:1卡弹，0正常
	unsigned char champerErrerState;
	
	//射击命令：1射击，0不射击
	unsigned char shoot;
	
	//瞄准命令：1瞄准，0不瞄准
	unsigned char aim;
	//如果上面char个数不是4的倍数，需要使用dummy对齐，dummy没有任何含义
	//unsigned char dummy[2];
	
	//gunPosDatabase指向枪的姿态数据库，数据库存储在database.c中，初始化时需要指定
	gun_pose_t **gunPoseDatabase;
	//命令集
	shoot_command_t *shootCommand;
	//目标着陆台号，只有在手动模式下才生效，自动模式下忽略
	int targetPlant;
	//防守台分区，用于上枪打盘
	int targetZone;
	//射击次数
	int shootTimes;
	//检查到位使用的时间
	int checkTimeUsage;
	//标志枪中是否已经上弹
	unsigned char reloadState;
	//射击步数，对于步的定义暂时为在一个位置打一个柱子的一种参数
	unsigned char shootStep;
	//下一步标志位,用于接收平板指令   置1为下一步
	unsigned char nextStep;
	//下一步标志位，用于GunTask
	unsigned char stepState;
	//射击过程中每一步命令发射次数
	unsigned char targetStepShootTimes;
	//射击过程中每一步实际发射次数
	unsigned char actualStepShootTimes[7][2];
	
}gun_t;

//着陆台状态
typedef struct
{
	//是否有球
	unsigned char ball;
	//是否已经落上盘
	unsigned char plate;
}plant_t;
//着陆台类型
typedef struct
{
	int size;
}land_t;

//机器人结构体封装了机器的关键数据，为全局数据，此结构体暂时放在此处
typedef struct 
{	
	movebase_t moveBase;
	//机器人左边枪
	gun_t leftGun;
	//机器人右边枪
	gun_t rightGun;
	//机器人上边枪
	gun_t upperGun;

	//机器人总射击次数，为(leftGun.shootTimes+rightGun.shootTimes + upperGun.shootTimes)
	int shootTimes;
	
	//机器人所处的阶段：未初始化，初始化，加速，匀速，减速，取弹，上战场，开枪
	int stage;
	//机器人状态，是否正常：低压、过流、过温、
	int status;
	//着陆台状态
	plant_t plantState[7];
	
}robot_t;

/*
* 场地信息，尺寸，包含着陆台
*/
typedef struct
{
	int size;
}field_t;


/**************************************************************************************
 函数定义，以下机器人操作函数会读写robot_t类型的机器人实例变量
！！！！！！！！每个枪的操作函数需要支持重入
**************************************************************************************/
/*
*名称：ROBOT_Init
*功能：机器人初始化，初始化底盘，初始化枪，初始化
*参数：none
*注意：上面的枪不需要上子弹，因为是手动上弹
*/
status_t ROBOT_Init(void);

/*
*名称：ROBOT_GunLoad
*功能：安装弹夹，即抓取子弹过程
*参数：
*status:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹，因为是手动上弹
*/
status_t ROBOT_GunLoad(void);

/*
*名称：ROBOT_GunOpenSafety
*功能：拉开枪保险，子弹安装好后才能进行此步骤
*参数：
*status:GUN_NO_ERROR，GUN_OPEN_SAFETY_ERROR
*注意：上面的枪不需要
*/
status_t ROBOT_GunOpenSafety(void);
/*
*名称：ROBOT_CheckGunOpenSafety
*功能：检查拉开枪保险
*参数：
*status:GUN_NO_ERROR，GUN_OPEN_SAFETY_ERROR
*注意：上面的枪不需要
*/
status_t ROBOT_CheckGunOpenSafety(void);

/**
*名称：ROBOT_LeftGunReload
*功能：左枪上弹，每次射击前需要上弹一次
*		通过发送CAN命令给间隔一段时间后
*@param None
*@retval status_t:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹
*/
status_t ROBOT_LeftGunReload(void);
/**
*名称：ROBOT_RightGunReload
*功能：右枪上弹，每次射击前需要上弹一次
*		通过发送CAN命令给间隔一段时间后
*@param None
*@retval status_t:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹
*/
status_t ROBOT_RightGunReload(void);

/**
*名称：ROBOT_LeftGunCheckReload
*功能：检查左枪上弹情况
*@param None
*@retval status_t:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹
*/
status_t ROBOT_LeftGunCheckReload(void);

/**
*名称：ROBOT_LeftGunCheckReload
*功能：检查右枪上弹情况
*@param None
*@retval status_t:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹
*/
status_t ROBOT_RightGunCheckReload(void);
/*
*名称：ROBOT_GunReload
*功能：根据枪膛传感器，检测子弹状态，决定后面开枪的具体参数
*参数：
*gun :LEFT_GUN, RIGHT_GUN
*status:GUN_NO_ERROR，GUN_RELOAD_ERROR
*注意：上面的枪不需要上子弹，因为是手动上弹
*/
status_t ROBOT_GunCheckBulletState(unsigned char gun);


/** @defgroup Shoot Tragedy
  * @brief 
  * @{
  */

/**
  * @brief	Get left gun shoot command
  * @note	
  * @param	
  *     @arg	
  * @param	
  * @retval	
  */
shoot_command_t ROBOT_LeftGunGetShootCommand(void);

/**
  * @}
  */

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

shoot_command_t ROBOT_RightGunGetShootCommand(void);

/**
  * @}
  */

/**
*名称： ROBOT_LeftGunAim ROBOT_RightGunAim ROBOT_UpperGunAim
*功能： 瞄准，目标改变后需要先调用此接口来重新瞄准,此函数将发送CAN命令
*	上面的枪目前机械上没有roll，没有右侧传送带speed2
*	故 ROBOT_LeftGunAim 和 ROBOT_LeftGunAim 各发送5条CAN命令
*	ROBOT_UpperGunAim仅发送3条CAN命令
*参数： None
* @retval status:GUN_NO_ERROR
*/
status_t ROBOT_LeftGunAim(void);
status_t ROBOT_RightGunAim(void);
status_t ROBOT_UpperGunAim(void);

/*
*名称：ROBOT_LeftGunCheckShootPoint
*功能：检查底盘是否走到位
*参数：
*none
*status:
*注意：
*/
 status_t ROBOT_LeftGunCheckShootPoint(void);

/*
*名称：ROBOT_RightGunCheckShootPoint
*功能：检查底盘是否走到位
*参数：
*none
*status:
*注意：
*/
status_t ROBOT_RightGunCheckShootPoint(void);


/*
*名称：ROBOT_LeftGunCheckAim
*功能：检查瞄准是否已完成
*参数：
*none
*status:GUN_AIM_IN_PROCESS， GUN_AIM_DONE
*注意：
*/
status_t ROBOT_LeftGunCheckAim(void);
/*
*名称：ROBOT_RightGunCheckAim
*功能：检查瞄准是否已完成，不同枪分开检测为了防止重入，
*因为此函数中需要设计超时
*参数：
*none
*status:GUN_AIM_IN_PROCESS， GUN_AIM_DONE
*注意：
*/
status_t ROBOT_RightGunCheckAim(void);

/*
*名称：ROBOT_UpperGunCheckAim
*功能：检查瞄准是否已完成
*参数：
*none
*status:GUN_AIM_IN_PROCESS， GUN_AIM_DONE
*注意：
*/
status_t ROBOT_UpperGunCheckAim(void);

/**
*名称：ROBOT_LeftGunShoot
*功能：左枪开枪，开枪前需要确保子弹上膛，拉开保险，枪支架已经就绪
*@param None
*@retval status:GUN_NO_ERROR，GUN_CHAMPER_ERROR， GUN_NO_BULLET_ERROR， GUN_NO_READY_ERROR
*/
status_t ROBOT_LeftGunShoot(void);
/**
*名称：ROBOT_RightGunShoot 
*功能：右枪开枪，开枪前需要确保子弹上膛，拉开保险，枪支架已经就绪
*@param None
*@retval status:GUN_NO_ERROR，GUN_CHAMPER_ERROR， GUN_NO_BULLET_ERROR， GUN_NO_READY_ERROR
*/
status_t ROBOT_RightGunShoot(void);
/**
*名称：ROBOT_UpperGunShoot
*功能：上枪开枪，开枪前需要确保子弹足够
*@param None
*@retval status:GUN_NO_ERROR，GUN_CHAMPER_ERROR， GUN_NO_BULLET_ERROR， GUN_NO_READY_ERROR
*/
status_t ROBOT_UpperGunShoot(void);

/**
*@name ROBOT_LeftGunHome
*功能：左枪归位，开枪后为了更好的上膛需要归位
*@param None
*@retval status:GUN_NO_ERROR
*@note fix me, 此处发出命令后等待两秒以确保其能够归位，应加位置检测
*/
status_t ROBOT_LeftGunHome(void);
/**
*@name ROBOT_LeftGunHome
*功能：左枪归位，开枪后为了更好的上膛需要归位
*@param None
*@retval status:GUN_NO_ERROR
*@note fix me, 此处发出命令后等待两秒以确保其能够归位，应加位置检测
*/
status_t ROBOT_RightGunHome(void);
/*
*名称：ROBOT_GunCheckMode
*功能：检查枪的模式
*参数：
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*status:
*/
status_t ROBOT_GunCheckMode(unsigned char gun);

/*
*名称：ROBOT_LeftGunCheckPlantState
*功能：左枪检查各个柱子状态并更新指令
*参数：
*status:
*/
status_t ROBOT_LeftGunCheckPlantState(void);

/*
*名称：ROBOT_RightGunCheckPlantState
*功能：左枪检查各个柱子状态并更新指令
*参数：
*status:
*/
status_t ROBOT_RightGunCheckPlantState(void);
/*
*名称：ROBOT_UpperGunCheckPlantState
*功能：左枪检查各个柱子状态并更新指令
*参数：
*status:
*/
status_t ROBOT_UpperGunCheckPlantState(void);

/*
*名称：ROBOT_LeftGunReloadAim
*功能：左枪检查各个柱子状态并更新指令
*参数：
*status:
*/
status_t ROBOT_LeftGunReloadAim(void);
/*
*名称：ROBOT_RightGunReloadAim
*功能：左枪检查各个柱子状态并更新指令
*参数：
*status:
*/
status_t ROBOT_RightGunReloadAim(void);
/*
*名称：ROBOT_LeftGunPoint1ShootTimes
*功能：计算左枪第一个发射位置的目标发射次数
*参数：
*status:
*/
unsigned char ROBOT_LeftGunPoint1ShootTimes(void);
/*
*名称：ROBOT_RightGunPoint1ShootTimes
*功能：计算右枪第一个发射位置的目标发射次数
*参数：
*status:
*/
unsigned char ROBOT_RightGunPoint1ShootTimes(void);
/*
*名称：ROBOT_LeftGunPoint3ShootTimes
*功能：计算左枪第一个发射位置的目标发射次数
*参数：
*status:
*/
unsigned char ROBOT_LeftGunPoint3ShootTimes(void);
/*
*名称：ROBOT_RightGunPoint3ShootTimes
*功能：计算右枪第一个发射位置的目标发射次数
*参数：
*status:
*/
unsigned char ROBOT_RightGunPoint3ShootTimes(void);

/*
============================================================
                   枪参数变换与逆变换            
============================================================
*/

int32_t LeftGunYawTransform(float position);
int32_t LeftGunPitchTransform(float position);
int32_t LeftGunRollTransform(float position);
int32_t LeftGunLeftSpeedTransform(float speed);
int32_t LeftGunRightSpeedTransform(float speed);

float LeftGunYawInverseTransform(int32_t position);
float LeftGunPitchInverseTransform(int32_t position);
float LeftGunRollInverseTransform(int32_t position);
float LeftGunLeftSpeedInverseTransform(int32_t speed);
float LeftGunRightSpeedInverseTransform(int32_t speed);


int32_t RightGunYawTransform(float yaw);
int32_t RightGunPitchTransform(float pitch);
int32_t RightGunRollTransform(float roll);
int32_t RightGunLeftSpeedTransform(float speed);
int32_t RightGunRightSpeedTransform(float speed);

float RightGunYawInverseTransform(int32_t position);
float RightGunPitchInverseTransform(int32_t position);
float RightGunRollInverseTransform(int32_t position);
float RightGunLeftSpeedInverseTransform(int32_t speed);
float RightGunRightSpeedInverseTransform(int32_t speed);

int32_t UpperGunYawTransform(float yaw);
int32_t UpperGunPitchTransform(float pitch);
int32_t UpperGunLeftSpeedTransform(float speed);

float UpperGunYawInverseTransform(int32_t position);
float UpperGunPitchInverseTransform(int32_t position);
float UpperGunLeftSpeedInverseTransform(int32_t speed);

//temrary
typedef struct
{
	float yawAng;
	float pitchAng;
	float rollAng;
	float vel1;
	float vel2;
	int gunNum;
}shootCtr_t;

void ShootCtr(shootCtr_t *shootPara);

void LeftGunSendDebugInfo(void);
void RightGunSendDebugInfo(void);


#endif
