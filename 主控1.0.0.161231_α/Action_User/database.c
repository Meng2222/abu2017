/**
  ******************************************************************************
  * @file	 database.c
  * @author  ACTION_2017
  * @version V0.0.0.170321_alpha
  * @date	 2017/03/21
  * @brief   This file contains all data for shoot and move
  *
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "database.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup Gun_Shoot_Position_Database
  * @brief
  * @{
  */

//左侧枪姿态数据库
//{yaw, pitch, row, speed1, speed2}
gun_pose_t gLeftGunPosDatabase[SHOOT_POINT_NUMBER][SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{
	//第一个发射位置，对应靠近装载区位置
	{
		//第一类参数，对应打球参数
		{//1#着陆台
		 {-11.0f, 11.0f, 18.6f, 93.0f, 12.0f},
		 //2#着陆台
		 {4.5f, 16.0f, 24.4f, 99.0f, 15.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}},

		//第二类参数，对应落盘参数
		{//1#着陆台
		 {-18.0f, 27.5f, 23.6f, 92.0f, 7.0f},
		 //2#着陆台
		 {2.5f, 30.2f, 23.4f, 99.0f, 15.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}}

	},
	//第二个发射位置，对应靠近出发区位置
	{
		//第一类参数，对应打球参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}},

		//第二类参数，对应落盘参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}}

	},
	//第三个发射位置，对应场地中点
	{
		//第一类参数，对应打球参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-4.0f, 24.8f, 16.9f, 109.0f, 8.0f},
		 //4#着陆台
		 {16.2f, 17.0f, 18.5f, 111.0f,4.0f},
		 //5#着陆台
		 {31.1f,11.0f, 20.4f,114.0f,5.0f},
		 //6#着陆台
		// {-9.8f,14.9f, 22.3f,122.0f,13.0f},
		  {-7.3f,13.9f, 21.8f,128.0f,13.0f},
		 //7#着陆台
		 {0.0f,26.4f, 20.4f,77.0f,12.0f}},
	

		//第二类参数，对应落盘参数
		{//1#着陆台
		// {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		  {-38.9f, 20.5f, 19.6f, 101.0f, 19.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-8.0f, 32.0f, 19.8f, 104.0f, 12.0f},
		 //4#着陆台
		 {15.7f, 26.7f, 23.1f, 98.0f,16.0f},
		 //5#着陆台
		 {29.2f,22.5f, 23.4f,108.0f,9.0f},
		 //6#着陆台
		 //{-12.4f,24.4f, 25.7f,116.0f,19.0f},
		 {-7.3f,24.9f,19.3f,141.0f,13.0f},
		 //7#着陆台
		 {0.0f,40.0f, 20.4f,54.0f,12.0f}}
	}
};

//右侧枪姿态数据库
gun_pose_t gRightGunPosDatabase[SHOOT_POINT_NUMBER][SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{
	//第一个发射位置，对应靠近装载区位置
	{
		//第一类参数，对应打球参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}},

		//第二类参数，对应落盘参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}}

	},
	//第二个发射位置，对应靠近出发区位置
	{
		//第一类参数，对应打球参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}},

		//第二类参数，对应落盘参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}}

	},
	//第三个发射位置，对应场地中点
	{
		//第一类参数，对应打球参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,27.9f, 3.7f,96.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}},

		//第二类参数，对应落盘参数
		{//1#着陆台
		 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
		 //2#着陆台
		 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
		 //3#着陆台
		 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
		 //4#着陆台
		 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
		 //5#着陆台
		 {45.8f,32.5f, 40.6f,81.0f,24.0f},
		 //6#着陆台
		 {0.5f,26.9f, 3.7f,93.0f,15.0f},
		 //7#着陆台
		 {26.2f,34.0f, 20.4f,83.0f,24.0f}}

	}
};

//上面枪姿态数据库
gun_pose_t gUpperGunPosDatabase[SHOOT_METHOD_NUMBER][ZONE_NUMBER] = \
{
	//第1种子弹状态，对应的7个着陆台的枪的姿态
	{{-4.7f, -0.3f, 0.0f, 111.0f, 0.0f},
	{-0.2f, 0.1f, 0.0f, 116.0f, 0.0f},
	{-0.2f, -0.9f, 0.0f, 116.0f, 0.0f},
	{3.9f, -0.9f, 0.0f, 116.0f, 0.0f}
	},


	//第2种子弹状态，对应的7个着陆台的枪的姿态
	{{0.0f, 1.0f, 2.0f, 3.0f, 4.0f},
	{5.0f, 6.0f, 7.0f, 8.0f, 9.0f},
	{0.1f, 0.2f, 0.3f, 0.4f, 0.5f},
	{0.6f, 0.7f, 0.8f, 0.9f, 1.0f}}


};


/**
  * @}
  */


/** @defgroup Gun_Shoot_Command
  * @brief
  * @{
  */


//左枪射击柱子的顺序
shoot_command_t gLeftGunShootCmds[LEFT_GUN_POINT1_AUTO_BULLET_NUMBER+LEFT_GUN_POINT2_AUTO_BULLET_NUMBER+LEFT_GUN_POINT3_AUTO_BULLET_NUMBER] = \
{

	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD1},
	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD1},
	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD2},
	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD2},
	{SHOOT_POINT1, PLANT2,	SHOOT_METHOD1},
	{SHOOT_POINT1, PLANT2,	SHOOT_METHOD1},
	{SHOOT_POINT1, PLANT2,	SHOOT_METHOD2},
	{SHOOT_POINT1, PLANT2,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT3,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT3,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT3,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT3,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD1},	
	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD1},	
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD2},
	
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD2},

	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1},	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2},
	//****************************************
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2},
	//****************************************
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2}



};

//右枪射击柱子的顺序
shoot_command_t gRightGunShootCmds[RIGHT_GUN_POINT1_AUTO_BULLET_NUMBER+RIGHT_GUN_POINT2_AUTO_BULLET_NUMBER+RIGHT_GUN_POINT3_AUTO_BULLET_NUMBER] = \
{

	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD1},
	{SHOOT_POINT1, PLANT2,	SHOOT_METHOD1},
	{SHOOT_POINT1, PLANT3,	SHOOT_METHOD1},
	{SHOOT_POINT1, PLANT4,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD1},
	{SHOOT_POINT3, PLANT1,	SHOOT_METHOD1},
	{SHOOT_POINT2, PLANT2,	SHOOT_METHOD1},
	{SHOOT_POINT2, PLANT3,	SHOOT_METHOD1},
	{SHOOT_POINT2, PLANT2,	SHOOT_METHOD1},
	{SHOOT_POINT2, PLANT3,	SHOOT_METHOD1}

};

////上面枪射击柱子的顺序
//shoot_command_t gUpperGunShootCmds = \
//{
//	{
//		{PLANT1,	SHOOT_METHOD1},
//		{PLANT2,	SHOOT_METHOD1},
//		{PLANT3,	SHOOT_METHOD1},
//		{PLANT4,	SHOOT_METHOD1},
//		{PLANT5,	SHOOT_METHOD1},
//		{PLANT6,	SHOOT_METHOD1},
//		{PLANT7,	SHOOT_METHOD1},
//		{PLANT1,	SHOOT_METHOD1},
//		{PLANT2,	SHOOT_METHOD1},
//		{PLANT3,	SHOOT_METHOD1}
//	}
//};

/**
  * @}
  */


/** @defgroup Walk_Track_Database
  * @brief    此结构体数组的大小为 WALKTRACKDATABASE_POINT_NUMBER * 3 * 4 Byte
  * @{
  */

posture_t gWalkTrackDatabase[WALKTRACKDATABASE_POINT_CAPACITY]={0};

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern robot_t gRobot;

/**
  * @brief  Update LeftGunPosDatabase in Manual Mode
  * @note   this function will update the 
  * @param  None
  * @retval None
  */
void UpdateLeftGunPosDatabaseManulMode(void)
{
	if(gRobot.moveBase.targetPoint == 2)
	{
		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].yaw = gRobot.leftGun.targetPose.yaw;
		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].pitch = gRobot.leftGun.targetPose.pitch;
		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].roll = gRobot.leftGun.targetPose.roll;
		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed1 = gRobot.leftGun.targetPose.speed1;
		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed2 = gRobot.leftGun.targetPose.speed2;
	
	}
	else if(gRobot.moveBase.targetPoint == 3)
	{
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].yaw = gRobot.leftGun.targetPose.yaw;
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].pitch = gRobot.leftGun.targetPose.pitch;
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].roll = gRobot.leftGun.targetPose.roll;
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed1 = gRobot.leftGun.targetPose.speed1;
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed2 = gRobot.leftGun.targetPose.speed2;
	}
	else if(gRobot.moveBase.targetPoint == 1)
	{
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].yaw = gRobot.leftGun.targetPose.yaw;
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].pitch = gRobot.leftGun.targetPose.pitch;
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].roll = gRobot.leftGun.targetPose.roll;
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed1 = gRobot.leftGun.targetPose.speed1;
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed2 = gRobot.leftGun.targetPose.speed2;
	}
}
/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

