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

gun_pose_t gLeftGunReloadPosDatabase[SHOOT_POINT_NUMBER][SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
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

//右侧枪姿态数据库
gun_pose_t gRightGunReloadPosDatabase[SHOOT_POINT_NUMBER][SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
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
gun_pose_t gUpperGunPosDatabase[LAND_NUMBER][SHOOT_METHOD_NUMBER][ZONE_NUMBER] = \
{
	
	//一号柱子对应参数
	{
		//第一种参数类型对应打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	},
	//二号柱子对应参数
	{
		//第一种参数类型对应打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	},	
	//三号柱子对应参数
	{
		//第一种参数类型对应打球
		{
			{2.2f,10.9f,0.0f,104.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{11.4f,22.4f,0.0f,97.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	},	
	//四号柱子对应参数
	{
		//第一种参数类型对应打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
		},
		//第二种参数类型对应落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	},	
	//五号柱子对应参数
	{
		//第一种参数类型对应打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	},	
	//六号柱子对应参数
	{
		//第一种参数类型对应打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	},	
	//七号柱子对应参数
	{
		//第一种参数类型对应打球
		{
			{-3.2f, -2.4f, 0.0f, 121.0f, 0.0f},
			{0.4f, -2.3f, 0.0f, 121.0f, 0.0f},
			{-1.4f, -3.0f, 0.0f, 121.0f, 0.0f},
			{4.6f, -2.4f, 0.0f, 121.0f, 0.0f}
		},
		//第二种参数类型对应落盘
		{
			{8.1f,23.3f,0.0f,48.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	}
};


/**
  * @}
  */

/** @defgroup LeftGunPriority
  * @brief
  * @{
  */
uint8_t LeftGunPriority[7] = {PLANT1,PLANT2,PLANT7,PLANT3,PLANT6,PLANT4,PLANT5};

/** @defgroup LeftGunPriority
  * @brief
  * @{
  */
uint8_t RightGunPriority[7] = {PLANT5,PLANT4,PLANT6,PLANT3,PLANT7,PLANT2,PLANT1};



/** @defgroup Gun_Shoot_Command
  * @brief
  * @{
  */


//左枪射击柱子的顺序
shoot_command_t gLeftGunShootCmds[LEFT_GUN_AUTO_SHOOT_STEP_NUMBER] = \
{

	{SHOOT_POINT3, PLANT1,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT1,	SHOOT_METHOD2 , 2},
//	{SHOOT_POINT1, PLANT2,	SHOOT_METHOD1 , 2},
//	{SHOOT_POINT1, PLANT2,	SHOOT_METHOD2 , 2},
//	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD1 , 2},	
//	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD2 , 2},
//	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD1 , 2},	
//	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD2 , 2},
	{SHOOT_POINT3, PLANT2,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT2,	SHOOT_METHOD2 , 2},	
//	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD1 , 1},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3}
};

//右枪射击柱子的顺序
shoot_command_t gRightGunShootCmds[RIGHT_GUN_AUTO_SHOOT_STEP_NUMBER] = \
{

//	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD1 , 2},
//	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD2 , 2},
	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD1 , 2},	
	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD2 , 2},
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD1 , 2},	
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD2 , 2},
//	{SHOOT_POINT3, PLANT3,	SHOOT_METHOD1 , 2},
//	{SHOOT_POINT3, PLANT3,	SHOOT_METHOD2 , 2}
//	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD1 , 1},
//	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD2 , 1},
//	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3}

};

////上面枪射击柱子的顺序
shoot_command_t gUpperGunShootCmds[UPPER_GUN_AUTO_STEP_NUMBER] = \
{
	{PLANT7,SHOOT_METHOD2,0,1},
	{PLANT3,SHOOT_METHOD1,0,1},
	{PLANT3,SHOOT_METHOD2,0,1}	
};

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
void UpdateLeftGunPosDatabaseManualMode(void)
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
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].yaw = gRobot.leftGun.targetPose.yaw;
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].pitch = gRobot.leftGun.targetPose.pitch;
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].roll = gRobot.leftGun.targetPose.roll;
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed1 = gRobot.leftGun.targetPose.speed1;
		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed2 = gRobot.leftGun.targetPose.speed2;
	}
	else if(gRobot.moveBase.targetPoint == 1)
	{
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].yaw = gRobot.leftGun.targetPose.yaw;
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].pitch = gRobot.leftGun.targetPose.pitch;
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].roll = gRobot.leftGun.targetPose.roll;
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed1 = gRobot.leftGun.targetPose.speed1;
		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed2 = gRobot.leftGun.targetPose.speed2;
	}
}

/**
  * @brief  Update RightGunPosDatabase in Manual Mode
  * @note   this function will update the 
  * @param  None
  * @retval None
  */
void UpdateRightGunPosDatabaseManualMode(void)
{
	if(gRobot.moveBase.targetPoint == 2)
	{
		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].yaw = gRobot.rightGun.targetPose.yaw;
		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].pitch = gRobot.rightGun.targetPose.pitch;
		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].roll = gRobot.rightGun.targetPose.roll;
		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed1 = gRobot.rightGun.targetPose.speed1;
		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed2 = gRobot.rightGun.targetPose.speed2;
	
	}
	else if(gRobot.moveBase.targetPoint == 3)
	{
		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].yaw = gRobot.rightGun.targetPose.yaw;
		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].pitch = gRobot.rightGun.targetPose.pitch;
		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].roll = gRobot.rightGun.targetPose.roll;
		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed1 = gRobot.rightGun.targetPose.speed1;
		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed2 = gRobot.rightGun.targetPose.speed2;
	}
	else if(gRobot.moveBase.targetPoint == 1)
	{
		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].yaw = gRobot.rightGun.targetPose.yaw;
		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].pitch = gRobot.rightGun.targetPose.pitch;
		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].roll = gRobot.rightGun.targetPose.roll;
		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed1 = gRobot.rightGun.targetPose.speed1;
		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed2 = gRobot.rightGun.targetPose.speed2;
	}
}
/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

