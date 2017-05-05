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
uint16_t recordPointTotalNum = 1u;
/** @defgroup Gun_Shoot_Position_Database
  * @brief
  * @{
  */

//左侧枪姿态数据库
//{yaw, pitch, row, speed1, speed2}
gun_pose_t gLeftGunPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{

		//第一类参数，对应打球参数
		{//1#着陆台
		 {-28.5f, 0.0f, 14.1f, 11.0f, 117.0f},
		 //2#着陆台
		 {-16.4f, 9.7f, 2.0f, 25.0f, 134.0f},
		 //3#着陆台
		 {1.5f, 22.8f, -0.9f, 6.0f, 110.0f},
		 //4#着陆台
		 {29.4f, 17.5f, 12.8f, 0.0f,113.0f},
		 //5#着陆台
		 {38.6f,13.0f, 8.4f,0.0f,112.0f},
		 //6#着陆台
		 {1.8f,9.0f,0.5f,22.0f,126.0f},
		 //7#着陆台
		 {5.5f, 25.4f, 7.9f,17.0f,81.0f}},

		//第二类参数，对应新盘落盘参数
		{//1#着陆台
		  {-34.6f, 20.2f, 16.0f, 20.0f, 93.0f},
		 //2#着陆台
//		 {-13.6f, 25.3f, 17.3f, 20.0f, 95.0f},
		 {-16.8f, 26.7f, 13.5f, 18.0f, 97.0f},
		 //3#着陆台
		 {2.7f, 26.9f, 6.6f, 20.0f, 96.0f},
		 //4#着陆台
		 {16.9f, 23.2f, 2.0f, 18.0f,98.0f},
		 //5#着陆台
		 {32.2f,16.6f, 4.4f,19.0f,100.0f},
		 //6#着陆台
		 {-3.1f,20.3f,2.0f,11.0f,129.0f},
		 //{-3.6f,19.8f,2.0f,21.0f,124.0f}
		 //7#着陆台
		 {4.5f,35.7f, 2.0f,14.0f,50.0f}},
		
		//第三类参数，对应打球同时落盘参数
		{//1#着陆台
		 {-28.0f, 7.0f, 14.1f, 10.0f, 113.0f},
		 //2#着陆台
		 {-16.4f, 15.7f, 7.3f, 5.0f, 106.0f},
		 //3#着陆台
		 {2.2f, 25.3f, 2.0f, 10.0f, 110.0f},
		 //4#着陆台
		 {29.4f, 17.5f, 12.8f, 0.0f,113.0f},
		 //5#着陆台
		 {38.6f,13.0f, 8.4f,0.0f,112.0f},
		 //6#着陆台
		 {0.2f,8.1f, 0.5f,27.0f,128.0f},
		 //7#着陆台
		 {6.0f, 25.9f,2.0f,16.0f,77.0f}},
		//第四类参数，对应旧盘落盘参数
		{//1#着陆台
		  {-33.1f, 19.2f, 16.0f, 22.0f, 94.0f},
		 //2#着陆台
//		 {-13.6f, 25.3f, 17.3f, 20.0f, 95.0f},
		 {-25.2f, 25.3f, 2.0f, 18.0f, 94.0f},
		 //3#着陆台
		 {-2.0f, 30.4f, 2.1f, 20.0f, 99.0f},
		 //4#着陆台
		 {16.9f, 23.2f, 2.0f, 18.0f,98.0f},
		 //5#着陆台
		 {32.2f,16.6f, 4.4f,19.0f,100.0f},
		 //6#着陆台
		 {-5.1f,19.0f,2.0f,15.0f,127.0f},
		 //7#着陆台
		 {4.5f,35.7f, 2.0f,14.0f,50.0f}},
		//第五类参数，对应新盘补弹打球参数
		{//1#着陆台
		 {-28.0f, 7.7f, 14.1f, 11.0f, 117.0f},
		 //2#着陆台
		 {-11.4f, 11.7f, 14.5f, 22.0f, 107.0f},
		 //3#着陆台
		 {2.2f, 25.3f, 2.0f, 10.0f, 110.0f},
		 //4#着陆台
		 {29.4f, 17.5f, 12.8f, 0.0f,113.0f},
		 //5#着陆台
		 {38.6f,13.0f, 8.4f,0.0f,112.0f},
		 //6#着陆台
		 {0.2f,8.1f, 0.5f,27.0f,128.0f},
		 //7#着陆台
		 {6.0f, 25.9f,2.0f,16.0f,77.0f}},
		//第六类参数，对应新盘补弹落盘参数
		{//1#着陆台
		  {-35.1f, 20.2f, 16.0f, 21.0f, 94.0f},
		 //2#着陆台
//		 {-13.6f, 25.3f, 17.3f, 20.0f, 95.0f},
		 {-22.2f, 24.2f, 6.0f, 18.0f, 94.0f},
		 //3#着陆台
		 {2.7f, 26.9f, 6.6f, 20.0f, 96.0f},
		 //4#着陆台
		 {16.9f, 23.2f, 2.0f, 18.0f,98.0f},
		 //5#着陆台
		 {32.7f,16.6f, 4.4f,18.0f,98.0f},
		 //6#着陆台
		 {-3.1f,20.3f,2.0f,11.0f,129.0f},
		 //{-3.6f,19.8f,2.0f,21.0f,124.0f}
		 //7#着陆台
		 {4.5f,35.7f, 2.0f,14.0f,50.0f}}

};
//左枪上弹姿态
gun_pose_t gLeftGunReloadPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{

		//第一类参数，对应打球参数
		{//1#着陆台
		 {-28.5f, 0.0f, 14.1f, 11.0f, 117.0f},
		 //2#着陆台
		 {-16.4f, 9.7f, 2.0f, 25.0f, 134.0f},
		 //3#着陆台
		 {1.5f, 22.8f, -0.9f, 6.0f, 110.0f},
		 //4#着陆台
		 {29.4f, 17.5f, 12.8f, 0.0f,113.0f},
		 //5#着陆台
		 {38.6f,13.0f, 8.4f,0.0f,112.0f},
		 //6#着陆台
		 {1.8f,9.0f,0.5f,22.0f,126.0f},
		 //7#着陆台
		 {5.5f, 25.4f, 7.9f,17.0f,81.0f}},

		//第二类参数，对应新盘落盘参数
		{//1#着陆台
		  {-34.6f, 20.2f, 16.0f, 20.0f, 93.0f},
		 //2#着陆台
//		 {-13.6f, 25.3f, 17.3f, 20.0f, 95.0f},
		 {-16.8f, 26.7f, 13.5f, 18.0f, 97.0f},
		 //3#着陆台
		 {2.7f, 26.9f, 6.6f, 20.0f, 96.0f},
		 //4#着陆台
		 {16.9f, 23.2f, 2.0f, 18.0f,98.0f},
		 //5#着陆台
		 {32.2f,16.6f, 4.4f,19.0f,100.0f},
		 //6#着陆台
		 {-3.1f,20.3f,2.0f,11.0f,129.0f},
		 //{-3.6f,19.8f,2.0f,21.0f,124.0f}
		 //7#着陆台
		 {4.5f,35.7f, 2.0f,14.0f,50.0f}},
		
		//第三类参数，对应打球同时落盘参数
		{//1#着陆台
		 {-28.0f, 7.0f, 14.1f, 10.0f, 113.0f},
		 //2#着陆台
		 {-16.4f, 15.7f, 7.3f, 5.0f, 106.0f},
		 //3#着陆台
		 {2.2f, 25.3f, 2.0f, 10.0f, 110.0f},
		 //4#着陆台
		 {29.4f, 17.5f, 12.8f, 0.0f,113.0f},
		 //5#着陆台
		 {38.6f,13.0f, 8.4f,0.0f,112.0f},
		 //6#着陆台
		 {0.2f,8.1f, 0.5f,27.0f,128.0f},
		 //7#着陆台
		 {6.0f, 25.9f,2.0f,16.0f,77.0f}},
		//第四类参数，对应旧盘落盘参数
		{//1#着陆台
		  {-33.1f, 19.2f, 16.0f, 22.0f, 94.0f},
		 //2#着陆台
//		 {-13.6f, 25.3f, 17.3f, 20.0f, 95.0f},
		 {-25.2f, 25.3f, 2.0f, 18.0f, 94.0f},
		 //3#着陆台
		 {-2.0f, 30.4f, 2.1f, 20.0f, 99.0f},
		 //4#着陆台
		 {16.9f, 23.2f, 2.0f, 18.0f,98.0f},
		 //5#着陆台
		 {32.2f,16.6f, 4.4f,19.0f,100.0f},
		 //6#着陆台
		 {-5.1f,19.0f,2.0f,15.0f,127.0f},
		 //7#着陆台
		 {4.5f,35.7f, 2.0f,14.0f,50.0f}},
		//第五类参数，对应新盘补弹打球参数
		{//1#着陆台
		 {-28.0f, 7.7f, 14.1f, 11.0f, 117.0f},
		 //2#着陆台
		 {-11.4f, 11.7f, 14.5f, 22.0f, 107.0f},
		 //3#着陆台
		 {2.2f, 25.3f, 2.0f, 10.0f, 110.0f},
		 //4#着陆台
		 {29.4f, 17.5f, 12.8f, 0.0f,113.0f},
		 //5#着陆台
		 {38.6f,13.0f, 8.4f,0.0f,112.0f},
		 //6#着陆台
		 {0.2f,8.1f, 0.5f,27.0f,128.0f},
		 //7#着陆台
		 {6.0f, 25.9f,2.0f,16.0f,77.0f}},
		//第六类参数，对应新盘补弹落盘参数
		{//1#着陆台
		  {-35.1f, 20.2f, 16.0f, 21.0f, 94.0f},
		 //2#着陆台
//		 {-13.6f, 25.3f, 17.3f, 20.0f, 95.0f},
		 {-22.2f, 24.2f, 6.0f, 18.0f, 94.0f},
		 //3#着陆台
		 {2.7f, 26.9f, 6.6f, 20.0f, 96.0f},
		 //4#着陆台
		 {16.9f, 23.2f, 2.0f, 18.0f,98.0f},
		 //5#着陆台
		 {32.7f,16.6f, 4.4f,18.0f,98.0f},
		 //6#着陆台
		 {-3.1f,20.3f,2.0f,11.0f,129.0f},
		 //{-3.6f,19.8f,2.0f,21.0f,124.0f}
		 //7#着陆台
		 {4.5f,35.7f, 2.0f,14.0f,50.0f}}
};


//右侧枪姿态数据库
gun_pose_t gRightGunPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{

		//第一类参数，对应打球参数
		{//1#着陆台
		 {-39.1f, 8.8f, 15.4f, 116.0f, 0.0f},
		 //2#着陆台
		 {-28.3f, 15.8f, 15.5f, 119.0f, 0.0f},
		 //3#着陆台
		 {-5.5f, 23.5f, 12.7f, 111.0f, 5.0f},
		 //4#着陆台
		 {15.3f,10.8f,14.5f,127.0f,22.0f},
		 //5#着陆台
		 {30.3f,6.8f, 17.0f,115.0f,10.0f},
		 //6#着陆台
//		 {1.9f,8.0f, -1.4f,130.0f,23.0f},
		 {4.9f,9.0f,0.6f,123.0f,19.0f},
		 //7#着陆台
		 {-5.0f,24.9f, 7.0f,11.0f,76.0f}},

		
		//第二类参数，对应新盘落盘参数
		{//1#着陆台
		 {-38.5f, 18.5f, 15.5f, 103.0f, 11.0f},
		 //2#着陆台
		 {-22.0f, 22.4f, 15.5f, 91.0f, 18.0f},
		 //3#着陆台
		 {2.8f, 24.4f, 6.1f, 92.0f, 22.0f},
		 //4#着陆台
		 {18.4f,23.7f,16.0f,89.0f,20.0f},
		 //5#着陆台
		 {38.7f,20.5f,14.0f,96.0f,21.0f},
		 //6#着陆台
		 {4.0f,21.4f,6.5f,131.0f,16.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}},
		
		 //第三类参数，对应打球同时落盘参数
		{//1#着陆台
		 {-36.1f, 11.8f, 7.9f, 113.0f, 2.0f},
		 //2#着陆台
		 {-23.3f, 14.6f, 6.0f, 119.0f, 0.0f},
		 //3#着陆台
		 {-5.5f, 23.5f, 12.7f, 111.0f, 5.0f},
		 //4#着陆台
		 {13.8f,17.9f,15.0f,110.0f,0.0f},
		 //5#着陆台
		 {29.1f,8.3f, 17.0f,113.0f,9.0f},
		 //6#着陆台
		 {1.9f,9.5f,2.1f,118.0f,25.0f},
//		 {1.4f,10.0f,2.1f,118.0f,25.0f},
		 //7#着陆台
		 {-5.0f,25.4f, 7.5f,11.0f,77.0f}},
		//第四类参数，对应旧盘落盘参数
		{//1#着陆台
		 {-38.5f, 18.5f, 15.5f, 103.0f, 11.0f},
		 //2#着陆台
		 {-22.0f, 22.4f, 15.5f, 91.0f, 18.0f},
		 //3#着陆台
		 {2.8f, 24.4f, 6.1f, 92.0f, 22.0f},
		 //4#着陆台
		 {19.4f,23.2f,16.0f,91.0f,20.0f},
		 //5#着陆台
		 {34.4f,19.5f,17.0f,94.0f,16.0f},
		 //6#着陆台
		 {3.0f,21.4f,6.5f,129.0f,13.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}},
		//第五类参数，对应新盘补弹打球参数
		{//1#着陆台
		 {-36.1f, 11.8f, 7.9f, 113.0f, 2.0f},
		 //2#着陆台
		 {-23.3f, 14.6f, 6.0f, 119.0f, 0.0f},
		 //3#着陆台
		 {-5.5f, 23.5f, 12.7f, 111.0f, 5.0f},
		 //4#着陆台
		 {14.8f,10.5f,14.5f,126.0f,22.0f},
		 //5#着陆台
		 {29.1f,8.8f, 17.0f,114.0f,10.0f},
		 //6#着陆台
//		 {1.9f,8.0f, -1.4f,130.0f,23.0f},
		 {1.9f,9.5f,2.1f,118.0f,25.0f},
		 //7#着陆台
		 {-5.0f,25.4f, 7.5f,11.0f,77.0f}},
		//第六类参数，对应新盘补弹落盘参数
		{//1#着陆台
		 {-38.5f, 18.5f, 15.5f, 103.0f, 11.0f},
		 //2#着陆台
		 {-22.0f, 22.4f, 15.5f, 91.0f, 18.0f},
		 //3#着陆台
		 {2.8f, 24.4f, 6.1f, 92.0f, 22.0f},
		 //4#着陆台
		 {18.4f,23.7f,16.0f,89.0f,20.0f},
		 //5#着陆台
		 {37.9f,21.0f,14.0f,96.0f,19.0f},
		 //6#着陆台
		 {3.5f,21.4f,6.5f,130.0f,16.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}}

};

//右侧枪上弹姿态数据库
gun_pose_t gRightGunReloadPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{

		//第一类参数，对应打球参数
		{//1#着陆台
		 {-39.1f, 8.8f, 15.4f, 116.0f, 0.0f},
		 //2#着陆台
		 {-28.3f, 15.8f, 15.5f, 119.0f, 0.0f},
		 //3#着陆台
		 {-5.5f, 23.5f, 12.7f, 111.0f, 5.0f},
		 //4#着陆台
		 {15.3f,10.8f,14.5f,127.0f,22.0f},
		 //5#着陆台
		 {30.3f,6.8f, 17.0f,115.0f,10.0f},
		 //6#着陆台
//		 {1.9f,8.0f, -1.4f,130.0f,23.0f},
		 {4.9f,9.0f,0.6f,123.0f,19.0f},
		 //7#着陆台
		 {-5.0f,24.9f, 7.0f,11.0f,76.0f}},

		
		//第二类参数，对应新盘落盘参数
		{//1#着陆台
		 {-38.5f, 18.5f, 15.5f, 103.0f, 11.0f},
		 //2#着陆台
		 {-22.0f, 22.4f, 15.5f, 91.0f, 18.0f},
		 //3#着陆台
		 {2.8f, 24.4f, 6.1f, 92.0f, 22.0f},
		 //4#着陆台
		 {18.4f,23.7f,16.0f,89.0f,20.0f},
		 //5#着陆台
		 {38.7f,20.5f,14.0f,96.0f,21.0f},
		 //6#着陆台
		 {4.0f,21.4f,6.5f,131.0f,16.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}},
		
		 //第三类参数，对应打球同时落盘参数
		{//1#着陆台
		 {-36.1f, 11.8f, 7.9f, 113.0f, 2.0f},
		 //2#着陆台
		 {-23.3f, 14.6f, 6.0f, 119.0f, 0.0f},
		 //3#着陆台
		 {-5.5f, 23.5f, 12.7f, 111.0f, 5.0f},
		 //4#着陆台
		 {13.8f,17.9f,15.0f,110.0f,0.0f},
		 //5#着陆台
		 {29.1f,8.3f, 17.0f,113.0f,9.0f},
		 //6#着陆台
		 {1.9f,9.5f,2.1f,118.0f,25.0f},
//		 {1.4f,10.0f,2.1f,118.0f,25.0f},
		 //7#着陆台
		 {-5.0f,25.4f, 7.5f,11.0f,77.0f}},
		//第四类参数，对应旧盘落盘参数
		{//1#着陆台
		 {-38.5f, 18.5f, 15.5f, 103.0f, 11.0f},
		 //2#着陆台
		 {-22.0f, 22.4f, 15.5f, 91.0f, 18.0f},
		 //3#着陆台
		 {2.8f, 24.4f, 6.1f, 92.0f, 22.0f},
		 //4#着陆台
		 {19.4f,23.2f,16.0f,91.0f,20.0f},
		 //5#着陆台
		 {34.4f,19.5f,17.0f,94.0f,16.0f},
		 //6#着陆台
		 {3.0f,21.4f,6.5f,129.0f,13.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}},
		//第五类参数，对应新盘补弹打球参数
		{//1#着陆台
		 {-36.1f, 11.8f, 7.9f, 113.0f, 2.0f},
		 //2#着陆台
		 {-23.3f, 14.6f, 6.0f, 119.0f, 0.0f},
		 //3#着陆台
		 {-5.5f, 23.5f, 12.7f, 111.0f, 5.0f},
		 //4#着陆台
		 {14.8f,10.5f,14.5f,126.0f,22.0f},
		 //5#着陆台
		 {29.1f,8.8f, 17.0f,114.0f,10.0f},
		 //6#着陆台
//		 {1.9f,8.0f, -1.4f,130.0f,23.0f},
		 {1.9f,9.5f,2.1f,118.0f,25.0f},
		 //7#着陆台
		 {-5.0f,25.4f, 7.5f,11.0f,77.0f}},
		//第六类参数，对应新盘补弹落盘参数
		{//1#着陆台
		 {-38.5f, 18.5f, 15.5f, 103.0f, 11.0f},
		 //2#着陆台
		 {-22.0f, 22.4f, 15.5f, 91.0f, 18.0f},
		 //3#着陆台
		 {2.8f, 24.4f, 6.1f, 92.0f, 22.0f},
		 //4#着陆台
		 {18.4f,23.7f,16.0f,89.0f,20.0f},
		 //5#着陆台
		 {37.9f,21.0f,14.0f,96.0f,19.0f},
		 //6#着陆台
		 {3.5f,21.4f,6.5f,130.0f,16.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}}

};


//上面枪姿态数据库
gun_pose_t gUpperGunPosDatabase[LAND_NUMBER][UPPER_SHOOT_METHOD_NUMBER][ZONE_NUMBER] = \
{
	
	//一号柱子对应参数
	{
		//第一种参数类型对应打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
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
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第四种参数类型对应补弹打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第五种参数类型对应补弹落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
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
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第四种参数类型对应补弹打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第五种参数类型对应补弹落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
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
			{0.4f,7.7f,0.0f,132.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{10.0f,23.6f,0.0f,97.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第四种参数类型对应补弹打球
		{
			{0.4f,7.7f,0.0f,132.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第五种参数类型对应补弹落盘
		{
			{10.0f,22.6f,0.0f,96.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
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
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第四种参数类型对应补弹打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第五种参数类型对应补弹落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
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
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第四种参数类型对应补弹打球
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第五种参数类型对应补弹落盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
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
			{11.2f,16.0f,0.0f,125.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{11.2f,16.0f,0.0f,125.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第四种参数类型对应补弹打球
		{
			{11.2f,16.0f,0.0f,125.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第五种参数类型对应补弹落盘
		{
			{11.2f,16.0f,0.0f,125.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
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
			{0.8f, 1.3f, 0.0f, 114.0f, 0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.4f, -2.3f, 0.0f, 121.0f, 0.0f},
			{-1.4f, -3.0f, 0.0f, 121.0f, 0.0f},
			{4.6f, -2.4f, 0.0f, 121.0f, 0.0f}
		},
		//第二种参数类型对应落盘
		{
			{8.5f,24.7f,0.0f,46.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{-5.4f, -3.3f, 0.0f, 131.0f, 0.0f},
			{-4.4f, -4.3f, 0.0f, 131.0f, 0.0f},
			{0.4f, -3.4f, 0.0f, 131.0f, 0.0f},			
			{-1.4f, -4.0f, 0.0f, 131.0f, 0.0f},
			{4.6f, -3.5f, 0.0f, 131.0f, 0.0f},
			{4.0f, -4.3f, 0.0f, 131.0f, 0.0f}
		},
		//第四种参数类型对应补弹打球
		{
			{0.8f, 1.3f, 0.0f, 114.0f, 0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.4f, -2.3f, 0.0f, 121.0f, 0.0f},
			{-1.4f, -3.0f, 0.0f, 121.0f, 0.0f},
			{4.6f, -2.4f, 0.0f, 121.0f, 0.0f}
		},
		//第五种参数类型对应补弹落盘
		{
			{8.5f,24.7f,0.0f,46.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
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
uint8_t LeftGunPriority[7] = {PLANT2,PLANT1,PLANT6,PLANT3,PLANT7,PLANT4,PLANT5};

/** @defgroup LeftGunPriority
  * @brief
  * @{
  */
uint8_t RightGunPriority[7] = {PLANT4,PLANT5,PLANT6,PLANT3,PLANT7,PLANT2,PLANT1};


uint8_t UpperGunPriority[7] = {PLANT7,PLANT3,PLANT6/*,PLANT3,PLANT7,PLANT2,PLANT1*/};
/** @defgroup Gun_Shoot_Command
  * @brief
  * @{
  */


//左枪射击柱子的顺序
shoot_command_t gLeftGunShootCmds[LEFT_GUN_AUTO_SHOOT_STEP_NUMBER] = \
{
	{SHOOT_POINT3, PLANT2,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT1,	SHOOT_METHOD2 , 2},	
	{SHOOT_POINT3, PLANT2,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT2,	SHOOT_METHOD2 , 2},
//	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD1 , 1},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1 , 2},
	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3}

};

//右枪射击柱子的顺序
shoot_command_t gRightGunShootCmds[RIGHT_GUN_AUTO_SHOOT_STEP_NUMBER] = \
{

//	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD1 , 2},
//	{SHOOT_POINT1, PLANT1,	SHOOT_METHOD2 , 2},	
	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT5,	SHOOT_METHOD2 , 2},
	{SHOOT_POINT3, PLANT4,	SHOOT_METHOD1 , 2},	
    {SHOOT_POINT3, PLANT4,	SHOOT_METHOD2 , 2},	
//	{SHOOT_POINT3, PLANT3,	SHOOT_METHOD1 , 2},
//	{SHOOT_POINT3, PLANT3,	SHOOT_METHOD2 , 2}
//	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD1 , 1},
//	{SHOOT_POINT3, PLANT7,	SHOOT_METHOD2 , 1},
//	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD1 , 2},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3},	
	{SHOOT_POINT3, PLANT6,	SHOOT_METHOD2 , 3}


};

////上面枪射击柱子的顺序
shoot_command_t gUpperGunShootCmds[UPPER_GUN_AUTO_STEP_NUMBER] = \
{
	{SHOOT_POINT3,PLANT7,SHOOT_METHOD1,1},	
	{SHOOT_POINT3,PLANT3,SHOOT_METHOD1,1},
	{SHOOT_POINT3,PLANT3,SHOOT_METHOD2,1},		
	{SHOOT_POINT3,PLANT7,SHOOT_METHOD2,1}
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
/** @defgroup Walk_Track_Database
  * @brief    此结构体数组的大小为 WALKTRACKDATABASE_POINT_NUMBER * 3 * 4 Byte
  * @{
  */
#ifdef COLLECTING_DOT_ENABLE
recordWalkTrackInfo_t gWalkTrackDatabase[WALKTRACKDATABASE_POINT_CAPACITY]={0};
#else
//const recordWalkTrackInfo_t *gWalkTrackDatabase = (recordWalkTrackInfo_t *)(0x08060004);
#endif
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern robot_t gRobot;

///**
//  * @brief  Update LeftGunPosDatabase in Manual Mode
//  * @note   this function will update the 
//  * @param  None
//  * @retval None
//  */
//void UpdateLeftGunPosDatabaseManualMode(void)
//{
//	if(gRobot.moveBase.targetPoint == 2)
//	{
//		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].yaw = gRobot.leftGun.targetPose.yaw;
//		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].pitch = gRobot.leftGun.targetPose.pitch;
//		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].roll = gRobot.leftGun.targetPose.roll;
//		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed1 = gRobot.leftGun.targetPose.speed1;
//		gLeftGunPosDatabase[0][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed2 = gRobot.leftGun.targetPose.speed2;
//	
//	}
//	else if(gRobot.moveBase.targetPoint == 3)
//	{
//		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].yaw = gRobot.leftGun.targetPose.yaw;
//		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].pitch = gRobot.leftGun.targetPose.pitch;
//		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].roll = gRobot.leftGun.targetPose.roll;
//		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed1 = gRobot.leftGun.targetPose.speed1;
//		gLeftGunPosDatabase[1][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed2 = gRobot.leftGun.targetPose.speed2;
//	}
//	else if(gRobot.moveBase.targetPoint == 1)
//	{
//		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].yaw = gRobot.leftGun.targetPose.yaw;
//		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].pitch = gRobot.leftGun.targetPose.pitch;
//		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].roll = gRobot.leftGun.targetPose.roll;
//		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed1 = gRobot.leftGun.targetPose.speed1;
//		gLeftGunPosDatabase[2][gRobot.leftGun.shootParaMode][gRobot.leftGun.targetPlant].speed2 = gRobot.leftGun.targetPose.speed2;
//	}
//}

///**
//  * @brief  Update RightGunPosDatabase in Manual Mode
//  * @note   this function will update the 
//  * @param  None
//  * @retval None
//  */
//void UpdateRightGunPosDatabaseManualMode(void)
//{
//	if(gRobot.moveBase.targetPoint == 2)
//	{
//		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].yaw = gRobot.rightGun.targetPose.yaw;
//		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].pitch = gRobot.rightGun.targetPose.pitch;
//		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].roll = gRobot.rightGun.targetPose.roll;
//		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed1 = gRobot.rightGun.targetPose.speed1;
//		gRightGunPosDatabase[0][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed2 = gRobot.rightGun.targetPose.speed2;
//	
//	}
//	else if(gRobot.moveBase.targetPoint == 3)
//	{
//		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].yaw = gRobot.rightGun.targetPose.yaw;
//		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].pitch = gRobot.rightGun.targetPose.pitch;
//		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].roll = gRobot.rightGun.targetPose.roll;
//		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed1 = gRobot.rightGun.targetPose.speed1;
//		gRightGunPosDatabase[1][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed2 = gRobot.rightGun.targetPose.speed2;
//	}
//	else if(gRobot.moveBase.targetPoint == 1)
//	{
//		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].yaw = gRobot.rightGun.targetPose.yaw;
//		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].pitch = gRobot.rightGun.targetPose.pitch;
//		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].roll = gRobot.rightGun.targetPose.roll;
//		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed1 = gRobot.rightGun.targetPose.speed1;
//		gRightGunPosDatabase[2][gRobot.rightGun.shootParaMode][gRobot.rightGun.targetPlant].speed2 = gRobot.rightGun.targetPose.speed2;
//	}
//}
/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

