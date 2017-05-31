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
#include "robot.h"
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
/******************************红场参数*****************************/
#ifdef RED_FIELD
		//第一类参数，对应打球同时落盘参数
		{//1#着陆台
		 {-26.2f, 10.4f, 16.4f, 0.0f, 118.0f},
		 //2#着陆台
		 {-10.9f, 16.7f, 16.9f, 3.0f, 111.0f},
		 //3#着陆台
		 {2.5f, 22.8f, -0.6f, 5.0f, 110.0f},
		 //4#着陆台
		 {29.4f, 15.0f, 12.8f, 0.0f,116.0f},
		 //5#着陆台
		 {41.5f,7.0f, 7.9f,10.0f,110.0f},
		 //6#着陆台
		 {-1.8f,9.0f, -4.0f,21.0f,131.0f},
		 //7#着陆台
		 {6.0f, 25.4f,6.0f,16.0f,77.0f}},
		//第二类参数，对应新盘自动落盘参数
		{//1#着陆台
		  {-31.2f, 14.2f, 18.7f, 17.0f, 86.0f},
		 //2#着陆台
		 {-16.5f, 21.9f, 11.1f, 14.0f, 91.0f},
		 //3#着陆台
		 {0.7f, 26.4f, 6.6f, 19.0f, 92.0f},
		 //4#着陆台
		 {21.9f, 21.2f, 8.0f, 14.0f,93.0f},
		 //5#着陆台
		 {33.5f,13.6f, 2.8f,10.0f,100.0f},
		 //6#着陆台
		 {-4.5f,18.4f,-0.3f,7.0f,130.0f},
		 //7#着陆台
		 {4.5f,36.2f, 2.0f,14.0f,50.0f}},
		
		//第三类参数，对应新盘补弹打球参数
		{//1#着陆台
		 {-26.2f, 10.4f, 16.4f, 0.0f, 118.0f},
		 //2#着陆台
		 {-10.9f, 16.7f, 16.9f, 3.0f, 111.0f},
		 //3#着陆台
		 {2.5f, 22.8f, -0.6f, 5.0f, 110.0f},
		 //4#着陆台
		 {29.4f, 15.0f, 12.8f, 0.0f,116.0f},
		 //5#着陆台
		 {41.5f,7.0f, 7.9f,10.0f,110.0f},
		 //6#着陆台
		 {-1.8f,9.0f, -4.0f,21.0f,131.0f},
		 //7#着陆台
		 {6.0f, 25.4f,6.0f,16.0f,77.0f}},
		//第四类参数，对应新盘补弹落盘参数
		{//1#着陆台
		  {-31.2f, 14.2f, 18.7f, 17.0f, 86.0f},
		 //2#着陆台
		 {-16.5f, 21.9f, 11.1f, 14.0f, 91.0f},
		 //3#着陆台
		 {0.7f, 26.4f, 6.6f, 19.0f, 92.0f},
		 //4#着陆台
		 {21.9f, 21.2f, 8.0f, 14.0f,93.0f},
		 //5#着陆台
		 {33.5f,13.6f, 2.8f,10.0f,100.0f},
		 //6#着陆台
		 {-4.5f,18.4f,-0.3f,7.0f,130.0f},
		 //7#着陆台
		 {4.5f,36.2f, 2.0f,14.0f,50.0f}}
#endif
/*******************************蓝场参数********************************/
#ifdef BLUE_FIELD
		//第一类参数，对应打球同时落盘参数
		{//1#着陆台
		 {-26.2f, 10.4f, 16.4f, 0.0f, 118.0f},
		 //2#着陆台
		 {-10.9f, 16.7f, 16.9f, 3.0f, 111.0f},
		 //3#着陆台
		 {2.5f, 22.8f, -0.6f, 5.0f, 110.0f},
		 //4#着陆台
		 {28.9f, 16.0f, 12.8f, 0.0f,117.0f},
		 //5#着陆台
		 {41.5f,7.0f, 7.9f,10.0f,110.0f},
		 //6#着陆台
		 {-1.8f,9.0f, -4.0f,21.0f,131.0f},
		 //7#着陆台
		 {6.0f, 25.4f,6.0f,16.0f,77.0f}},
		//第二类参数，对应新盘自动落盘参数
		{//1#着陆台
		  {-31.2f, 14.2f, 18.7f, 17.0f, 86.0f},
		 //2#着陆台
		 {-16.5f, 21.9f, 11.1f, 14.0f, 91.0f},
		 //3#着陆台
		 {1.2f, 26.4f, 6.6f, 19.0f, 91.0f},
		 //4#着陆台
		 {22.9f, 20.7f, 8.0f, 14.0f,93.0f},
		 //5#着陆台
		 {33.5f,14.1f, 2.8f,10.0f,100.0f},
		 //6#着陆台
		 {-5.0f,18.4f,-0.3f,7.0f,131.0f},
		 //7#着陆台
		 {4.5f,36.2f, 2.0f,14.0f,50.0f}},
		
		//第三类参数，对应新盘补弹打球参数
		{//1#着陆台
		 {-26.2f, 10.4f, 16.4f, 0.0f, 118.0f},
		 //2#着陆台
		 {-10.9f, 16.7f, 16.9f, 3.0f, 111.0f},
		 //3#着陆台
		 {2.5f, 22.8f, -0.6f, 5.0f, 110.0f},
		 //4#着陆台
		 {28.9f, 16.0f, 12.8f, 0.0f,117.0f},
		 //5#着陆台
		 {41.5f,7.0f, 7.9f,10.0f,110.0f},
		 //6#着陆台
		 {-1.8f,9.0f, -4.0f,21.0f,131.0f},
		 //7#着陆台
		 {6.0f, 25.4f,6.0f,16.0f,77.0f}},
		//第四类参数，对应新盘补弹落盘参数
		{//1#着陆台
		  {-31.2f, 14.2f, 18.7f, 17.0f, 86.0f},
		 //2#着陆台
		 {-16.5f, 21.9f, 11.1f, 14.0f, 91.0f},
		 //3#着陆台
		 {1.2f, 26.4f, 6.6f, 19.0f, 91.0f},
		 //4#着陆台
		 {22.9f, 20.7f, 8.0f, 14.0f,93.0f},
		 //5#着陆台
		 {33.5f,14.1f, 2.8f,10.0f,100.0f},
		 //6#着陆台
		 {-5.0f,18.4f,-0.3f,7.0f,131.0f},
		 //7#着陆台
		 {4.5f,36.2f, 2.0f,14.0f,50.0f}}		 
#endif		 
		
};
//左枪上弹姿态
gun_pose_t gLeftGunReloadPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] ={0};


//右侧枪姿态数据库
gun_pose_t gRightGunPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{

/************************红场参数************************/
#ifdef RED_FIELD
		 //第一类参数，对应打球同时落盘参数
		{//1#着陆台
		 {-38.1f, 8.0f, 6.1f, 110.0f, 8.0f},
		 //2#着陆台
		 {-27.8f, 14.0f, 15.5f, 115.0f, 7.0f},
		 //3#着陆台
		 {-1.0f, 22.8f, 0.9f, 110.0f, 7.0f},
		 //4#着陆台
		 {10.8f,18.4f,20.0f,104.0f,7.0f},
		 //5#着陆台
		 {29.1f,7.8f, 18.0f,114.0f,10.0f},
		 //6#着陆台
		 {4.0f,14.9f,4.6f,117.0f,14.0f},
		 //7#着陆台
		 {-5.0f,25.4f, 7.5f,11.0f,77.0f}},
		
		//第二类参数，对应新盘自动落盘参数
		{//1#着陆台
		 {-39.0f, 15.6f, 17.1f, 95.0f, 14.0f},
		 //2#着陆台
		 {-25.0f, 21.9f, 20.5f, 86.0f, 20.0f},
		 //3#着陆台
		 {1.8f, 26.7f, 6.1f, 92.0f, 20.0f},
		 //4#着陆台
		 {15.4f,20.7f,20.0f,85.0f,20.0f},
		 //5#着陆台
		 {32.9f,14.4f,23.0f,83.0f,21.0f},
		 //6#着陆台
		 {6.5f,18.4f,-0.3f,129.0f,7.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}},
		
		//第三类参数，对应新盘补弹打球参数
		{//1#着陆台
		 {-38.1f, 8.0f, 6.1f, 110.0f, 8.0f},
		 //2#着陆台
		 {-27.8f, 14.0f, 15.5f, 115.0f, 7.0f},
		 //3#着陆台
		 {-1.0f, 22.8f, 0.9f, 110.0f, 7.0f},
		 //4#着陆台
		 {10.8f,18.4f,20.0f,104.0f,7.0f},
		 //5#着陆台
		 {29.1f,7.8f, 18.0f,114.0f,10.0f},
		 //6#着陆台
		 {4.0f,14.9f,4.6f,117.0f,14.0f},
		 //7#着陆台
		 {-5.0f,25.4f, 7.5f,11.0f,77.0f}},
		//第四类参数，对应新盘补弹落盘参数
		{//1#着陆台
		 {-39.0f, 15.6f, 17.1f, 95.0f, 14.0f},
		 //2#着陆台
		 {-25.0f, 21.9f, 20.5f, 86.0f, 20.0f},
		 //3#着陆台
		 {1.8f, 26.7f, 6.1f, 92.0f, 20.0f},
		 //4#着陆台
		 {15.4f,20.7f,20.0f,85.0f,20.0f},
		 //5#着陆台
		 {32.9f,14.4f,23.0f,83.0f,21.0f},
		 //6#着陆台
		 {6.5f,18.4f,-0.3f,129.0f,7.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}}
#endif

/*****************************蓝场参数***************************/
#ifdef BLUE_FIELD
		 		 //第一类参数，对应打球同时落盘参数
		{//1#着陆台
		 {-38.1f, 8.0f, 6.1f, 110.0f, 8.0f},
		 //2#着陆台
		 {-27.8f, 14.0f, 15.5f, 115.0f, 7.0f},
		 //3#着陆台
		 {-1.0f, 22.8f, 0.9f, 110.0f, 7.0f},
		 //4#着陆台
		 {10.8f,17.9f,20.0f,104.0f,7.0f},
		 //5#着陆台
		 {29.1f,7.3f, 18.0f,114.0f,10.0f},
		 //6#着陆台
		 {4.0f,14.9f,4.6f,117.0f,14.0f},
		 //7#着陆台
		 {-5.0f,25.4f, 7.5f,11.0f,77.0f}},
		
		//第二类参数，对应新盘自动落盘参数
		{//1#着陆台
		 {-39.0f, 15.6f, 17.1f, 95.0f, 14.0f},
		 //2#着陆台
		 {-25.5f, 21.9f, 20.5f, 86.0f, 20.0f},
		 //3#着陆台
		 {1.8f, 26.7f, 6.1f, 92.0f, 20.0f},
		 //4#着陆台
		 {15.4f,21.2f,20.0f,85.0f,20.0f},
		 //5#着陆台
		 {32.4f,15.4f,23.0f,84.0f,21.0f},
		 //6#着陆台
		 {7.0f,18.4f,-0.3f,129.0f,7.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}},
		
		//第三类参数，对应新盘补弹打球参数
		{//1#着陆台
		 {-38.1f, 8.0f, 6.1f, 110.0f, 8.0f},
		 //2#着陆台
		 {-27.8f, 14.0f, 15.5f, 115.0f, 7.0f},
		 //3#着陆台
		 {-1.0f, 22.8f, 0.9f, 110.0f, 7.0f},
		 //4#着陆台
		 {10.8f,17.9f,20.0f,104.0f,7.0f},
		 //5#着陆台
		 {29.1f,7.3f, 18.0f,114.0f,10.0f},
		 //6#着陆台
		 {4.0f,14.9f,4.6f,117.0f,14.0f},
		 //7#着陆台
		 {-5.0f,25.4f, 7.5f,11.0f,77.0f}},
		//第四类参数，对应新盘补弹落盘参数
		{//1#着陆台
		 {-39.0f, 15.6f, 17.1f, 95.0f, 14.0f},
		 //2#着陆台
		 {-25.5f, 21.9f, 20.5f, 86.0f, 20.0f},
		 //3#着陆台
		 {1.8f, 26.7f, 6.1f, 92.0f, 20.0f},
		 //4#着陆台
		 {15.4f,21.2f,20.0f,85.0f,20.0f},
		 //5#着陆台
		 {32.4f,15.4f,23.0f,84.0f,21.0f},
		 //6#着陆台
		 {7.0f,18.4f,-0.3f,129.0f,7.0f},
		 //7#着陆台
		 {-1.0f,35.5f, 2.9f,50.0f,15.0f}}
#endif		 

};

//右侧枪上弹姿态数据库
gun_pose_t gRightGunReloadPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] ={0};


//上面枪姿态数据库
gun_pose_t gUpperGunPosDatabase[LAND_NUMBER][UPPER_SHOOT_METHOD_NUMBER][ZONE_NUMBER] = \
{
/**************************红场参数*******************************/
#ifdef RED_FIELD
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
		//第一种参数类型对应打球落盘
		{
			{2.3f,10.3f,0.0f,109.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{8.0f,20.4f,0.0f,99.0f,0.0f},
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
			{2.3f,10.3f,0.0f,109.0f,0.0f},
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
			{7.3f,12.0f,0.0f,125.0f,0.0f},
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
			{7.3f,12.0f,0.0f,125.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	},	
	//七号柱子对应参数
	{
		//第一种参数类型对应打球同时落盘
		{
			{3.0f, 1.4f, 0.0f, 70.0f, 0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.4f, -2.3f, 0.0f, 121.0f, 0.0f},
			{-1.4f, -3.0f, 0.0f, 121.0f, 0.0f},
			{4.6f, -2.4f, 0.0f, 121.0f, 0.0f}
		},
		//第二种参数类型对应落盘
		{
			{8.5f,24.7f,0.0f,49.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{-6.8f, -3.5f, 0.0f, 130.0f, 0.0f},
			{-6.6f, -3.8f, 0.0f, 130.0f, 0.0f},
			{-1.6f, -3.8f, 0.0f, 130.0f, 0.0f},			
			{1.2f, -4.1f, 0.0f, 130.0f, 0.0f},
			{2.8f, -3.7f, 0.0f, 130.0f, 0.0f},
			{3.0f, -4.3f, 0.0f, 130.0f, 0.0f}
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
			{8.5f,24.7f,0.0f,49.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	}
#endif
	
/**************************蓝场参数*******************************/
#ifdef BLUE_FIELD
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
		//第一种参数类型对应打球落盘
		{
			{2.3f,10.3f,0.0f,109.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第二种参数类型对应落盘
		{
			{8.0f,20.4f,0.0f,99.0f,0.0f},
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
			{2.3f,10.3f,0.0f,109.0f,0.0f},
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
			{7.3f,12.0f,0.0f,125.0f,0.0f},
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
			{7.3f,12.0f,0.0f,125.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	},	
	//七号柱子对应参数
	{
		//第一种参数类型对应打球同时落盘
		{
			{3.0f, 1.4f, 0.0f, 70.0f, 0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.4f, -2.3f, 0.0f, 121.0f, 0.0f},
			{-1.4f, -3.0f, 0.0f, 121.0f, 0.0f},
			{4.6f, -2.4f, 0.0f, 121.0f, 0.0f}
		},
		//第二种参数类型对应落盘
		{
			{8.5f,24.7f,0.0f,49.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		},
		//第三种参数类型对应打盘
		{
			{-6.8f, -3.5f, 0.0f, 130.0f, 0.0f},
			{-6.6f, -3.8f, 0.0f, 130.0f, 0.0f},
			{-1.6f, -3.8f, 0.0f, 130.0f, 0.0f},			
			{1.2f, -4.1f, 0.0f, 130.0f, 0.0f},
			{2.8f, -3.7f, 0.0f, 130.0f, 0.0f},
			{3.0f, -4.3f, 0.0f, 130.0f, 0.0f}
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
			{8.5f,24.7f,0.0f,49.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f},
			{0.0f,0.0f,0.0f,0.0f,0.0f}
		}
	}
#endif
};


/**
  * @}
  */

/** @defgroup LeftGunPriority
  * @brief
  * @{
  */
uint8_t LeftGunPriority[7] = {PLANT2,PLANT1,PLANT3,PLANT6,PLANT7,PLANT4,PLANT5};

/** @defgroup LeftGunPriority
  * @brief
  * @{
  */
uint8_t RightGunPriority[7] = {PLANT4,PLANT5,PLANT3,PLANT6,PLANT7,PLANT2,PLANT1};


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

