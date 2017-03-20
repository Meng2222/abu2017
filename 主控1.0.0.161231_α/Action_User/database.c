#include "database.h"

//左侧枪姿态数据库
//{yaw, pitch, row, speed1, speed2}
gun_pose_t gLeftGunPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{
	//第1种子弹状态，对应的7个着陆台的枪的姿态
	{
	 //1#着陆台  4500  一号打球
	 {-12.5f, 11.0f, 18.6f, 98.0f, 12.0f},
	 //2#着陆台  4500   二号打球
	 {5.0f, 15.5f, 26.4f, 93.0f, 20.0f},
	 //3#着陆台  7500   假装是三号的一号落盘
	 {-14.0f, 25.0f, 21.1f, 92.0f, 12.0f},
	 //4#着陆台  7500   同上，二号
	 {5.0f, 30.2f, 23.4f, 99.0f,18.0f},
	 //5#着陆台  7500
	 {45.8f,32.5f, 40.6f,81.0f,24.0f},
	 //6#着陆台  7500
	 {0.5f,26.9f, 3.7f,93.0f,15.0f},
	 //7#着陆台
	 {26.2f,34.0f, 20.4f,83.0f,24.0f}},
	
	//第2位置，对应的7个着陆台的枪的姿态
	{{0.0f, 1.0f, 2.0f, 3.0f, 4.0f},
		{5.0f, 6.0f, 7.0f, 8.0f, 9.0f},
		{0.1f, 0.2f, 0.3f, 0.4f, 0.5f},
		{0.6f, 0.7f, 0.8f, 0.9f, 1.0f},
		{0.1f, 1.1f, 2.1f, 3.1f, 4.1f},
		{5.1f, 6.1f, 7.1f, 8.1f, 9.1f},
		{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
	

};

//右侧枪姿态数据库
gun_pose_t gRightGunPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER] = \
{
	//第1种子弹状态，对应的7个着陆台的枪的姿态
	{	 //1#着陆台
	 {45.0f,16.0f, 40.2f,89.0f,21.0f},
	 //2#着陆台
	 {0.0f,33.6f, 2.5f,84.0f,21.0f},
	 //3#着陆台
	 {45.5f,17.7f, 40.8f,104.0f,19.0f},
	 //4#着陆台
	 {0.5f,32.5f, 2.1f,84.0f,20.0f},
	 //5#着陆台
	 {45.8f,32.5f, 40.6f,81.0f,24.0f},
	 //6#着陆台
	 {0.5f,26.9f, 3.7f,93.0f,15.0f},
	 //7#着陆台
	 {26.2f,34.0f, 20.4f,83.0f,24.0f}},
	
	//第2种子弹状态，对应的7个着陆台的枪的姿态
	{{0.0f, 1.0f, 2.0f, 3.0f, 4.0f},{5.0f, 6.0f, 7.0f, 8.0f, 9.0f},{0.1f, 0.2f, 0.3f, 0.4f, 0.5f},{0.6f, 0.7f, 0.8f, 0.9f, 1.0f},
	{0.1f, 1.1f, 2.1f, 3.1f, 4.1f},{5.1f, 6.1f, 7.1f, 8.1f, 9.1f},{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
	

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

//左枪射击柱子的顺序
shoot_command_t gLeftGunShootCmds = \
{
	{
		{PLANT1,	SHOOT_METHOD1},
		{PLANT2,	SHOOT_METHOD1},
		{PLANT3,	SHOOT_METHOD1},
		{PLANT4,	SHOOT_METHOD1},
		{PLANT5,	SHOOT_METHOD1},
		{PLANT6,	SHOOT_METHOD1},
		{PLANT7,	SHOOT_METHOD1},
		{PLANT1,	SHOOT_METHOD1},
		{PLANT2,	SHOOT_METHOD1},
		{PLANT3,	SHOOT_METHOD1}
	}
};

//右枪射击柱子的顺序
shoot_command_t gRightGunShootCmds = \
{
	{
		{PLANT1,	SHOOT_METHOD1},
		{PLANT2,	SHOOT_METHOD1},
		{PLANT3,	SHOOT_METHOD1},
		{PLANT4,	SHOOT_METHOD1},
		{PLANT5,	SHOOT_METHOD1},
		{PLANT6,	SHOOT_METHOD1},
		{PLANT7,	SHOOT_METHOD1},
		{PLANT1,	SHOOT_METHOD1},
		{PLANT2,	SHOOT_METHOD1},
		{PLANT3,	SHOOT_METHOD1}
	}
};

//上面枪射击柱子的顺序
shoot_command_t gUpperGunShootCmds = \
{
	{
		{PLANT1,	SHOOT_METHOD1},
		{PLANT2,	SHOOT_METHOD1},
		{PLANT3,	SHOOT_METHOD1},
		{PLANT4,	SHOOT_METHOD1},
		{PLANT5,	SHOOT_METHOD1},
		{PLANT6,	SHOOT_METHOD1},
		{PLANT7,	SHOOT_METHOD1},
		{PLANT1,	SHOOT_METHOD1},
		{PLANT2,	SHOOT_METHOD1},
		{PLANT3,	SHOOT_METHOD1}
	}
};

