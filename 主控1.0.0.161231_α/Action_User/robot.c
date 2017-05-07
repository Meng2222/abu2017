#include <math.h>
#include "robot.h"
#include "timer.h"
#include "ucos_ii.h"
#include "gpio.h"
#include "cpu.h"
#include "usart.h"
robot_t gRobot = {0};

/*
============================================================
				   枪参数变换与逆变换			
============================================================
*/



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




