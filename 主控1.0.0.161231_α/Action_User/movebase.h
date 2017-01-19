/**
  ******************************************************************************
  * @file      movebase.h
  * @author    沈阳艾克申机器人有限公司
  * @version   V1.0.1
  * @date      2017/01/16
  * @brief     本文件包含该四轮全向移动底盘走行
               主要参数及主要功能函数声明
  ******************************************************************************
  * @attention
  *
  *
  *
  *
  ******************************************************************************
  */
#ifndef __MOVEBASE_H
#define __MOVEBASE_H

/*
 *关于底盘相关参数的宏定义
 */
//车轮半径 单位：m
#define WHEELRADIUS    0.076f
//减速比
#define REDUCTION     (299.0f/14.0f)


//电机最大速度 单位：脉冲/s
#define MAXVEL 250000.0f
//车速 m/s
//垂直方向速度
#define VELVERTICAL 0.0f
//水平方向速度
#define VELHORIZONTAL 1.0f

//电机加速能力  单位：脉冲/s^2
//设定电机最大加速度
#define MAXACC     50000.0f



/*
 * 关于操作指令反馈的宏定义
 */

//函数正常运行
#define RETURNOK 		 1
//无效变量
#define IVLDPARM   		 0
//有效变量
#define VALIDPARM 		 1
//变量超出范围
#define OUTOFRANGE 	    -2

/*
 *闭环用到的宏定义
 */

//角度闭环的PID参数
#define KPANG       1.5f
#define KIANG       0.0f
#define KDANG       0.0f
//位置闭环的PID参数
#define KPPOS       0.8f
#define KIPOS       0.0f
#define KDPOS       0.0f
//限制角度闭环的PID输出最大值
#define MAXANGPIDOUT   1080.0f
//限制位置闭环的PID输出最大值
#define MAXPOSPIDOUT   2.8f

/*
 *其他宏定义
 */
#define PI            3.141592653579f


/*
 *单位转换
 */

//弧度制和角度制相互转换
#define ANGTORAD(x) (float)((x)/180.0f*3.141592653579f)
#define RADTOANG(x) (float)((x)/3.141592653579f*180.0f)

/*
 *相关结构体
 */
//关于加速度的结构体
typedef struct
{
	float wheel1;
	float wheel2;
	float wheel3;
	float wheel4;
}motorAcc_t;

//关于三个轮速度的结构体类型
//单位：脉冲/s
typedef struct
{
	float v1;
	float v2;
	float v3;
	float v4;
}wheelSpeed_t;

/*
 *电机加速度配置相关函数
 */

/**
* @brief  计算电机加速度
* @param  carAcc : 机器人的合加速度
* @param  angle : 机器人平移方向角
* @retval mototAcc ：四轮电机加速度
* @author ACTION
*/
motorAcc_t CalcMotorAcc(float carAcc,float angle);

/**
* @brief  配置电机加速度
* @param  motorAcc ： 四轮电机加速度结构体
* @retval 无
* @author ACTION
*/
void SetMotorAcc(motorAcc_t motorAcc);
/**
* @brief  配置电机速度
* @param  speed ： 四轮电机速度结构体
* @retval 无
* @author ACTION
*/
void FourWheelVelControl(wheelSpeed_t speed);

/**
* @brief   电机制动抱死
* @param  无
* @retval 无
* @author ACTION
*/
void StopMove(void);

/**
* @brief   脉冲速度转化为标准单位速度
* @param  velPulse ： 速度 脉冲/秒
* @retval velStandard ： 速度 米/秒
* @author ACTION
*/
float VelPulse2Standard(float velPulse);

/**
* @brief   标准单位速度转化为脉冲速度
* @param  velStandard ： 速度 米/秒
* @retval velPulse ： 速度 脉冲/秒
* @author ACTION
*/
float VelStandard2Pulse(float velStandard);

//运动函数
int Move(float velX, float velY);

//减速x方向速度函数
float XSpeedDown(float posX, float dstX, float speedBegin);

//加速x方向速度函数
float XSpeedUp(float posX, float dstX, float speedEnd);

#endif

