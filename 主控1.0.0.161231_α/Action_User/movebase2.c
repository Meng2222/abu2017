/**
  *******************************************************************************************************
  * @file	movebase2.c
  * @author  ACTION_2017
  * @version V0.0.0.170321_alpha
  * @date	2017/03/28
  * @brief   This file contains all the functions prototypes for
  *
  *******************************************************************************************************
  * @attention
  *
  *
  *******************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------*/
#include <math.h>

#include "movebase2.h"
#include "gpio.h"
#include "timer.h"
#include "database.h"
#include "flash.h"
#include "stm32f4xx_usart.h"
#include "dma.h"
#include "usart.h"

extern robot_t gRobot;
/* Private typedef -------------------------------------------------------------------------------------*/


/**
  * @brief
  */

//typedef struct
//{
//
//}_t;

/* Private define --------------------------------------------------------------------------------------*/

#define LAUNCH_AREA_DISP	13026.96f

#define VEL_UPPER_BOUND		3000.0f		//Unit: mm/s
#define ACC_UPPER_BOUND		2500.0f		//Unit: mm/s^2
#define DEC_LOWER_BOUND		2500.0f		//Unit: mm/s^2
//前瞻量
#define   LOOKAHEAD_LENTH	100.0f		//Unit: mm


/* Private macro ----------------------------------------------------------------------------------------*/
/* Private variables ------------------------------------------------------------------------------------*/
//static uint16_t closetPointIndex = 1000u;
//static float offsetAngle = 0.0f;
static uint16_t updateTime = 0u;
//static float xCorrection = 0.0f;
static float dispCorrection = 0.0f;
static uint16_t velTimeCnt = 0u;
/* Private function prototypes --------------------------------------------------------------------------*/
#ifdef COLLECTING_DOT_ENABLE
static void WarningRecordWalkTrackError(void);
#endif

/* Private functions ------------------------------------------------------------------------------------*/


void Sendfloat(float val)
{
	UART5_OUT((uint8_t*)" %d", (int)val);
    if((val - (int)val)*10 >= 1.0f)
    {
        UART5_OUT((uint8_t *)".%d", (val - (int)val)*10);
    }
	UART5_OUT((uint8_t*)"\t");
}

/** @defgroup Get_Kinematic_Information
  * @brief
  * @{
  */
/**
  * @brief	设定X坐标的矫正量
  * @param	theoreticalX 理论坐标
  * @retval	None
  */
//void SetXCorrection(float theoreticalX)
//{
//	xCorrection += theoreticalX - gRobot.moveBase.actualPosition.x;
//}
/**
  * @brief	设定位移量的矫正量 注意： 此矫正量并没有真正更改了gRobot 中的actualDisp
  * @param	theoreticalDisp 理论上相对于原点的位移
  * @retval	None
  */
void SetDispCorrection(float theoreticalDisp)
{
	dispCorrection += theoreticalDisp - gRobot.moveBase.actualKenimaticInfo.disp;
}
/**
  * @brief	获取位移的改正量
  * @note
  * @retval	dispCorrection
  */
float GetDispCorrection(void)
{
	return dispCorrection;
}


/**
  * @brief	为计算速度计数 1ms一次
  * @note	
 * @param	None
  * @retval	
  */
void velTimerCounting(void)
{
	velTimeCnt++;
}

/**
  * @brief  UpdateMoveBaseData
  * @note
  * @param  angle: data that will update the angle in gRobot.movebase.actualPos
  * @param  posX: data that will update the X pos in gRobot.movebase.actualPos
  * @param  posY: data that will update the Y pos in gRobot.movebase.actualPos
  * @retval None
  */
void UpdateXYAngle(float angle, float posX, float posY)
{
	/**
	Originally, the positive direction of Y-axis is pointed by the camera,
	and the X-axis and angle follow the right-hand rule.
	However, to make it convenient, the positive direction of X and Y axis
	is reversed from the original direction, and they remain follow the
	right-hand rule
	*/
	gRobot.moveBase.actualPosition.angle	= angle;
	gRobot.moveBase.actualPosition.x		= posX;
	gRobot.moveBase.actualPosition.y		= posY;
	/**
	this variable is count up whenever the data is sent by
	the positioning systerm, and will be cleared once
	CalcactualVel() is called
	*/
	updateTime = velTimeCnt;
}

/**
  * @brief  Calculate the absolute value of actual velocity and return it
  *			This Function is suppose to be called every 10 ms
  *			如果数据没有更新，则输出上一次计算的的速度
  *			如果调用此函数之间超过多个10ms 也能输出正确的速度，
  * @note 	fix me actualPosition 的更新是异步的,必须配合UpdateMOveBaseData()调用
  * @param	None
  * @retval a float that is the absolute value of velocity Unit: mm/s^2
  */
void UpdateKenimaticInfo(void)
{
	static float posX_last = 0.0f, posY_last = 0.0f;
	static uint16_t updateTimeLast = 0u;
	float displacementLenth = 0.0f;
	float vX = 0.0f, vY = 0.0f;
	static float vel = 0.0f;
	if(updateTime != updateTimeLast)
	{
		//Displacement Increment in (5* updateTime)ms Unit: mm
		//Warning: 严格来说，这不是X方向分速度与Y方向分速度
		vX = gRobot.moveBase.actualPosition.x - posX_last;
		vY = gRobot.moveBase.actualPosition.y - posY_last;
		//Unit: mm
		displacementLenth = sqrtf(vX * vX +vY * vY);
		vel = 1000.0f * displacementLenth / ((uint16_t)(updateTime - updateTimeLast));
		//暂时认为只有一维运动 fix me
//		if(gRobot.moveBase.actualPosition.x < posX_last)
//			vel = -vel;
		gRobot.moveBase.actualKenimaticInfo.vt = vel;
		/*if the actual speed is so small, the return value of the atan2(Vy, Vx) will be unstable, and
		the relative error will be too large.*/
		//To solve this problem, we set the vtAngle as 0.0f when the vt is too small.
		if(fabs(gRobot.moveBase.actualKenimaticInfo.vt) < 20.0f){
			gRobot.moveBase.actualKenimaticInfo.vtAngle = 0.0f;
		}
		else
		{
			gRobot.moveBase.actualKenimaticInfo.vtAngle = atan2f(vY, vX);
		}
		gRobot.moveBase.actualKenimaticInfo.journey += displacementLenth;
		gRobot.moveBase.actualKenimaticInfo.disp = sqrtf(\
														powf(gRobot.moveBase.actualPosition.x, 2.0f)\
														+ powf(gRobot.moveBase.actualPosition.y, 2.0f)\
															);

		posX_last = gRobot.moveBase.actualPosition.x;
		posY_last = gRobot.moveBase.actualPosition.y;
		updateTimeLast = updateTime;
	}
}

/**
  * @}
  */


///** @defgroup Motion_Control
//  * @brief
//  * @{
//  */
///**
//  * @brief	This Function will calculate the velocity of the three wheel
//  *				according to the targetAngle and targetVel.
//  * @note	This function just do the calculating job.
//  *			If the result of this function is always directly sent to the
//  *				the wheel, it is the best that the input were continuous.
//  * @param	targetAngle: Masured in radian. Zero angle point to the LEFT
//				side of the camera.
//  * @param	targetVel: Unit: mm/s^2
//  * @param	targetVel: Unit: mm/s^2
//  * @retval	a structure that contain the velocity of three wheel.
//  */
//wheelSpeed_t CalcWheelVel(	float targetTranslationalVelAngle, float targetTranslationalVel,\
//							float targetAngularSpeed)
//{
//	wheelSpeed_t wheelSpeed = {0};

//	wheelSpeed.forwardWheelSpeed	= Vel2Pulse(targetTranslationalVel * cosf(ANGTORAD(60.0f  + gRobot.moveBase.actualPosition.angle)\
//										- targetTranslationalVelAngle) + targetAngularSpeed * ROTATERAD);
//	wheelSpeed.leftWheelSpeed		= Vel2Pulse(targetTranslationalVel * cosf(ANGTORAD(180.0f + gRobot.moveBase.actualPosition.angle)\
//										- targetTranslationalVelAngle) + targetAngularSpeed * ROTATERAD);
//	wheelSpeed.backwardWheelSpeed	= Vel2Pulse(targetTranslationalVel * cosf(ANGTORAD(-60.0f  + gRobot.moveBase.actualPosition.angle)\
//										- targetTranslationalVelAngle) + targetAngularSpeed * ROTATERAD);

//	return wheelSpeed;
//}
///**
//  * @}
//  */


//#ifdef COLLECTING_DOT_ENABLE
///** @defgroup Record_Walk_Track
//  * @brief
//  * @{
//  */
///**
//  * @brief  Warning with beep when some error happen when recording walking track
//  * @note
//  * @param  None
//  * @retval None
//  */
//static void WarningRecordWalkTrackError(void)
//{
//	for(;;)
//	{
//		BEEP_ON;
//		TIM_Delayms(TIM5, 500);
//		BEEP_OFF;
//		TIM_Delayms(TIM5, 500);
//	}
//}

///**
//  * @brief  Record Walking Track
//  * @note   X, Y, angle, Journey, Displacement must be updated before calling this function
//  * @param	None
//  * @retval
//  */
//void RecordWalkingTrack(void)
//{
//	static uint16_t pointCnt = 0;
//	UpdateKenimaticInfo();
//	if(pointCnt < WALKTRACKDATABASE_POINT_CAPACITY)
//	{
//		if(fabs(gRobot.moveBase.actualKenimaticInfo.journey - gWalkTrackDatabase[pointCnt].journey) > 10.0f )
//		{
//			pointCnt++;
//			//Warning: The structure actualPossition must be fully updated before calling this function
//			gWalkTrackDatabase[pointCnt].x = gRobot.moveBase.actualPosition.x;
//			gWalkTrackDatabase[pointCnt].y = gRobot.moveBase.actualPosition.y;
//			gWalkTrackDatabase[pointCnt].angle = gRobot.moveBase.actualPosition.angle;
//			gWalkTrackDatabase[pointCnt].journey = gRobot.moveBase.actualKenimaticInfo.journey;
//			gWalkTrackDatabase[pointCnt].disp = gRobot.moveBase.actualKenimaticInfo.disp;
//			recordPointTotalNum ++;
//			Sendfloat(gWalkTrackDatabase[pointCnt].x);
//			Sendfloat(gWalkTrackDatabase[pointCnt].y);
//			Sendfloat(gWalkTrackDatabase[pointCnt].angle);
//			Sendfloat(gWalkTrackDatabase[pointCnt].journey);
//			Sendfloat(gWalkTrackDatabase[pointCnt].disp);
//			USART_SendData(UART5, (uint8_t)-100);
//			USART_SendData(UART5, (uint8_t)-100);
//			USART_SendData(UART5, (uint8_t)-100);
//			USART_SendData(UART5, (uint8_t)-100);
//		}
//	}
//	else
//	{
//		FlashWriteWalkTrackData();
//		WarningRecordWalkTrackError();
//	}
//}


///**
//  * @}
//  */
//#endif

///** @defgroup Path_Planning
//  * @brief
//  * @{
//  */

///**
//  * @brief Find the point that displacement that is the closest one to the LAUNCH_AREA_DISP
//  * @retval	None
//  */
//void FindClosestPoint(void)
//{
//	uint16_t closestPointCnt = 0u;
//	float closestPointDisp = 2000.0f;
//	for(uint16_t i = 0; i < recordPointTotalNum; i++)
//	{
//		if(gWalkTrackDatabase[i].disp < LAUNCH_AREA_DISP - 1.0f)
//			continue;
//		else if(fabs(gWalkTrackDatabase[i].disp - LAUNCH_AREA_DISP) < closestPointDisp)
//		{
//			closestPointDisp = fabs(gWalkTrackDatabase[i].disp - LAUNCH_AREA_DISP);
//			closestPointCnt = i;
//		}
//	}
//	offsetAngle = atan2(gWalkTrackDatabase[closestPointCnt].y, gWalkTrackDatabase[closestPointCnt].x);
//	Sendfloat(offsetAngle);
//	closetPointIndex = closestPointCnt;
//}

///**
//  * @brief	Plan the path to the final destination
//  * @note	This fuction find the equivalent point in the WalkTrackDatabase
//  * 		This fuction is only suitable for a straight line track now,
//  *			as it just use the displacement to the origin to search the short
//  *			term target point.
//  * @param	finalDisp: Displcement between final target to origin
//  * @retval	a structure that report the short term target point
//  */
//recordWalkTrackInfo_t PlanPath(float finalDisp /*abbr. targetDisplacement 目标点相对与原点的位移*/)
//{
//	static uint16_t pointCnt = 0u;
//	uint16_t pointCntStart = 0u;
//	float posErr = 0.0f;

//	//Find equivalant point in the WalkTrackDatabase
//	//等价点即在数据库中与当前点在路程Journey上相差最近的点
//	//fix me, 由于gWalkTrackDatabase中 点的间距非常短所以没有做插值处理

//	pointCntStart = pointCnt;
//	while(fabs(gWalkTrackDatabase[pointCnt].disp - gRobot.moveBase.actualKenimaticInfo.disp) > 15.0f
//		|| fabs(gWalkTrackDatabase[pointCnt].journey - gRobot.moveBase.actualKenimaticInfo.journey) > 300.0f)
//	{
//		pointCnt ++;
//		if(pointCnt >= recordPointTotalNum){
//			pointCnt = 0;
//		}
//		//fix me 如果在gWalkTrackDatabase中搜索不到等价点直接输出最靠近终点的点
//		if(pointCnt == pointCntStart)
//		{
//			BEEP_ON;
//			return gWalkTrackDatabase[closetPointIndex];
//		}
//	}
//	// position error between the actual position and the equivalent position
//	// Unit: mm
//	posErr = sqrtf(\
//				powf(gWalkTrackDatabase[pointCnt].x - gRobot.moveBase.actualPosition.x, 2.0f)\
//				+ powf(gWalkTrackDatabase[pointCnt].y - gRobot.moveBase.actualPosition.y, 2.0f)\
//					);

//	//estimate the Lookahead distance and find the target point

//	//与行走的方向相关

//	if(posErr > 0.75f *LOOKAHEAD_LENTH)
//	{
//		posErr = 0.75f * LOOKAHEAD_LENTH;
//	}
//	//fix me 如果在gWalkTrackDatabase中搜索不到目标点直接输出最靠近终点的点
//	pointCntStart = pointCnt;
//	if(finalDisp >= gRobot.moveBase.actualKenimaticInfo.disp)
//	{
//		while((fabs(gWalkTrackDatabase[pointCnt].disp - (gRobot.moveBase.actualKenimaticInfo.disp + LOOKAHEAD_LENTH - posErr)) > 15.0f
//				&& fabs(gWalkTrackDatabase[pointCnt].disp - finalDisp) > 15.0f)
//			|| fabs(gWalkTrackDatabase[pointCnt].journey - gRobot.moveBase.actualKenimaticInfo.journey) > 300.0f)
//		{
//			pointCnt ++;
//			if(pointCnt >= recordPointTotalNum){
//				pointCnt = 0;
//			}
//			if(pointCnt == pointCntStart)
//			{
//				BEEP_ON;
//				return gWalkTrackDatabase[closetPointIndex];
//			}
//		}
//	}
//	else/* if(finalDisp < gRobot.moveBase.actualKenimaticInfo.disp)*/
//	{
//		while((fabs(gWalkTrackDatabase[pointCnt].disp - (gRobot.moveBase.actualKenimaticInfo.disp - LOOKAHEAD_LENTH + posErr)) > 15.0f
//				&& fabs(gWalkTrackDatabase[pointCnt].disp - finalDisp) > 15.0f)
//			|| fabs(gWalkTrackDatabase[pointCnt].journey - gRobot.moveBase.actualKenimaticInfo.journey) > 300.0f)
//		{
//			if(pointCnt > 0){
//				pointCnt --;
//			}
//			else{
//				pointCnt = recordPointTotalNum - 1;
//			}
//			if(pointCnt == pointCntStart)
//			{
//				BEEP_ON;
//				return gWalkTrackDatabase[closetPointIndex];
//			}
//		}
//	}
//	return gWalkTrackDatabase[pointCnt];
//}


///**
//  * @brief	Plan the direction
//  * @note
//  * @param	targetPosition: a pointer
//  * @retval	a float that contain the infomation to target angle
//  */
//float PlanDirection(const recordWalkTrackInfo_t *targetPosition, float finalDisp)
//{
//	static float lastFinalDisp = 0.0f;
//	//计算X方向偏差和Y方向偏差
//	float errX = targetPosition->x - gRobot.moveBase.actualPosition.x;
//	float errY = targetPosition->y - gRobot.moveBase.actualPosition.y;
//	//计算与最终目标点的
//	float exAngle = atan2f(errY, errX);
//	float angleErr = exAngle - gRobot.moveBase.actualKenimaticInfo.vtAngle;
//	float dispToFinal = finalDisp - gRobot.moveBase.actualKenimaticInfo.disp;
//	if(gRobot.moveBase.actualKenimaticInfo.vtAngle  == 0.0f)
//	{
//		//在UpdateKenimatic中 速度太小时 把实际速度方向vtAngle赋值为了0.0f
//		//故不对exAngle做限制大小的处理
//	}
//	else
//	{
//		if(lastFinalDisp != finalDisp)
//		{
//			lastFinalDisp = finalDisp;
//			//Do nothing 不限幅度
//		}
//		else
//		{
//			while(angleErr > PI)
//			{
//				angleErr -= 2*PI;
//			}
//			while(angleErr <= -PI)
//			{
//				angleErr += 2*PI;
//			}
//			if(angleErr > ANGTORAD(15.0f) && angleErr <= ANGTORAD(90.0f))
//			{
//				exAngle = gRobot.moveBase.actualKenimaticInfo.vtAngle + ANGTORAD(15.0f);
//			}
//			else if(angleErr < ANGTORAD(-15.0f) && angleErr > ANGTORAD(-90.0f))
//			{
//				exAngle = gRobot.moveBase.actualKenimaticInfo.vtAngle - ANGTORAD(15.0f);
//			}
//			else if(angleErr > ANGTORAD(90.0f) && angleErr < ANGTORAD(160.0f))
//			{
//				exAngle = gRobot.moveBase.actualKenimaticInfo.vtAngle + ANGTORAD(160.0f);
//			}
//			else if(angleErr < ANGTORAD(-90.0f) && angleErr > ANGTORAD(-160.0f))
//			{
//				exAngle = gRobot.moveBase.actualKenimaticInfo.vtAngle - ANGTORAD(160.0f);
//			}

//			while(exAngle > PI)
//			{
//				exAngle -= 2*PI;
//			}
//			while(exAngle <= -PI)
//			{
//				exAngle += 2*PI;
//			}
//		}
//	}
//#ifdef RED_FIELD
//	if(dispToFinal >= 0.0f && dispToFinal < 100.0f){
//		exAngle = ANGTORAD(-.0f);
//	}
//	else if(dispToFinal < 0.0f && dispToFinal > -100.0f){
//		exAngle = ANGTORAD(-172.0f);
//	}
//	else if(dispToFinal > 0.0f && dispToFinal < 1500.0f){
//		exAngle = ANGTORAD(-4.0f);
//	}
//	else if(dispToFinal < 0.0f && dispToFinal > -1500.0f){
//		exAngle = ANGTORAD(-176.0f);
//	}
//#endif
//#ifdef BLUE_FIELD
//	if(dispToFinal >= 0.0f && dispToFinal < 100.0f){
//		exAngle = ANGTORAD(-172.0f);
//	}
//	else if(dispToFinal < 0.0f && dispToFinal > -100.0f){
//		exAngle = ANGTORAD(-8.0f);
//	}
//	else if(dispToFinal > 0.0f && dispToFinal < 1500.0f){
//		exAngle = ANGTORAD(-176.0f);
//	}
//	else if(dispToFinal < 0.0f && dispToFinal > -1500.0f){
//		exAngle = ANGTORAD(-4.0f);
//	}
//#endif
//	exAngle -= offsetAngle;
//	while(exAngle > PI)
//	{
//		exAngle -= 2*PI;
//	}
//	while(exAngle <= -PI)
//	{
//		exAngle += 2*PI;
//	}
//	return exAngle;
//}


///**
//  * @}
//  */


///** @defgroup Velocity_Planning
//  * @brief
//  * @{
//  */

///**
//  * @brief	Velocity PID control
//  * @note
//  * @param	expectDisp: abbr. expect value of displacement Unit: mm
//  * @param	actualDisp： actual value of displacement Unit: mm
//  * @retval
//  */
//static float VePIDControl(float expectDisp, float actualDisp)
//{
//	#define KP_VEL	2.0f
//	#define KI_VEL  0.0f

//	float velPIDOutput = KP_VEL * (expectDisp - actualDisp);

//	if(velPIDOutput > 300.0f)
//		velPIDOutput = 300.0f;
//	else if(velPIDOutput < -300.0f)
//		velPIDOutput = -300.0f;
//	return velPIDOutput;
//	#undef KP_VEL
//	#undef KI_VEL
//}

///**
//  * @brief	Position PID control
//  * @note
//  * @param	expectDisp: abbr. expect value of displacement Unit: mm
//  * @param	actualDisp： actual value of displacement Unit: mm
//  * @retval
//  */
//static float PositionPIDControl(float expectDisp, float actualDisp)
//{
//	#define KP_POSITION	6.0f
//	#define KI_POSITION  0.0f

//	float positionPIDOutput = KP_POSITION * (expectDisp - actualDisp);

//	if(positionPIDOutput > 300.0f)
//		positionPIDOutput = 300.0f;
//	else if(positionPIDOutput < -300.0f)
//		positionPIDOutput = -300.0f;
//	return positionPIDOutput;
//	#undef KP_POSITION
//	#undef KI_POSITION
//}

///**
//  * @brief	Planning the velocity
//  * @note	This fuction MUST BE CALL EVERY 10ms AND 电机控制周期也必须在10ms, 否则可能错位
//  * @param	the displacement of final destination  Unit: mm
//  * @param	the absolute value of target target velocity  Unit: mm/s
//  * @param	the absolute value of target Acceleration  Unit: mm/s^2
//  * @param	the absolute value of target Decelaration  Unit: mm/s^2
//  * @retval
//  */
//float PlanTranslationalVel(float finalDisp, float targetVel, float targetAcc, float targetDec)
//{
//	static float lastFinalDisp = 0.0f;
//	float dispToDisp = 0.0f;
//	float planVel = 0.0f;

//	static float virtualVel = 0.0f;
//	static float virtualDisp = 0.0f;
//	dispToDisp = finalDisp - gRobot.moveBase.actualKenimaticInfo.disp;

//	//参数检验 如果输入参数有误 蜂鸣器将报警
//	if(targetAcc < 0.0f)
//	{
//		targetAcc = -targetAcc;
//		BEEP_ON;
//	}
//	if (targetAcc > ACC_UPPER_BOUND)
//	{
//		targetAcc = ACC_UPPER_BOUND;
//		BEEP_ON;
//	}

//	if(targetDec < 0.0f)
//	{
//		targetDec = -targetDec;
//		BEEP_ON;
//	}
//	if(targetDec > DEC_LOWER_BOUND)
//	{
//		targetDec = DEC_LOWER_BOUND;
//		BEEP_ON;
//	}

//	if(targetVel < 0.0f){
//		targetVel = -targetVel;
//		BEEP_ON;
//	}
//	if(targetVel > VEL_UPPER_BOUND){
//		targetVel = VEL_UPPER_BOUND;
//		BEEP_ON;
//	}


//	/*为了使不同段的速度速度规划不互相干扰 virtualDisp 与 actualDisp 不至于相差过大*/
//	if(finalDisp != lastFinalDisp)
//	{
//		virtualDisp = gRobot.moveBase.actualKenimaticInfo.disp;
//		virtualVel = gRobot.moveBase.actualKenimaticInfo.vt;
//		lastFinalDisp = finalDisp;
//	}

////	if(fabs(gRobot.moveBase.actualKenimaticInfo.disp - finalDisp) < 200.0f
////		&& fabs(virtualDisp - finalDisp) < 200.0f)
////	{
////		virtualDisp = finalDisp;
////	}

//	//换算到 mm/s/(10ms) 的单位 以后targetAcc, targetDec 可以看作下一个周期10ms内的速度增量 方便使用
//	//但是如果遇到运动学公式的的代入时必须注意单位的大小
//	targetAcc /= 100.0f;
//	targetDec /= 100.0f;
//	if(dispToDisp >= 0)
//	{
//		if(virtualVel >= 0.0f)
//		{
//			if(dispToDisp < virtualVel * virtualVel / (100.0f * targetDec)/ 2.0f){		//s < v^2 / (2*a)
//				virtualVel -= targetDec;
//			}
//			else if(virtualVel < targetVel - targetAcc){
//				virtualVel += targetAcc;
//			}
//			else if(virtualVel > targetVel + targetDec){
//				virtualVel -= targetDec;
//			}
//			else
//			{
//				virtualVel = targetVel;
//			}

//		}
//		else/* if(virtualVel < 0.0f)*/
//		{
//			virtualVel += targetDec;
//		}
//	}
//	else/* if(dispErr < 0)*/
//	{
//		//fix me, 注意这里只是为了计算方便而把dispToDisp 置为正数
//		dispToDisp = -dispToDisp;
//		if(virtualVel <= 0.0f)
//		{
//			if(dispToDisp < virtualVel * virtualVel / (100.0f * targetDec) / 2.0f){		//s < v^2 / (2*a)
//				virtualVel += targetDec;
//			}
//			else if(virtualVel > -targetVel + targetAcc){
//				virtualVel -= targetAcc;
//			}
//			else if(virtualVel < -targetVel -targetDec){
//				virtualVel -= targetDec;
//			}
//			else
//			{
//				virtualVel = -targetVel;
//			}
//		}
//		else// if(virtualVel > 0.0f)
//		{
//			virtualVel -= targetDec;
//		}
//	}
//			//fix me
//			/*确保速度连续, 对于给定速度和实际速度不可以有太大的差值
//			但是实际测试中发现轮速等换算不准, 可能使得车不停加速*/
////			if(virtualVel > gRobot.moveBase.actualKenimaticInfo.vt + ACC_UPPER_BOUND / 50.0f){
////				virtualVel = gRobot.moveBase.actualKenimaticInfo.vt + ACC_UPPER_BOUND / 50.0f;
////			}
////			else if(virtualVel < gRobot.moveBase.actualKenimaticInfo.vt - DEC_LOWER_BOUND / 50.0f){
////				virtualVel = gRobot.moveBase.actualKenimaticInfo.vt - DEC_LOWER_BOUND / 50.0f;
////			}
//	//fix me 此处与调用周期有关
//	virtualDisp += virtualVel * 0.01f;
//	if(virtualDisp > gRobot.moveBase.actualKenimaticInfo.disp + 150.0f){
//		virtualDisp = gRobot.moveBase.actualKenimaticInfo.disp + 150.0f;
//		BEEP_ON;
//	}
//	else if(virtualDisp < gRobot.moveBase.actualKenimaticInfo.disp - 150.0f){
//		virtualDisp = gRobot.moveBase.actualKenimaticInfo.disp - 150.0f;
//		BEEP_ON;
//	}
//	else
//		BEEP_OFF;

////	Sendfloat(virtualDisp);
////	Sendfloat(gRobot.moveBase.actualKenimaticInfo.disp);

//	if(fabs(dispToDisp) < 200.0f){
//		//取符号 利用 v = sqrt(2as)  再加上PID调节
//		planVel =(0.25f + (fabs(dispToDisp) / 250.0f)) *  sqrtf(2.0f * targetDec * 100.0f * fabs(dispToDisp));
//	}
//	else{
//		planVel=virtualVel + PositionPIDControl(virtualDisp, gRobot.moveBase.actualKenimaticInfo.disp);
////		planVel=virtualVel;
//	}
////	Sendfloat(virtualVel);
//	//fix me 由于输出值为速度的绝对值，速度的方向已经有其他函数给出，此处应有更好的结构输出
//	if(planVel < 0.0f)
//		planVel = -planVel;
//	return planVel;
//}


///**
//  * @}
//  */





///** @defgroup	Pose_Adjust
//  * @brief		姿态闭环 即机器人的朝向
//  * @{
//  */
///**
//  * @brief	姿态闭环  Input:姿态 OutPut:旋转角速度
//  * @note	带限幅 积分输出最大不超过3°每秒 整体输出不超过6°每秒
//  * @param	targetPoseAngle:
//  * @param	actualPoseAngle:
//  * @retval	angularSpeed: Mesure in Radius
//  */
//float PosePIDControl(float targetPoseAngle, float actualPoseAngle)
//{
//	float errAngle = targetPoseAngle - actualPoseAngle;
//	float angularSpeed = 0.0f;
//	static float Iterm = 0.0f;
//	#define KP_POSE	3.0f
//	#define KI_POSE 0.0f
//	#define ITERM_POSE_UPPER_BOUND  ANGTORAD(3.0f)
//	#define POSE_OUTPUT_UPPER_BOUND ANGTORAD(6.0f)
//	/*具备限幅的I调节*/
//	//积分
//	Iterm += KI_POSE * errAngle;
//	//限幅
//	if(Iterm > ITERM_POSE_UPPER_BOUND)
//		Iterm = ITERM_POSE_UPPER_BOUND;
//	else if(Iterm < -ITERM_POSE_UPPER_BOUND)
//	{
//		Iterm = -ITERM_POSE_UPPER_BOUND;
//	}

//	//P+I
//	angularSpeed = KP_POSE * ANGTORAD(errAngle) + Iterm;

//	/*PI调节整体限幅*/
//	if(angularSpeed > POSE_OUTPUT_UPPER_BOUND)
//		angularSpeed = POSE_OUTPUT_UPPER_BOUND;
//	else if(angularSpeed < -POSE_OUTPUT_UPPER_BOUND)
//	{
//		angularSpeed = -POSE_OUTPUT_UPPER_BOUND;
//	}

//	return angularSpeed;
//	#undef KP_POSE
//	#undef KI_POSE
//	#undef ITERM_POSE_MAX
//	#undef POSE_OUTPUT_UPPER_BOUND
//}


///**
//  * @}
//  */


///** @defgroup
//  * @brief
//  * @{
//  */
//static uint16_t walkTime = 0u;
///**
//  * @brief
//  * @note
//  * @param
//  *	 @arg
//  * @param
//  * @retval
//  */
//void CountWalkTime(void)
//{
//	if(walkTime < 65535)
//		walkTime ++;
//}


///**
//  * @brief
//  * @note
//  * @param
//  *	 @arg
//  * @param
//  * @retval
//  */
//uint16_t ReadWalkTime(void)
//{
//	USART_SendData(UART5, (uint8_t)-100);
//	USART_SendData(UART5, (uint8_t)-100);
//	USART_SendData(UART5, (uint8_t)-100);

//	USART_SendData(UART5, (uint8_t)walkTime/1000);
//	USART_SendData(UART5, (uint8_t)-100);
//	USART_SendData(UART5, (uint8_t)-100);
//	USART_SendData(UART5, (uint8_t)-100);
//	return walkTime;
//}
///**
//  * @}
//  */

///** @defgroup
//  * @brief
//  * @{
//  */

///**
//  * @brief
//  * @note	This function is supposed to be called every 10ms
//  * @param	None
//  * @retval	None
//  */
//void MovebaseRun(float finalDisp, float targetVel, float targetAcc, float targetDec, uint8_t enable)
//{
//	recordWalkTrackInfo_t	positionNextPeriod = {0};
//	float	   directionAngleNextPeriod = 0.0f;
//	float		translationalVelNextPeriod = 0.0f;
//	float		rotationalVelNextPeriod = 0.0f;
//	wheelSpeed_t wheelSpeedNextPeriod = {0.0f};

//	/*注意到actualPosition 之中x, y, angle 和 journey, disp 的更新是异步的*/
//	UpdateKenimaticInfo();
//	//This is a structure
//	positionNextPeriod	= PlanPath(finalDisp);

//	directionAngleNextPeriod = PlanDirection(&positionNextPeriod, finalDisp);

//	//此函数以后应该改为根据当前的速度方向和规划出切向方向的速度和法向方向的速度
//	translationalVelNextPeriod	= PlanTranslationalVel(finalDisp, targetVel, targetAcc, targetDec);
//	rotationalVelNextPeriod = PosePIDControl(positionNextPeriod.angle, gRobot.moveBase.actualPosition.angle);
//	wheelSpeedNextPeriod = CalcWheelVel(directionAngleNextPeriod, translationalVelNextPeriod, rotationalVelNextPeriod);

//	Sendfloat(RADTOANG(gRobot.moveBase.actualKenimaticInfo.vtAngle));
//	Sendfloat(RADTOANG(directionAngleNextPeriod));

//	Sendfloat(gRobot.moveBase.actualKenimaticInfo.disp);
//	Sendfloat(gRobot.moveBase.actualPosition.y);
////	Sendfloat(positionNextPeriod.disp - gRobot.moveBase.actualKenimaticInfo.disp);
////	Sendfloat(positionNextPeriod.journey - gRobot.moveBase.actualKenimaticInfo.journey);

//	Sendfloat(translationalVelNextPeriod);
//	Sendfloat(gRobot.moveBase.actualKenimaticInfo.vt);
////	Sendfloat(Pulse2Vel(wheelSpeedNextPeriod.leftWheelSpeed));


//	if(enable){
//		WheelVelControl(&wheelSpeedNextPeriod);
//	}
//}

///**
//  * @}
//  */


/********************* (C) COPYRIGHT NEU_ACTION_2017 *****************END OF FILE************************/


