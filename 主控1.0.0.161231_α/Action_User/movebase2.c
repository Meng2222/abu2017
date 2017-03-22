/**
  ******************************************************************************
  * @file    movebase2.c
  * @author  ACTION_2017
  * @version V0.0.0.170321_alpha
  * @date    2017/03/21
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  * 
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "movebase2.h"
#include "database.h"

/* Private typedef -----------------------------------------------------------*/

/** 
  * @brief  
  */

//typedef struct
//{
//	
//}_t;

/* Private define ------------------------------------------------------------*/

/** @defgroup Record_Walk_Track
  * @brief 
  * @{
  */

/**
  * @brief  
  * @note 
  * @param  
  *     @arg 
  * @param 
  * @retval 
  */
void RecordWalkingTrack(void)
{
	static uint16_t pointCnt = 0;
	extern robot_t gRobot;
	if(pointCnt < WALKTRACKDATABASE_POINT_CAPACITY)
	{
		if(gRobot.moveBase.actualXPos - gWalkTrackDatabase[pointCnt].x)
		{
			gWalkTrackDatabase[pointCnt].x = gRobot.moveBase.actualXPos;
			gWalkTrackDatabase[pointCnt].y = gRobot.moveBase.actualYPos;
			gWalkTrackDatabase[pointCnt].angle = gRobot.moveBase.actualAngle;
		}
	}

}


/**
  * @}
  */
  
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  
  * @note
  * @param  
  * @retval None
  */

/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/


