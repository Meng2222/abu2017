/**
  ******************************************************************************
  * @file    database.h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017/03/21
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DATABASE_H
#define __DATABASE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "robot.h"

/* Exported types ------------------------------------------------------------*/

/** @defgroup 
  * @brief 
  * @{
  */

typedef struct
{
	float x;
	float y;
	float angle;
	//路程
	float journey;
	//abbr. displacement
	float disp;
}recordWalkTrackInfo_t;

/**
  * @}
  */

 
/* Exported constants --------------------------------------------------------*/



/** @defgroup Length_Of_Gun_Postion_Database
  * @brief    用其中浮点数的个数衡量 因这三个database的元素都为浮点数
  * @{
  */

#define LEFTGUNPOSDATABASE_FLOAT_NUM	(sizeof(gLeftGunPosDatabase)/sizeof(float))
#define RIGHTGUNPOSDATABASE_FLOAT_NUM	(sizeof(gRightGunPosDatabase)/sizeof(float))
#define UPPERGUNPOSDATABASE_FLOAT_NUM	(sizeof(gUpperGunPosDatabase)/sizeof(float))

/**
  * @}
  */

/** @defgroup Length_of_Walk_Track_Database
  * @brief    采集的点的个数 按照5cm一个点测
  * @{
  */

#define WALKTRACKDATABASE_POINT_CAPACITY     500


/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported Variables --------------------------------------------------------*/


/** @defgroup Gun_Shoot_Position_Database
  * @{
  */

extern gun_pose_t gLeftGunPosDatabase[SHOOT_POINT_NUM][SHOOT_METHOD_NUMBER][LAND_NUMBER];
extern gun_pose_t gRightGunPosDatabase[SHOOT_POINT_NUM][SHOOT_METHOD_NUMBER][LAND_NUMBER];
extern gun_pose_t gLeftGunReloadPosDatabase[SHOOT_POINT_NUM][SHOOT_METHOD_NUMBER][LAND_NUMBER];
extern gun_pose_t gRightGunReloadPosDatabase[SHOOT_POINT_NUM][SHOOT_METHOD_NUMBER][LAND_NUMBER];

extern gun_pose_t gUpperGunPosDatabase[SHOOT_POINT_NUM][LAND_NUMBER][UPPER_SHOOT_METHOD_NUMBER][ZONE_NUMBER];


/**
  * @}
  */

extern uint8_t LeftGunPriority[7];
extern uint8_t RightGunPriority[7];
extern uint8_t UpperGunPriority[7];

/** @defgroup Gun_Shoot_Command
  * @brief 
  * @{
  */

extern shoot_command_t gLeftGunShootCmds[LEFT_GUN_AUTO_SHOOT_STEP_NUMBER];
extern shoot_command_t gRightGunShootCmds[RIGHT_GUN_AUTO_SHOOT_STEP_NUMBER];
extern shoot_command_t gUpperGunShootCmds[UPPER_GUN_AUTO_STEP_NUMBER];


//extern shoot_command_t gUpperGunShootCmds;

/**
  * @}
  */


/** @defgroup Walk_Track_Database
  * @brief 
  * @{
  */

extern posture_t gWalkTrackDatabase[WALKTRACKDATABASE_POINT_CAPACITY];

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
///**
//  * @brief  Update LeftGunPosDatabase in Manual Mode
//  * @note   this function will update the 
//  * @param  None
//  * @retval None
//  */
//void UpdateLeftGunPosDatabaseManualMode(void);

///**
//  * @brief  Update RightGunPosDatabase in Manual Mode
//  * @note   this function will update the 
//  * @param  None
//  * @retval None
//  */
//void UpdateRightGunPosDatabaseManualMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __DATABASE_H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

