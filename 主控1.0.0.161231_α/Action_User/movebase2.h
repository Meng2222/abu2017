/**
  ******************************************************************************
  * @file    movebase2.h
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE2_H
#define __MOVEBASE2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"
#include "robot.h"
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/




/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup Get_Kinematic_Information
  * @brief
  * @{
  */

/**
  * @brief	设定X坐标的矫正量
  * @param	theoreticalX 理论坐标
  * @param	actualX 实际坐标
  * @retval	None
  */
void SetXCorrection(float theoreticalX);
void SetDispCorrection(float theoreticalDisp);

/**
  * @brief  UpdateMoveBaseData
  * @note
  * @param  angle: data that will update the angle in gRobot.movebase.actualPos
  * @param  posX: data that will update the X pos in gRobot.movebase.actualPos
  * @param  posY: data that will update the Y pos in gRobot.movebase.actualPos
  * @retval None
  */
void UpdateXYAngle(float angle, float posX, float posY);
void UpdateKenimaticInfo(void);


/**
  * @}
  */




/** @defgroup Record_Walk_Track
  * @brief
  * @{
  */

/**
  * @brief  Record Walking Track
  * @note   X, Y, angle, Journey, Displacement must be updated before calling this function
  * @param	None
  * @retval
  */
void RecordWalkingTrack(void);

/**
  * @}
  */



/** @defgroup 
  * @brief 
  * @{
  */

/**
  * @brief	
  * @note	This function is supposed to be called every 10ms
  * @param	None
  * @retval	None
  */
void MovebaseRun(float finalDisp, float targetVel, float targetAcc, float targetDec, uint8_t enable);

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* __MOVEBASE2_H */




/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/


