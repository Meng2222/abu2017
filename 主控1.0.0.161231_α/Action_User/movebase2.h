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
#define RED_FIELD
//#define BLUE_FIELD
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
/**
  * @brief	设定位移量的矫正量 注意： 此矫正量并没有真正更改了gRobot 中的actualDisp
  * @param	theoreticalDisp 理论上相对于原点的位移
  * @retval	None
  */
void SetDispCorrection(float theoreticalDisp);

/**
  * @brief	为计算速度计数
  * @note	
 * @param	None
  * @retval	
  */
void velTimerCounting(void);

/**
  * @brief	获取位移的改正量
  * @note	
  * @retval	dispCorrection
  */
float GetDispCorrection(void);

/**
  * @brief  UpdateMoveBaseData
  * @note
  * @param  angle: data that will update the angle in gRobot.movebase.actualPos
  * @param  posX: data that will update the X pos in gRobot.movebase.actualPos
  * @param  posY: data that will update the Y pos in gRobot.movebase.actualPos
  * @retval None
  */
void UpdateXYAngle(float angle, float posX, float posY);

/**
  * @brief  Calculate the absolute value of actual velocity and return it
  *			This Function is suppose to be called every 10 ms
  *			如果数据没有更新，则输出上一次计算的的速度
  *			如果调用此函数之间超过多个10ms 也能输出正确的速度，
  * @note 	fix me actualPosition 的更新是异步的,必须配合UpdateMOveBaseData()调用
  * @param	None
  * @retval a float that is the absolute value of velocity Unit: mm/s^2
  */
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

//fix me  Group is wrong
/**
  * @brief Find the point that displacement that is the closest one to the LAUNCH_AREA_DISP
  * @retval	None
  */
void FindClosestPoint(void);

/**
  * @}
  */

void Sendfloat(float val);


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


