/**
  ******************************************************************************
  * @file    GET_SET.h
  * @author  ST42
  * @version 
  * @date   
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GET_SET_H
#define __GET_SET_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SetPosX(float val);
float GetPosX(void);

void SetPosY(float val);
float GetPosY(void);

void SetAngle(float val);
float GetAngle(void);

void UpdateVel(void);
float GetVel(void);

void SetMotorVel(uint8_t *value);
uint8_t GetMotorVel(int motorNum);
float GetMotorVelRate(void);
float GetMotorVelAng(void);

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

