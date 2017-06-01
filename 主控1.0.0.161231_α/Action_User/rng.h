/**
  ******************************************************************************
  * @file    rng.h
  * @author  ACTION_2017
  * @version V0.0.0.20170601_alpha
  * @date    2017/06/01
  * @brief   This file contains all the functions prototypes for creating 
  *          random number using the Random Number Generator(RNG) firmware
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RNG_H
#define __RNG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx_rng.h"
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup Random_Number_Generation
  * @brief 
  * @{
  */

/**
*名称：RNG_Config
*功能：初始化随机数发生器
*参数：none
*注意：
*
*/
void RNG_Config(void);

/*
*名称：RNG_Get_RandonNum
*功能：获取随机数
*参数：void
*注意：
*
*/
uint32_t RNG_Get_RandomNum(void);

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* __RNG_H */

