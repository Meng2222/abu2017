/**
  *******************************************************************************************************
  * @file	 rng.c
  * @author  ACTION_2017
  * @version V0.0.0.20170601_alpha
  * @date	 2017/06/01
  * @brief   This file contains function and example to create random number using the Random 
  *          Number Generator(RNG) firmware
  *
  *******************************************************************************************************
  * @SimpleExaple
  *	RNG_Config();
  * rand0 = (float)(RNG_Get_RandomNum()/0xffffffff)>0.5f;
  * rand1 = (float)(RNG_Get_RandomNum()/0xffffffff)>0.5f;
  * rand2 = (float)(RNG_Get_RandomNum()/0xffffffff)>0.5f;
  * RNG_Cmd(DISABLE);
  *
  *******************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------*/

#include "rng.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "usart.h"

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

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

void RNG_Config(void)
{
	uint16_t retry=0;
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE); //开启 RNG 时钟
	RNG_Cmd(ENABLE); //使能 RNG
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET&&retry<10000)//等待就绪
	{
		retry++;
		delay_us(100);
	}
	if(retry>=10000)
	{
		UART5_OUT((uint8_t *)"RNG Config ERROR!!\r\n");
	}
}

/*
*名称：RNG_Get_RandonNum
*功能：获取随机数
*参数：void
*注意：
*
*/
uint32_t RNG_Get_RandomNum(void)
{
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)==RESET); //等待随机数就绪
	return RNG_GetRandomNumber();
}

/**
  * @}
  */

/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
