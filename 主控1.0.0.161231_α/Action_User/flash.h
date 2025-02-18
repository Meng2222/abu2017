/**
 ******************************************************************************
 * @file	*.h
 * @author  Lxy Action
 * @version 
 * @date   
 * @brief   This file contains the headers of 
 ******************************************************************************
 * @attention
 *
 *
 * 
 * 
 *
 ******************************************************************************
 */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus  //为C提供接口
extern "C"
{
#endif

	/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

	/* Exported types ------------------------------------------------------------*/
	/* Exported constants --------------------------------------------------------*/
	/* Exported macro ------------------------------------------------------------*/
	/* Exported functions ------------------------------------------------------- */
void Flash_Write(uint8_t *data,uint32_t len);
void Flash_Zero(uint32_t len);
void Flash_Read(uint8_t *data,uint32_t len);
/**
 * @brief  初始化FLASH
 * @note   如果定义了WRITE_DATABASE_IN_FLASH_AT_BEGINNING
 *				则写入数据，FLASH中存储的原有数据全部将被擦除,并且被database.c中的数据代替
 *			如果没有定义WRITE_DATABASE_IN_FLASH_AT_BEGINNING，
 *				则database.c中的数据将无效，三个数组中的数据将会被FLASH中以前记录下来的数据代替
 * @param  None
 * @retval None
 */
void FlashInit(void);
uint8_t  *GetFlashArr(void);
float	*GetResultArr(void);
uint32_t *GetCountArr(void);
uint8_t  GetFlashUpdataFlag(void);
void	 SetFlashUpdateFlag(uint8_t val);


void FlashWriteFloatArr(float *data, uint32_t len);


/**
  * @brief  向FLASH中写入3个枪的位置数据
  * @note   此函数将会使用大概一秒钟的时间使用时最好慎重考虑
  * @param  None
  * @retval None
  */
void FlashWriteGunPosData(void);
/**
 * @brief  初始化FLASH
 * @note   如果定义了WRITE_DATABASE_IN_FLASH_AT_BEGINNING
 *				则写入数据，FLASH中存储的原有数据全部将被擦除,并且被database.c中的数据代替
 *			如果没有定义WRITE_DATABASE_IN_FLASH_AT_BEGINNING，
 *				则database.c中的数据将无效，三个数组中的数据将会被FLASH中以前记录下来的数据代替
 * @param  None
 * @retval None
 */
void FlashWriteGunPosData(void);

#ifdef __cplusplus  //为C提供接口
}
#endif	

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
