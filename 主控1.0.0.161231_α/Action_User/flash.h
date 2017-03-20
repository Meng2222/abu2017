/**
  ******************************************************************************
  * @file    *.h
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
#define FlashFloatDataLen  (sizeof(gGunPosDatabaseFromFlash)/sizeof(float))
/* Exported functions ------------------------------------------------------- */
void Flash_Write(uint8_t *data,uint32_t len);
void Flash_Zero(uint32_t len);
void Flash_Read(uint8_t *data,uint32_t len);
void Flash_Init(void);
uint8_t  *GetFlashArr(void);
float    *GetResultArr(void);
uint32_t *GetCountArr(void);
uint8_t  GetFlashUpdataFlag(void);
void     SetFlashUpdateFlag(uint8_t val);
	

void FlashWriteFloatArr(float *data, uint32_t len);

#ifdef __cplusplus  //为C提供接口
}
#endif	
	
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/
