/**
 ******************************************************************************
 * @file     flash.c
 * @author   lxy DZC
 * @version
 * @date     2017/03/21
 * @brief
 ******************************************************************************
 * @attention
 *
 *
 *
 *
 ******************************************************************************
 */
/* Includes -------------------------------------------------------------------*/

#include "flash.h"
#include "stm32f4xx_flash.h"


#include "gpio.h"
#include "database.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define FLASH_SECTOR_6_BLOCK_BASE_ADDRESS     0x08040000   //FLASH起始地址 STM32F407VET6第6扇区的起始地址
#define FLASH_SECTOR_7_BLOCK_BASE_ADDRESS     0x08060000   //FLASH起始地址 STM32F407VET6第6扇区的起始地址
//#define WRITE_DATABASE_IN_FLASH_AT_BEGINNING 1

/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/
/**
 * @brief  FLASH加密
 *
 * @param  None
 * @retval None
 */
static void Flash_Encryp(void)
{
#ifdef FLASH_ENCRYP

	/* 判断FLASH是否被保护 */
	if(SET != FLASH_OB_GetRDP())
	{
		/*若不被保护*/
		FLASH_Unlock();
		FLASH_OB_Unlock();
		/* 等级0不保护，等级1可逆读保护，等级2不可逆读保护 */
		FLASH_OB_RDPConfig(OB_RDP_Level_1);
		FLASH_OB_Launch();
		FLASH_OB_Lock();
		FLASH_Lock();
	}
#endif
}
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
 * @brief  向FLASH中写入数据
 * @note   正常来说使用3.3V时可以按照32位写入 
 * @param  data: 存储数据的指针
 * @param  len:  写入的数据量，单位：byte
 * @retval None
 */
void Flash_Write(uint8_t *data,uint32_t len)
{
	uint32_t count;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3);
	FLASH_WaitForLastOperation();
	for(count = 0; count < len; count++)
	{
		FLASH_ProgramByte(FLASH_SECTOR_6_BLOCK_BASE_ADDRESS + count, *(data + count));
	}
	FLASH_Lock();
}





/**
 * @brief  将FLASH中的数据改成0
 *
 * @param  len  :  改写的数据量，单位：byte
 * @retval None
 */
void Flash_Zero(uint32_t len)
{
	uint32_t count;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_EraseSector(FLASH_Sector_6, VoltageRange_3);
	FLASH_WaitForLastOperation();
	for(count = 0;count < len; count++)
	{
		FLASH_ProgramByte(FLASH_SECTOR_6_BLOCK_BASE_ADDRESS + count, 0x00);
	}
	FLASH_Lock();
}


/** @defgroup 
  * @brief 
  * @{
  */

/**
  * @brief  从FLASH中读取浮点数
  *
  * @param  storage :  存储数据的指针
  * @param  flashAddr: FLASH的起始地址
  * @param  len  :  读取的数据量，单位：byte
  * @retval None
  */
void Flash_ReadFloat(float *data, uint32_t flashAddr, uint32_t len)
{
	uint32_t i;
	for(i = 0; i < len; i++)
		*(data+i)= *((float *)(flashAddr) + i);
}



/**
  * @brief  初始化FLASH
  * @note   如果定义了WRITE_DATABASE_IN_FLASH_AT_BEGINNING
  *				则写入数据，FLASH中存储的原有数据全部将被擦除,并且被database.c中的数据代替
  *			如果没有定义WRITE_DATABASE_IN_FLASH_AT_BEGINNING，
  *				则database.c中的数据将无效，三个数组中的数据将会被FLASH中以前记录下来的数据代替
  * @param  None
  * @retval None
  */
void FlashInit(void)
{
#ifdef WRITE_DATABASE_IN_FLASH_AT_BEGINNING
	BEEP_ON;
	FlashWriteGunPosData();
	BEEP_OFF;

#endif
	/* 读取FLASH中保存的数据，并将其存到RAM里 */
	Flash_ReadFloat((float *)gLeftGunPosDatabase, FLASH_SECTOR_6_BLOCK_BASE_ADDRESS, LEFTGUNPOSDATABASE_FLOAT_NUM);
	Flash_ReadFloat((float *)gRightGunPosDatabase, \
					FLASH_SECTOR_6_BLOCK_BASE_ADDRESS + 4 * LEFTGUNPOSDATABASE_FLOAT_NUM, \
					RIGHTGUNPOSDATABASE_FLOAT_NUM);
//	Flash_ReadFloat((float *)gUpperGunPosDatabase,\
//					FLASH_SECTOR_6_BLOCK_BASE_ADDRESS + 4 * (LEFTGUNPOSDATABASE_FLOAT_NUM + RIGHTGUNPOSDATABASE_FLOAT_NUM), \
//					UPPERGUNPOSDATABASE_FLOAT_NUM);

	/* 保护Flash数据 */
	Flash_Encryp();
}


///**
//  * @brief	Read data from FLASH Sector 7 and put them into gWalkTrackDatabas[]
//  * @note	
//  * @param	None
//  * @retval	None
//  */
//void FlashReadWalkTrackData(void)
//{
//	Flash_ReadFloat((float *)gWalkTrackDatabase, FLASH_SECTOR_7_BLOCK_BASE_ADDRESS, recordPointTotalNum);
//}

/**
  * @}
  */


/** @defgroup 
  * @brief 
  * @{
  */



///**
//  * @brief	Write the WalkTrackDatabase into Flash Sector7
//  * @note	
//  * @param	None
//  * @retval	None
//  */
//void FlashWriteWalkTrackData(void)
//{
//	uint32_t count = 0u;
//	FLASH_Unlock();
//	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
//	FLASH_EraseSector(FLASH_Sector_7, VoltageRange_3);
//	FLASH_WaitForLastOperation();
//    
//	FLASH_ProgramWord(FLASH_SECTOR_7_BLOCK_BASE_ADDRESS + 4 * count, (uint32_t)recordPointTotalNum);
//    count ++;
//	for(; count < recordPointTotalNum; count++)
//	{
//		FLASH_ProgramWord(FLASH_SECTOR_7_BLOCK_BASE_ADDRESS + 4 * count, *(uint32_t *)((float *)gWalkTrackDatabase + count));
//	}
//    for(; count < 0x1FFFF; count++)
//    {
//       	FLASH_ProgramWord(FLASH_SECTOR_7_BLOCK_BASE_ADDRESS + 4 * count, (uint32_t)0x00000000u);
//    }
//	FLASH_Lock();
//}

/**
 * @brief  Write the Left/Right/Upper-GunPosDatabase into Flash Sector6
 * @param  None
 * @retval None
 */
void FlashWriteGunPosData(void)
{
    
	uint32_t count;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_EraseSector(FLASH_Sector_6, VoltageRange_3);
	FLASH_WaitForLastOperation();
	for(count = 0; count < LEFTGUNPOSDATABASE_FLOAT_NUM; count++)
	{
		FLASH_ProgramWord(FLASH_SECTOR_6_BLOCK_BASE_ADDRESS + 4 * count, *(uint32_t *)((float *)gLeftGunPosDatabase + count));
	}
	for(count = 0; count < RIGHTGUNPOSDATABASE_FLOAT_NUM; count++)
	{
		FLASH_ProgramWord(FLASH_SECTOR_6_BLOCK_BASE_ADDRESS + 4 * (LEFTGUNPOSDATABASE_FLOAT_NUM + count), *(uint32_t *)((float *)gRightGunPosDatabase + count));
	}
	for(count = 0; count < UPPERGUNPOSDATABASE_FLOAT_NUM; count++)
	{
		FLASH_ProgramWord(FLASH_SECTOR_6_BLOCK_BASE_ADDRESS + 4 * ( LEFTGUNPOSDATABASE_FLOAT_NUM + RIGHTGUNPOSDATABASE_FLOAT_NUM + count), *(uint32_t *)((float *)gUpperGunPosDatabase + count));
	}
	FLASH_Lock();
}

/**
  * @}
  */



/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/

