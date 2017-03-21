/**
 ******************************************************************************
 * @file
 * @author  lxy
 * @version
 * @date
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
#include "string.h"
#include "robot.h"
#include "database.h"

/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define FLASH_USER_ADDRESS 0x08040000   //FLASH起始地址
//#define WRITE_DATABASE_IN_FLASH_AT_BEGINNING 1
//fix me 以下两个宏定义只是为了测试
//#define TempTable_max	200
//#define TempTable_min	100
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
//static uint8_t  flashdata[160*(TempTable_max-TempTable_min)];  //从flash中取出的数据
static float	*Result;
static uint32_t *countnum;
static uint8_t  flag=0;


//static gun_pose_t gGunPosDatabaseFromFlash[GUN_NUMBER][CHAMPER_BULLET_MAX_FEATURE_STATE][SHOOT_METHOD_NUMBER][LAND_NUMBER] = {0};
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
		/* 若不被保护 */
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
 *
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
		FLASH_ProgramByte(FLASH_USER_ADDRESS + count, *(data + count));
	}
	FLASH_Lock();
}


/**
 * @brief  向FLASH中写入浮点数数组
 *
 * @param  data: 浮点数数组的指针
 * @param  len:  浮点数数据量，单位：4byte
 * @retval None
 */
void FlashWriteFloatArr(float *data, uint32_t len)
{
	uint32_t count;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	FLASH_EraseSector(FLASH_Sector_6, VoltageRange_3);
	FLASH_WaitForLastOperation();
	for(count = 0; count < len; count++)
	{
		FLASH_ProgramHalfWord(FLASH_USER_ADDRESS + 4 * count, *(uint16_t *)(data + count));
		FLASH_ProgramHalfWord(FLASH_USER_ADDRESS + 4 * count + 2, *((uint16_t *)(data + count) + 1));
	}
	FLASH_Lock();
}

/**
 * @brief  向FLASH中写入3个浮点数数组
 * @param  data: 浮点数数组的指针
 * @param  len:  浮点数数据量，单位：4byte
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
		FLASH_ProgramHalfWord(FLASH_USER_ADDRESS + 4 * count, 		*(uint16_t *)((float *)gLeftGunPosDatabase + count));
		FLASH_ProgramHalfWord(FLASH_USER_ADDRESS + 4 * count + 2,	*((uint16_t *)((float *)gLeftGunPosDatabase + count) + 1));
	}
	for(; count < RIGHTGUNPOSDATABASE_FLOAT_NUM; count++)
	{
		FLASH_ProgramHalfWord(FLASH_USER_ADDRESS + 4 * count,		*(uint16_t *)((float *)gRightGunPosDatabase + count));
		FLASH_ProgramHalfWord(FLASH_USER_ADDRESS + 4 * count + 2,	*((uint16_t *)((float *)gRightGunPosDatabase + count) + 1));
	}
	for(; count < RIGHTGUNPOSDATABASE_FLOAT_NUM; count++)
	{
		FLASH_ProgramHalfWord(FLASH_USER_ADDRESS + 4 * count,		*(uint16_t *)((float *)gUpperGunPosDatabase + count));
		FLASH_ProgramHalfWord(FLASH_USER_ADDRESS + 4 * count + 2,	*((uint16_t *)((float *)gUpperGunPosDatabase + count) + 1));
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
		FLASH_ProgramByte(FLASH_USER_ADDRESS+count, 0x00);
	}
	FLASH_Lock();
}


/**
 * @brief  从FLASH中读取数据
 *
 * @param  data :  存储数据的指针
 * @param  len  :  读取的数据量，单位：byte
 * @retval None
 */
void Flash_Read(uint8_t *data,uint32_t len)
{
	uint32_t i;
	for(i=0;i<len;i++)
		*(data+i)= *((uint8_t *)(FLASH_USER_ADDRESS+i));
}

/**
 * @brief  从FLASH中读取数据
 *
 * @param  data :  存储数据的指针
 * @param  len  :  读取的数据量，单位：byte
 * @retval None
 */
void Flash_ReadFloat(float *data, uint32_t len)
{
	uint32_t i;
	for(i = 0; i < len; i++)
		*(data+i)= *((float *)(FLASH_USER_ADDRESS + 4*i));
}

///**
//  * @brief  初始化FLASH
//	*
//  * @param  None
//  * @retval None
//  */
//void Flash_Init(void)
//{
//	/* 读取FLASH中保存的数据，并将其存到RAM里 */
//	Flash_Read(flashdata,160*(TempTable_max-TempTable_min));
//	/* 一个温度可以分成10等分，每0.1度对应两个芯片，
//	  3个角速度，1个计数值，一共16字节 */
//	/* 分割数据段，将零漂值与计数值分开 */
//	Result=(float *)flashdata;
//	countnum=(uint32_t *)(flashdata)+30*(TempTable_max-TempTable_min);//类上

//	/* 保护Flash数据 */
//	Flash_Encryp();
//}

/**
 * @brief  初始化FLASH
 * @note   如果定义了WRITE_DATABASE_IN_FLASH_AT_BEGINNING
 *				则写入数据，FLASH中存储的原有数据全部将被擦除,并且被database.c中的数据代替
 *			如果没有定义WRITE_DATABASE_IN_FLASH_AT_BEGINNING，
 *				则database.c中的数据将无效，三个数组中的数据将会被FLASH中以前记录下来的数据代替
 * @param  None
 * @retval None
 */
void Flash_Init(void)
{
#ifdef WRITE_DATABASE_IN_FLASH_AT_BEGINNING
	FlashWriteGunPosData();
#endif
	/* 读取FLASH中保存的数据，并将其存到RAM里 */
	Flash_ReadFloat((float *)gLeftGunPosDatabase, LEFTGUNPOSDATABASE_FLOAT_NUM);
	Flash_ReadFloat((float *)gRightGunPosDatabase, RIGHTGUNPOSDATABASE_FLOAT_NUM);
	Flash_ReadFloat((float *)gUpperGunPosDatabase, UPPERGUNPOSDATABASE_FLOAT_NUM);

	//	Result=(float *)gGunPosDatabaseFromFlash;
	//	countnum = (uint32_t *)(gGunPosDatabaseFromFlash) + len;//类上

	/* 保护Flash数据 */
	Flash_Encryp();
}
///**
//  * @brief  获取从FLASH里得到的数据的指针
//	*
//  * @param  None
//  * @retval flashdata : 指向flash数据的指针
//  */
//uint8_t *GetFlashArr(void)
//{
//	return flashdata;
//}
/**
 * @brief  从FLASH里得到的数据分两个部分，第一部分为数据区，第二部分为
 *		 计数区，该函数用于获得数据区的指针
 * @param  None
 * @retval Result : 指向flash数据区的指针
 */
float *GetResultArr(void)
{
	return Result;
}
/**
 * @brief  从FLASH里得到的数据分两个部分，第一部分为数据区，第二部分为
 *		 计数区，该函数用于获得计数区的指针
 * @param  None
 * @retval Result : 指向flash计数区的指针
 */
uint32_t *GetCountArr(void)
{
	return countnum;
}
/**
 * @brief  获得FLASH是否需要更新的标志位
 *
 * @param  None
 * @retval flag : flash更新的标志位
 */
uint8_t GetFlashUpdataFlag(void)
{
	return flag;
}
/**
 * @brief  设置FLASH更新标志位的值
 *
 * @param  val  : 赋给更新标志位的值
 * @retval None
 */
void  SetFlashUpdateFlag(uint8_t val)
{
	flag=val;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/

