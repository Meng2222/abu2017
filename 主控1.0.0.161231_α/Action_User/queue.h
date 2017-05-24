/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QUEUE_H
#define __QUEUE_H
#include "stdint.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define CMD_QUEUE_LENGTH 50

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */
//命令类型
typedef struct
{
	//柱子号
	uint8_t plantNum;
	
	//动作
	uint8_t method;
}cmd_t;

//命令缓冲器
typedef struct
{
	//命令线性数组
	cmd_t cmdArr[CMD_QUEUE_LENGTH];
	
	//头位置
	uint8_t headNum;
	
	//尾位置
	uint8_t tailNum;	
}cmdBuffer_t;

 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */



/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/

void InCmdQueue(cmd_t inCmd);
cmd_t OutCmdQueue(void);
cmd_t ReplaceHeadQueue(cmd_t inCmd);


#ifdef __cplusplus
}
#endif

#endif /* __QUEUE_H */

