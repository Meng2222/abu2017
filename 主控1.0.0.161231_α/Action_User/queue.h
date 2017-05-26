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
	
	//队列中元素个数
	uint8_t elementNum;
	
	//队列中打球命令状态
	uint8_t cmdBallState;
	
	//队列中落盘命令状态
	uint8_t cmdPlateState;
	
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

/**
  * @brief	
  * @note	
  * @param	None
  * @retval	
  */
uint8_t getCmdQueueElementNum(void);
/**
  * @brief  DelTailQueue
  * @note	删除队尾
  * @param  
  * @retval None
  */
void DelTailQueue(void);
/**
  * @brief  CheckCmdQueueState
  * @note	检查队列中命令状态
  * @param  
  * @retval None
  */
void CheckCmdQueueState(void);
#ifdef __cplusplus
}
#endif

#endif /* __QUEUE_H */

