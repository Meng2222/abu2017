/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QUEUE_H
#define __QUEUE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

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
/* Exported functions --------------------------------------------------------*/

/** @defgroup Basic_Queue_Operatiion
  * @brief	对队列的基础操作
  * @{
  */

/**
  * @brief  Enter cammand queue 入队
  * @note	入队 如果队列已满时将 将通过串口向wifi模块发出错误信息
  * @param  inCmd 将要入队的命令
  * @retval None
  */
void InCmdQueue(cmd_t inCmd);

/**
  * @brief	OutCmdQueue 出队
  * @note	命令出队 队伍为空时也有处理
  * @param	None
  * @retval 如果队列不为空 则返回队头的元素
  *			如果队列为空 则返回 {INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD}
  */
cmd_t OutCmdQueue(void);
/**
  * @brief	ClearCmdQueue 清空队列
  * @note	
  * @param	None
  * @retval 
  */
void ClearCmdQueue(void);
/**
  * @}
  */


/** @defgroup Alternative_Queue_Operation
  * @brief
  * @{
  */

/**
  * @brief  ReplaceHeadQueue 替换队列头处(即将出队)的元素
  * @note	其中 如果队列为空时使用到了InCmdQueue()函数
  * @param  inCmd 用来替换的命令
  * @retval None
  */
cmd_t ReplaceHeadQueue(cmd_t inCmd);

/**
  * @brief  DelTailQueue 删除队尾
  * @note	删除队尾的同时 还把被删除元素原来的位置复位为了INVALID变量
  * @param  None
  * @retval None
  */
void DelTailQueue(void);

/**
  * @brief  CheckCmdQueueState 检查队列中命令状态
  * @note	该函数遍历队列 然后修改全局变量gRobot.manualCmdQueue
  * @param  None
  * @retval None
  */
void CheckCmdQueueState(void);

/**
  * @brief  CheckCmdInQueue 检查队列中是否有命令
  * @note	如果不为空 遍历数组 寻找是否有相同的命令 相同的命令指 plantNum 与 method 均相同
  * @param  checkCmd
  * @retval None
  */
uint8_t CheckCmdInQueue(cmd_t checkCmd);


void InitQueue(uint8_t stopPoint);

cmd_t LeftGunOutQueue(void);

cmd_t RightGunOutQueue(void);


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __QUEUE_H */

