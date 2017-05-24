/**
  *******************************************************************************************************
  * @file	 queue.c
  * @author  ACTION_2017
  * @version V
  * @date	 2017/05/10
  * @brief   This file contains 
  *
  *******************************************************************************************************
  * @attention
  *
  *
  *******************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------*/

#include "queue.h"
#include "robot.h"

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/

extern robot_t gRobot;

/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

/**
  * @brief  
  * @note
  * @param  
  * @retval None
  */
void InCmdQueue(cmd_t inCmd)
{
	//判断队列是否已满，fix me，停止写入
	if (gRobot.manualCmdQueue.tailNum == gRobot.manualCmdQueue.headNum - 1 ||
	    (gRobot.manualCmdQueue.tailNum == CMD_QUEUE_LENGTH && gRobot.manualCmdQueue.headNum == 0))
	{
		
	}
	
	gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum].plantNum = inCmd.plantNum;
	gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum].method   = inCmd.method;
	gRobot.manualCmdQueue.tailNum++;
	
	//尾位置超过数组长度
	if (gRobot.manualCmdQueue.tailNum >= CMD_QUEUE_LENGTH)
	{
		gRobot.manualCmdQueue.tailNum -= CMD_QUEUE_LENGTH;
	}
	
}
/**
  * @brief  
  * @note
  * @param  
  * @retval None
  */
cmd_t OutCmdQueue(void)
{
	cmd_t outCmd = {INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	
	//判断是否为空
	if (gRobot.manualCmdQueue.headNum == gRobot.manualCmdQueue.tailNum)
	{
		return outCmd; 
	}
	
	outCmd.plantNum = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum;
	outCmd.method   = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method;
	gRobot.manualCmdQueue.headNum++;
	
	//头位置超过数组长度
	if (gRobot.manualCmdQueue.headNum >= CMD_QUEUE_LENGTH)
	{
		gRobot.manualCmdQueue.headNum -= CMD_QUEUE_LENGTH;
	}

	return outCmd;
}
/**
  * @brief  
  * @note
  * @param  
  * @retval None
  */
cmd_t ReplaceHeadQueue(cmd_t inCmd)
{	
	cmd_t outCmd = {INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	
	//判断是否为空
	if (gRobot.manualCmdQueue.headNum == gRobot.manualCmdQueue.tailNum)
	{
		InCmdQueue(inCmd);
		return outCmd;
	}

	outCmd.plantNum = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum;
	outCmd.method   = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method;
	
	gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = inCmd.plantNum;
	gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = inCmd.method;
	
	return outCmd;
}

/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
