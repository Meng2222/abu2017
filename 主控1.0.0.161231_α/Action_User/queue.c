/**
  *******************************************************************************************************
  * @file	 queue.c
  * @author  ACTION_2017
  * @version V
  * @date	 2017/05/28
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
#include "usart.h"

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
  * @param	None
  * @retval	
  */
uint8_t getCmdQueueElementNum(void)
{
	return gRobot.manualCmdQueue.elementNum;
}

/**
  * @brief  Enter cammand queue
  * @note
  * @param  
  * @retval None
  */
void InCmdQueue(cmd_t inCmd)
{
	//判断队列是否已满，fix me，停止写入
	if(gRobot.manualCmdQueue.tailNum + 1 == gRobot.manualCmdQueue.headNum ||
	    (gRobot.manualCmdQueue.tailNum == CMD_QUEUE_LENGTH - 1 && gRobot.manualCmdQueue.headNum == 0))
	{
		UART5_OUT((uint8_t*)"manualCmdQueue overflow");
	}
	else
	{
		gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum].plantNum = inCmd.plantNum;
		gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum].method   = inCmd.method;
		gRobot.manualCmdQueue.tailNum++;
		gRobot.manualCmdQueue.elementNum++;
		
		//尾位置超过数组长度
		if (gRobot.manualCmdQueue.tailNum >= CMD_QUEUE_LENGTH)
		{
			gRobot.manualCmdQueue.tailNum = 0;
		}
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
	if (gRobot.manualCmdQueue.headNum != gRobot.manualCmdQueue.tailNum)
	{
		outCmd.plantNum = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum;
		outCmd.method   = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method;

		//头位置移动 头的位置移动到了数组外的时候进行了处理	
		gRobot.manualCmdQueue.headNum++;
		if (gRobot.manualCmdQueue.headNum >= CMD_QUEUE_LENGTH)
		{
			gRobot.manualCmdQueue.headNum = 0;
		}
		
		gRobot.manualCmdQueue.elementNum --;
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
/**
  * @brief  DelTailQueue
  * @note	删除队尾
  * @param  
  * @retval None
  */
void DelTailQueue(void)
{	
	//判断是否为空
	if (gRobot.manualCmdQueue.headNum != gRobot.manualCmdQueue.tailNum)
	{	
		gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum].plantNum = INVALID_PLANT_NUMBER;
		gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum].method   = INVALID_SHOOT_METHOD;
		if(gRobot.manualCmdQueue.tailNum != 0)		
		{
			gRobot.manualCmdQueue.tailNum-=1;
		}
		else
		{
			gRobot.manualCmdQueue.tailNum = CMD_QUEUE_LENGTH - 1;
		}

	}
	
}
/**
  * @brief  CheckCmdQueueState
  * @note	检查队列中命令状态
  * @param  
  * @retval None
  */
void CheckCmdQueueState(void)
{	
	gRobot.manualCmdQueue.cmdPlateState = 0;
	gRobot.manualCmdQueue.cmdBallState = 0;
	//判断是否为空
	if (gRobot.manualCmdQueue.headNum != gRobot.manualCmdQueue.tailNum)
	{	
		for(uint8_t i = gRobot.manualCmdQueue.headNum;i<gRobot.manualCmdQueue.headNum + gRobot.manualCmdQueue.elementNum;i++)
		{
			uint8_t counter = 0;
			counter = i%CMD_QUEUE_LENGTH;
			if(gRobot.manualCmdQueue.cmdArr[counter].method%2)
			{
				gRobot.manualCmdQueue.cmdPlateState |= 0x01<<gRobot.manualCmdQueue.cmdArr[counter].plantNum;
			}
			else
			{
				gRobot.manualCmdQueue.cmdBallState |= 0x01<<gRobot.manualCmdQueue.cmdArr[counter].plantNum;			
			}
		}
	}
	
}
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
