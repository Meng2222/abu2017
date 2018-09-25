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
#include "rng.h"
#include "robot.h"
#include "usart.h"
#include "database.h"
/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/

extern robot_t gRobot;

/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

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
		UART5_OUT((uint8_t *)"plant%d method%d\r\n",inCmd.plantNum,inCmd.method);
		//尾位置递增1 循环数组尾位置需要处理
		gRobot.manualCmdQueue.tailNum++;
		if (gRobot.manualCmdQueue.tailNum >= CMD_QUEUE_LENGTH)
		{
			gRobot.manualCmdQueue.tailNum = 0;
		}
		gRobot.manualCmdQueue.elementNum++;
	}
}


/**
  * @brief	OutCmdQueue 出队
  * @note	命令出队 队伍为空时也有处理
  * @param	None
  * @retval 如果队列不为空 则返回队头的元素
  *			如果队列为空 则返回 {INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD}
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
  * @brief	ClearCmdQueue 清空队列
  * @note	
  * @param	None
  * @retval 
  */
void ClearCmdQueue(void)
{
	gRobot.manualCmdQueue.headNum = gRobot.manualCmdQueue.tailNum;
	gRobot.manualCmdQueue.elementNum = 0;
	gRobot.manualCmdQueue.cmdPlateState = 0;
	gRobot.manualCmdQueue.cmdPlateState = 0;
}
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
  * @brief  LeftGunOutQueue 
  * @note	其中 
  * @param  outCmd2 队列中第二个命令
  * @retval outCmd
  */
cmd_t LeftGunOutQueue(void)
{
	cmd_t outCmd = {INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	cmd_t outCmd2 = {INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};

	outCmd = OutCmdQueue();
	//判断是否为空
	if (gRobot.manualCmdQueue.headNum == gRobot.manualCmdQueue.tailNum)
	{
		return outCmd;
	}
	
	outCmd2 = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum];
	if(gRobot.moveBase.actualStopPoint!=SHOOT_POINT_MOVING)
	{
		switch(gRobot.moveBase.actualStopPoint)
		{
			case SHOOT_POINT1:
			{
				if(outCmd.plantNum == PLANT2)
				{
					if(outCmd2.plantNum == PLANT1)
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			case SHOOT_POINT2:
			{
				if((outCmd.plantNum == PLANT4)||(outCmd.plantNum == PLANT5))
				{
					if((outCmd2.plantNum == PLANT1)||(outCmd2.plantNum == PLANT2))
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}

			case SHOOT_POINT3:
			{
				if(outCmd.plantNum == PLANT5)
				{
					if(outCmd2.plantNum == PLANT4)
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			default:
				break;
		}
	}
	else
	{
		switch(gRobot.moveBase.targetPoint)
		{
			case SHOOT_POINT1:
			{
				if(outCmd.plantNum == PLANT2)
				{
					if(outCmd2.plantNum == PLANT1)
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			case SHOOT_POINT2:
			{
				if((outCmd.plantNum == PLANT4)||(outCmd.plantNum == PLANT5))
				{
					if((outCmd2.plantNum == PLANT1)||(outCmd2.plantNum == PLANT2))
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}

			case SHOOT_POINT3:
			{
				if(outCmd.plantNum == PLANT5)
				{
					if(outCmd2.plantNum == PLANT4)
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			default:
				break;
		}		
	}

	return outCmd;
}

/**
  * @brief  RightGunOutQueue 
  * @note	其中 
  * @param  outCmd2 队列中第二个命令
  * @retval outCmd
  */
cmd_t RightGunOutQueue(void)
{
	cmd_t outCmd = {INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};
	cmd_t outCmd2 = {INVALID_PLANT_NUMBER, INVALID_SHOOT_METHOD};

	outCmd = OutCmdQueue();
	//判断是否为空
	if (gRobot.manualCmdQueue.headNum == gRobot.manualCmdQueue.tailNum)
	{
		return outCmd;
	}
	
	outCmd2.plantNum = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum;
	outCmd2.method   = gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method;
	
	if(gRobot.moveBase.actualStopPoint!=SHOOT_POINT_MOVING)
	{
		switch(gRobot.moveBase.actualStopPoint)
		{
			case SHOOT_POINT1:
			{
				if(outCmd.plantNum == PLANT1)
				{
					if(outCmd2.plantNum == PLANT2)
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			case SHOOT_POINT2:
			{
				if((outCmd.plantNum == PLANT1)||(outCmd.plantNum == PLANT2))
				{
					if((outCmd2.plantNum == PLANT4)||(outCmd2.plantNum == PLANT5))
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			case SHOOT_POINT3:
			{
				if(outCmd.plantNum == PLANT4)
				{
					if(outCmd2.plantNum == PLANT5)
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			default:
				break;
		}
	}
	else
	{
		switch(gRobot.moveBase.targetPoint)
		{
			case SHOOT_POINT1:
			{
				if(outCmd.plantNum == PLANT1)
				{
					if(outCmd2.plantNum == PLANT2)
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			case SHOOT_POINT2:
			{
				if((outCmd.plantNum == PLANT1)||(outCmd.plantNum == PLANT2))
				{
					if((outCmd2.plantNum == PLANT4)||(outCmd2.plantNum == PLANT5))
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			case SHOOT_POINT3:
			{
				if(outCmd.plantNum == PLANT4)
				{
					if(outCmd2.plantNum == PLANT5)
					{
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].plantNum = outCmd.plantNum;
						gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.headNum].method   = outCmd.method;
						return outCmd2;
					}
				}
				break;
			}
			default:
				break;
		}	
	}


	return outCmd;
}
/**
  * @brief  DelTailQueue 删除队尾
  * @note	删除队尾的同时 还把被删除元素原来的位置复位为了INVALID变量
  * @param  None
  * @retval None
  */
void DelTailQueue(void)
{
	//判断是否为空
	if (gRobot.manualCmdQueue.headNum != gRobot.manualCmdQueue.tailNum)
	{
		//删除队尾的同时 还把被删除元素原来的位置复位为了INVALID变量
		if(gRobot.manualCmdQueue.tailNum != 0)
		{
			gRobot.manualCmdQueue.tailNum-=1;
		}
		else
		{
			gRobot.manualCmdQueue.tailNum = CMD_QUEUE_LENGTH - 1;
		}
		gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum].plantNum = INVALID_PLANT_NUMBER;
		gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum].method   = INVALID_SHOOT_METHOD;

		gRobot.manualCmdQueue.elementNum--;
	}

}


/**
  * @brief  CheckCmdQueueState 检查队列中命令状态
  * @note	该函数遍历队列 然后修改全局变量gRobot.manualCmdQueue
  * @param  None
  * @retval None
  */
void CheckCmdQueueState(void)
{
	uint8_t tempCmdPlateState = 0u;
	uint8_t tempCmdBallState = 0u;
	//判断是否为空
	if (gRobot.manualCmdQueue.headNum != gRobot.manualCmdQueue.tailNum)
	{
		for(uint8_t i = gRobot.manualCmdQueue.headNum;i<gRobot.manualCmdQueue.headNum + gRobot.manualCmdQueue.elementNum;i++)
		{
			uint8_t counter = 0u;
			//循环队列 对索引号进行处理
			counter = i%CMD_QUEUE_LENGTH;

			//取余得1则为落盘 取余为0为打球
			if(gRobot.manualCmdQueue.cmdArr[counter].method%2)
			{
				tempCmdPlateState |= (0x01<<gRobot.manualCmdQueue.cmdArr[counter].plantNum);
			}
			else
			{
				tempCmdBallState |= (0x01<<gRobot.manualCmdQueue.cmdArr[counter].plantNum);
			}
		}
	}
	//更新全局变量
	gRobot.manualCmdQueue.cmdPlateState = tempCmdPlateState;
	gRobot.manualCmdQueue.cmdBallState = tempCmdPlateState;
}

/**
  * @brief  CheckCmdInQueue 检查队列中是否有命令
  * @note	如果不为空 遍历数组 寻找是否有相同的命令 相同的命令指 plantNum 与 method 均相同
  * @param  checkCmd
  * @retval None
  */
uint8_t CheckCmdInQueue(cmd_t checkCmd)
{
	if (gRobot.manualCmdQueue.headNum != gRobot.manualCmdQueue.tailNum)
	{
		//如果不为空 遍历数组 寻找是否有相同的命令 相同的命令指 plantNum 与 method 均相同
		for(uint8_t i = gRobot.manualCmdQueue.headNum;i<gRobot.manualCmdQueue.headNum + gRobot.manualCmdQueue.elementNum;i++)
		{
			uint8_t counter = 0;
			//循环队列 对索引号进行处理
			counter = i % CMD_QUEUE_LENGTH;
			if(gRobot.manualCmdQueue.cmdArr[counter].method == checkCmd.method && \
				gRobot.manualCmdQueue.cmdArr[counter].plantNum == checkCmd.plantNum)
			{
				return 1;
			}
		}
	}
	//如果 1.队列为空 或者 2.遍历队列以后没有找到相同的命令 则返回0
	return 0;
}

/**
  * @}
  */

/** @defgroup Init_Queue_Operation
  * @brief
  * @{
  */
/**
  * @brief  Init
  * @note	
  * @param  
  * @retval None
  */
void InitQueue(unsigned char stopPoint)
{
	uint8_t rand0 = 0 , rand1 = 0, rand2 = 0;
	uint8_t randCmd;
	cmd_t initCmd = {INVALID_PLANT_NUMBER,INVALID_SHOOT_METHOD};
	
	for(uint8_t i = 5;i>0;i--)
	{
		initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT6; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT6; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT6; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT6; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT6; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
		initCmd.plantNum = PLANT6; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
	}
//	switch(stopPoint)
//	{		
//		//场地左侧发射点
//		case SHOOT_POINT1:
//		{
//			initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//			initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);	
//			break;
//		}
//		//场地中央发射点
//		case SHOOT_POINT2:
//		{

//			RNG_Config();
//			
//			rand0 = ((float)RNG_Get_RandomNum()/(float)0xffffffff)>0.5f;
//			rand1 = ((float)RNG_Get_RandomNum()/(float)0xffffffff)>0.5f;
//			rand2 = ((float)RNG_Get_RandomNum()/(float)0xffffffff)>0.5f;

//			RNG_Cmd(DISABLE);
//			
//			randCmd = rand0;
//			rand1 = rand1;
//			rand2 = rand2;

//			switch(randCmd)
//			{

//				case 0:
//					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//					UpperGunPriority[0] = PLANT1;
//					UpperGunPriority[1] = PLANT5;
////					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//				break;
//				case 1:
//					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//					UpperGunPriority[0] = PLANT5;
//					UpperGunPriority[1] = PLANT1;
////					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);	
//////					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//				break;
////				case 2:
////					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
////				break;
//				default:
////					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT2; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//////					initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT1; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
////					initCmd.plantNum = PLANT4; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);
//				break;
//			}

//			break;
//		}
//		//场地右侧发射点
//		case SHOOT_POINT3:
//		{
//			initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD3; InCmdQueue(initCmd);
//			initCmd.plantNum = PLANT5; initCmd.method = SHOOT_METHOD4; InCmdQueue(initCmd);			

//			break;
//		}
//		default:
//			break;
//	}
}


/**
  * @}
  */
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
