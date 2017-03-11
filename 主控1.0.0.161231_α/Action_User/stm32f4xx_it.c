/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include  <ucos_ii.h>
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "math.h"
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "String.h"
#include "stm32f4xx_dma.h"
#include "gpio.h"
#include "stm32f4xx_exti.h"
#include "elmo.h"
#include "gasvalvecontrol.h"
#include "movebase.h"
#include "robot.h"

//用来处理CAN接收数据
union MSG
{
	uint8_t data8[8];
	int data32[2];
	float dataf[2];
}msg;

//声明外部变量
extern robot_t gRobot;

void CAN1_RX0_IRQHandler(void)
{
	
	OS_CPU_SR  cpu_sr;
	uint8_t buffer[8];
	uint32_t StdId=0;
	uint8_t canNodeId = 0;
	int32_t i = 0;

	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN1, &StdId, buffer, 8);
	canNodeId = StdId - SDO_RESPONSE_COB_ID_BASE;
	
	if(canNodeId==LEFT_WHEEL_ID || canNodeId==FORWARD_WHEEL_ID || canNodeId==BACKWARD_WHEEL_ID)     //get speed value
	{
		//fix me, if length not 8
		for(i = 0; i < 8; i++)
			msg.data8[i] = buffer[i];

		if(msg.data32[0] == 0x00005856)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				//下面代码除以100为了将m/s转换为0.1m/s，fix me
				gRobot.moveBase.actualSpeed.leftWheelSpeed =(Pulse2Vel(msg.data32[1]))/100;
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.actualSpeed.forwardWheelSpeed =(Pulse2Vel(msg.data32[1]))/100;

			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.actualSpeed.backwardWheelSpeed =(Pulse2Vel(msg.data32[1]))/100;
			}
		}
		
		if(msg.data32[0] == 0x80005149)
		{
			//msg.dataf[1]是单位为安培的浮点数
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				gRobot.moveBase.acturalCurrent.leftWheelCurrent = (msg.dataf[1]);
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.acturalCurrent.forwardWheelCurrent = (msg.dataf[1]);
			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.acturalCurrent.backwardWheelCurrent = (msg.dataf[1]);
			}
		}
		if(msg.data32[0] == 0x00014954)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				//msg.data32[1]为单位为摄氏度的整数，范围是25~135，参考almo手册
				gRobot.moveBase.driverTemperature.leftWheelDriverTemperature = (msg.data32[1]);
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverTemperature.forwardWheelDrvierTemperature = (msg.data32[1]);
			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverTemperature.backwardWheelDriverTemperature = (msg.data32[1]);
			}

		}
	}
	//fix me,对于0x28x，可以统一处理，并不需要这么多复杂的判断
	if(canNodeId == LEFT_GUN_PITCH_ID || canNodeId == LEFT_GUN_ROLL_ID || canNodeId == LEFT_GUN_YAW_ID || \
		canNodeId == LEFT_GUN_LEFT_ID || canNodeId == LEFT_GUN_RIGHT_ID||canNodeId == RIGHT_GUN_PITCH_ID|| \
		canNodeId == RIGHT_GUN_ROLL_ID || canNodeId == RIGHT_GUN_YAW_ID || canNodeId == RIGHT_GUN_LEFT_ID|| \
		canNodeId ==RIGHT_GUN_RIGHT_ID)
	{
		for(i = 0; i < 8; i++)
		{
			msg.data8[i] = buffer[i];
		}

		if(msg.data32[0] == 0x40005856)
		{
			if(canNodeId == LEFT_GUN_LEFT_ID) 
			{
				gRobot.leftGun.actualPose.speed1 = LeftGunLeftSpeedInverseTransform(msg.data32[1]);
			}
			if(canNodeId == LEFT_GUN_RIGHT_ID) 
			{
				gRobot.leftGun.actualPose.speed2 = LeftGunRightSpeedInverseTransform(msg.data32[1]);
			}
			if(canNodeId == RIGHT_GUN_LEFT_ID) 
			{
				gRobot.rightGun.actualPose.speed1 = RightGunLeftSpeedInverseTransform(msg.data32[1]);
			}
			if(canNodeId == RIGHT_GUN_RIGHT_ID) 
			{
				gRobot.rightGun.actualPose.speed2 = RightGunRightSpeedInverseTransform(msg.data32[1]);
			}		
		}
		if(msg.data32[0] == 0x00005850)
		{
			if(canNodeId == LEFT_GUN_PITCH_ID) 
			{
				gRobot.leftGun.actualPose.pitch = LeftGunPitchInverseTransform(msg.data32[1]);    //俯仰
			}
			if(canNodeId == LEFT_GUN_ROLL_ID) 
			{
				gRobot.leftGun.actualPose.roll = LeftGunRollInverseTransform(msg.data32[1]);    //横滚
			}
			if(canNodeId == LEFT_GUN_YAW_ID) 
			{
				gRobot.leftGun.actualPose.yaw = LeftGunYawInverseTransform(msg.data32[1]);    //航向
			}
			if(canNodeId == RIGHT_GUN_PITCH_ID) 
			{
				gRobot.rightGun.actualPose.pitch = RightGunPitchInverseTransform(msg.data32[1]);    //俯仰
			}
			if(canNodeId == RIGHT_GUN_ROLL_ID) 
			{
				gRobot.rightGun.actualPose.roll = RightGunRollInverseTransform(msg.data32[1]);    //横滚
			}
			if(canNodeId == RIGHT_GUN_YAW_ID) 
			{
				gRobot.rightGun.actualPose.yaw = RightGunYawInverseTransform(msg.data32[1]);    //航向
			}
			if(canNodeId == UPPER_GUN_PITCH_ID) 
			{
				gRobot.upperGun.actualPose.pitch = UpperGunPitchInverseTransform(msg.data32[1]);    //俯仰
			}
			if(canNodeId == UPPER_GUN_YAW_ID) 
			{
				gRobot.upperGun.actualPose.yaw = UpperGunYawInverseTransform(msg.data32[1]);    //航向
			}
		}
	}
	
	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}

/*************定时器2******start************/
//每1ms调用一次

extern  OS_EVENT 		*PeriodSem;

void TIM2_IRQHandler(void)
{
	#define PERIOD_COUNTER 10
	//用来计数10次，产生10ms的定时器
	static uint8_t periodCounter = PERIOD_COUNTER;
	
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		//更新10ms计数器
		periodCounter--;
		if (periodCounter == 0)
		{
			OSSemPost(PeriodSem);
			periodCounter = PERIOD_COUNTER;
		}
		
		
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	OSIntExit();
}


void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)    
	{                                                
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	OSIntExit();
}


void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)    
	{                                                
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)    
	{              
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM3_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)    
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	OSIntExit();
}
 
void TIM4_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
	{                                  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	OSIntExit();
}

/*************************与平板通信**************************/
//fix me 
/*
*ACPC + [数据类型类型] + [枪号] + [数据]
*数据类型：roll(0)/patch(1)/yaw(2)/speed1(3)/speed2(4)
*数据：yaw data/patch data/roll data/speed1 data/speed2 data
*枪号：0左枪 1右枪 2上枪
*/
void UART4_IRQHandler(void)
{	 
	static int	status = 0;
	static uint8_t id = 0xff ,id2 = 0xff;

	static union
	{
		uint8_t data8[4];
		int32_t data32;
		float   dataf;
	}data;
	
	float targetAngle = 0.0f;
	
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET)   
	{
		uint8_t ch;
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		ch = USART_ReceiveData(UART4);
		
		USART_SendData(UART4, ch);
		
		switch (status)
		{
			case 0:                       
				if (ch == 'A')        
					status++;
				break;
			case 1:
				if (ch == 'C')
					status++;
				else
					status = 0;
				break;
			case 2: 
				if (ch == 'P')
					status++;                  //ACPC + [id1] + [id2] + data[4]
				else if (ch == 'C')
					status += 10;              //ACCT + [id]
				else
					status = 0;
				break;
			case 3:                            /*ACPC begin from here*/  
				if (ch == 'C')
					status++;
				else
					status = 0;
				break;	
			case 4:
				id = ch;
				status++;
				break;
			case 5:
				id2 = ch;				//左   打球0 打盘3 扔6  右 打球1 打盘4 扔7  上 打球2 打盘5 扔8
				status++;
			break;
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
				if(status < 10)
					data.data8[status - 6] = ch;
				status++;
				break;
			case 11:
				switch(id % 5)
				{
					//roll
					case 0:
 						targetAngle = data.dataf;

						switch(id2 % 3)
						{	
							case 0:
							//左枪
							gRobot.leftGun.targetPose.roll = targetAngle;
							//1~7表示7个着陆台，转换为0~6
							gRobot.leftGun.targetPlant = id/5;
							break;
							case 1:
							gRobot.rightGun.targetPose.roll = targetAngle;
							//1~7表示7个着陆台，转换为0~6
							gRobot.rightGun.targetPlant = id/5;
 							break;
							case 2:

 							break;
							default:
								id2 = 0xff;
							break;
						}
						break;
					case 1:
						//pitch
 						targetAngle = data.dataf;
 
						switch(id2 % 3)
						{	
							case 0:
							gRobot.leftGun.targetPose.pitch = targetAngle;
							break;
							case 1:
							gRobot.rightGun.targetPose.pitch = targetAngle;
							break;
 							case 2:
							gRobot.upperGun.targetPose.pitch = targetAngle;
							break;
							default:
								id2=0xff;
							break;
						}
						break;
					case 2:
						//yaw
 						targetAngle = data.dataf;
 
						switch(id2 % 3)
						{	
							case 0:
							gRobot.leftGun.targetPose.yaw = targetAngle;
							break;
							case 1:
							gRobot.rightGun.targetPose.yaw = targetAngle;
 							break;
							case 2:
							gRobot.upperGun.targetPose.yaw = targetAngle;
							break;
							default:
								id2 = 0xff;
							break;
						}
						break;
					case 3:
 						switch(id2 % 3)
						{
							case 0:
								gRobot.leftGun.targetPose.speed1 = data.data32;

							break;
							case 1:
								gRobot.rightGun.targetPose.speed1 = data.data32;
							break;
							case 2:
								gRobot.upperGun.targetPose.speed1 = data.data32;
							break;
							default:
								id2 = 0xff;
							break;
						}
						break;
					case 4:
 						switch(id2 % 3)
						{
							case 0:
								gRobot.leftGun.targetPose.speed2 = data.data32;
							break;
							case 1:
								gRobot.rightGun.targetPose.speed2 = data.data32;
							break;
							case 2:
							break;
							default:
								id2 = 0xff;
							break;
						}
						break;
					default:
						id = 0xff;
						break;
				}
				status = 0;
				id = 0xff;
				id2 = 0xff;
				break;
			case 12:                              /*ACCT begin from here*/
				if (ch == 'T')
					status++;
				else
					status = 0;
				break;
			case 13:
				id = ch;
				switch(id)
				{
					case 1:
						//通知左枪开枪任务执行开枪动作
						gRobot.leftGun.shoot = GUN_START_SHOOT;
						break;
					case 2:
						//通知右枪开枪任务执行开枪动作
						gRobot.rightGun.shoot = GUN_START_SHOOT;
						break;
					case 3:
						//通知上面枪开枪任务执行开枪动作
						gRobot.upperGun.shoot = GUN_START_SHOOT;
						break;
				}
				status=0;
			break;
			default:
				status = 0;
				id = 0xff;
				break;					
		}
	 }
	OSIntExit();
}

/****************陀螺仪串口接受中断********************/

void USART3_IRQHandler(void)       //更新频率200Hz
{	 
	static uint8_t ch;
	static union
    {
		uint8_t data[24];
		float ActVal[6];
    }posture;
	static uint8_t count = 0;
	static uint8_t i = 0;
	
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( USART3,USART_IT_RXNE);
		ch=USART_ReceiveData(USART3);
		switch(count)
		{
			case 0:
				if(ch == 0x0d)
					count++;
				else
					count = 0;
				break;
			 
			case 1:
				if(ch == 0x0a)
				{
					i = 0;
					count++;
				}
				else if(ch == 0x0d);
				else
					count = 0;
				break;
			 
			case 2:
				posture.data[i] = ch;
				i++;
				if(i >= 24)
				{
					i = 0;
					count++;
				}
				break;
			 
			case 3:
				if(ch == 0x0a)
					count++;
				else
					count=0;
				break;
			 
			case 4:
				if(ch == 0x0d)
				{
					gRobot.moveBase.actualAngle = posture.ActVal[0];
					posture.ActVal[1]           = posture.ActVal[1];
					posture.ActVal[2]           = posture.ActVal[2];
					gRobot.moveBase.actualXPos  = posture.ActVal[3];
					gRobot.moveBase.actualYPos  = posture.ActVal[4];
					posture.ActVal[5]           = posture.ActVal[5];
				}
				count = 0;
				break;

			default:
				count = 0;
				break;		 
		}	 	 
	}
	else
	{
			USART_ClearITPendingBit( USART3,USART_IT_PE);
			USART_ClearITPendingBit( USART3,USART_IT_TXE);
			USART_ClearITPendingBit( USART3,USART_IT_TC);
			USART_ClearITPendingBit( USART3,USART_IT_ORE_RX);
			USART_ClearITPendingBit( USART3,USART_IT_IDLE);
			USART_ClearITPendingBit( USART3,USART_IT_LBD);
			USART_ClearITPendingBit( USART3,USART_IT_CTS);
			USART_ClearITPendingBit( USART3,USART_IT_ERR);
			USART_ClearITPendingBit( USART3,USART_IT_ORE_ER);
			USART_ClearITPendingBit( USART3,USART_IT_NE);
			USART_ClearITPendingBit( USART3,USART_IT_FE);
			USART_ReceiveData(USART3);
	}	
	OSIntExit();
}

/*
* name:
* function:
* params:
* notes:
*/
u8 receive_data=0;
void USART6_IRQHandler(void)        
{	 
#define HEADER1 0x80
#define HEADER2 0x80
	
#define HEADER_STATE1 0
#define HEADER_STATE2 1
#define DATA_STATE 2

	static uint8_t data = 0;
 	static int state = 0;

	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET)   
	{
		USART_ClearITPendingBit( USART6,USART_IT_RXNE);
		data = USART_ReceiveData(USART6);	

		switch(state)
		{
			case HEADER_STATE1:
				if(data == HEADER1)
				{
					state++;
				}
				break;
			case HEADER_STATE2:
				if(data == HEADER2)
				{
					state++;
				}
				else
				{						
					state=0;
				}
				break;
			case DATA_STATE: 
				//更新7号着陆台飞盘位置, fix me
				receive_data=data;
				gRobot.upperGun.targetZone = data;
				state = 0;
				break;
			default:
				break;
		}	
	}
	
	OSIntExit();
}
/*********************************WIFI*************************/
/**************************************************************/

//通过判断接收连续2个字符之间的时间差不大于100ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过100ms,则认为不是1次连续数据.也就是超过100ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
void UART5_IRQHandler(void)
{
	u8 res;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)   
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		res = USART_ReceiveData(UART5);		
		if((USART5_RX_STA&(1 << 15)) == 0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(USART5_RX_STA<USART5_MAX_RECV_LEN)		//还可以接收数据
			{
				TIM_SetCounter(TIM7, 0);//计数器清空        				 
				if(USART5_RX_STA == 0)		
					TIM_Cmd(TIM7, ENABLE);  //使能定时器7 
				USART5_RX_BUF[USART5_RX_STA++] = res;		//记录接收到的值	 
			}
			else 
			{
				USART5_RX_STA |= 1 << 15;					//强制标记接收完成
			} 
		}
	}
	OSIntExit();		
}

//定时器7中断服务程序		    
void TIM7_IRQHandler(void)
{ 	
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//是更新中断
	{	 			   
		USART5_RX_STA |= 1 << 15;	//标记接收完成
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);  //清除TIM7更新中断标志    
		TIM_Cmd(TIM7, DISABLE);  //关闭TIM7 
	}	    
}

/*********************************WIFI*************************/
/**************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   while (1)
   {
   }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
 
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

