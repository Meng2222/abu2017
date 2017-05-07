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
#include "app_cfg.h"
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
#include "gpio.h"
#include "stm32f4xx_exti.h"
#include "robot.h"
extern canMsg_t CAN1Message[50];
extern canMsg_t CAN2Message[50];
extern int CAN1MsgCapacity;
extern int CAN2MsgCapacity;

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
	uint8_t i = 0;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN1, &StdId, buffer, 8);
	
	CAN1Message[CAN1MsgCapacity].canSendId = StdId;
	for(i = 0 ;i < 8 ;i++)
	{
		CAN1Message[CAN1MsgCapacity].message[i] = buffer[i];
	}
	CAN1MsgCapacity++;
	
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

/**
  * @brief  CAN1 SCE interrupt  handler
  * @note   
  * @param  None
  * @retval None
  */
void CAN1_SCE_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	USART_SendData(UART5, 1);	
	USART_SendData(UART5, CAN_GetLastErrorCode(CAN1));
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	BEEP_ON;
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	OSIntExit();
}

/**
  * @brief  CAN2 SCE interrupt  handler
  * @note   
  * @param  None
  * @retval None
  */
void CAN2_SCE_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	USART_SendData(UART5, 2);	
	USART_SendData(UART5, CAN_GetLastErrorCode(CAN2));
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	USART_SendData(UART5, (uint8_t)-100);
	BEEP_ON;
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	OSIntExit();
}

/**
  * @brief  CAN2 receive FIFO0 interrupt request handler
  * @note   
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	uint8_t buffer[8];
	uint32_t StdId=0;
	uint8_t i = 0;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN2, &StdId, buffer, 8);
	CAN2Message[CAN2MsgCapacity].canSendId = StdId;
	for(i = 0 ;i < 8 ;i++)
	{
		CAN2Message[CAN2MsgCapacity].message[i] = buffer[i];
	}
	CAN2MsgCapacity++;
	
	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}


/*************定时器2******start************/
//每1ms调用一次

extern  OS_EVENT 		*CANSendPeriodSem;

extern uint8_t canErrCode;
void TIM2_IRQHandler(void)
{
	#define CAN_SEND_PERIOD_COUNTER 1

	static uint8_t CANSendPeriodCounter = CAN_SEND_PERIOD_COUNTER;

	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();


	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		canErrCode = CAN1 ->ESR/*CAN_GetLastErrorCode(CAN1)*/;

		//更新计数器
		CANSendPeriodCounter--;
		if (CANSendPeriodCounter == 0)
		{
			OSSemPost(CANSendPeriodSem);
			CANSendPeriodCounter = CAN_SEND_PERIOD_COUNTER;
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
	if(CoreDebug->DHCSR&1)
	{
		__breakpoint(0);
	}
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

