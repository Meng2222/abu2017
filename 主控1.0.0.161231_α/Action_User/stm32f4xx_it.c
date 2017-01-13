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
#include "adc.h"
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "robs.h"
#include "walk.h"
#include "String.h"
#include "stm32f4xx_dma.h"
#include "gpio.h"
#include "stm32f4xx_exti.h"
#include "elmo.h"
#include "action_math.h"
#include "encoder.h"
/************************************************************/
/****************驱动器CAN1接口模块****start******************/
union Position
{
	uint8_t  Data8[2];
	uint16_t Data16;
}pos;
union Vell
{
	uint8_t  Data8[2];
	int16_t  Data16;
}vell;
uint16_t encoder_right,encoder_left;
int16_t vell_right,vell_left;


void CAN1_RX0_IRQHandler(void)
{
	
	OS_CPU_SR  cpu_sr;
	static uint8_t buffer[8];
	static uint32_t StdId=0;

	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN1, &StdId,buffer,8);

	if(StdId == 0x16)	//
	{
		pos.Data8[0]=buffer[0];
		pos.Data8[1]=buffer[1];
		encoder_right=pos.Data16;
		
		vell.Data8[0]=buffer[2];
		vell.Data8[1]=buffer[3]; 
		vell_right=vell.Data16;	
		SetEncVel(0,vell_right);
	}
	
	if(StdId == 0x18)	//
	{
		pos.Data8[0]=buffer[0];
		pos.Data8[1]=buffer[1];
		encoder_right=pos.Data16;
		
		vell.Data8[0]=buffer[2];
		vell.Data8[1]=buffer[3]; 
		vell_left=vell.Data16;		
		SetEncVel(1,vell_left);
	}
	
	CAN_ClearFlag(CAN1,CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1,CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1,CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1,CAN_FLAG_LEC);
	
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV1);
	OSIntExit();
}
/****************驱动器CAN1接口模块****end******************/
/************************************************************/

/*************定时器2******start************/
//每1ms调用一次  用于读取编码器的值和计算坐标

extern  OS_EVENT 		*PeriodSem;
//int posx,posy;
void TIM2_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
//	static uint8_t countss=0;
//	static uint8_t Count=0;
//	float pos_x;
//  float pos_y;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
	{

		OSSemPost(PeriodSem);
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   }
	 OSIntExit();
}


//定时器1  左编码器中断
void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM1, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	OSIntExit();
}

//定时器8  右编码器中断
void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM8, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

/********************************************************/
/*****************普通定时TIM5*****Start*****************/
void TIM5_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM5, TIM_IT_Update)==SET)    
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
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET)    
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	
	}
	OSIntExit();
}



//定时器4  
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
float roll[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float pitch[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float yaw[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int32_t speed1[7] = {0, 0, 0, 0, 0, 0, 0};
int32_t speed2[7] = {0, 0, 0, 0, 0, 0, 0};

extern uint8_t launcherStatus;

void USART1_IRQHandler(void)
{	 
	static int	status = 0;
	static uint8_t id = 0xff;
	static int extraCounter = 0;                  //count extra byte
	
	static union
	{
		uint8_t data8[4];
		int32_t data32;
		float   dataf;
	}dataConvert;
	static int ACCTid = 0;
	float temAngle = 0.0f;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)   
	{
		uint8_t ch;
		USART_ClearITPendingBit( USART1, USART_IT_RXNE);
		ch = USART_ReceiveData(USART1);
		USART_SendData(USART1, ch);
		
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
					status++;                  //ACPC + [id] + data[4] + extra[3]
				else if (ch == 'C')
					status += 10;              //ACCT + [id] + extra[7]
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
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
				if(status < 9)
					dataConvert.data8[status - 5] = ch;
				status++;
				break;
			case 11:
				switch(id % 5)
				{
					case 0:
						roll[id / 5] = dataConvert.dataf;
						temAngle = (45.0f-dataConvert.dataf);
						if(temAngle < 0.0f)		temAngle = 0.0f;
						if(temAngle > 45.0f)		temAngle = 45.0f;
						PosCrl(7,0,(int32_t)(temAngle * 56.8889f));
						break;
					case 1:
						pitch[id / 5] = dataConvert.dataf;
						temAngle = dataConvert.dataf + 6.0f;
						if(temAngle < 0.0f)		temAngle = 0.0f;
						if(temAngle > 36.0f)		temAngle = 36.0f;
						PosCrl(8,0,(int32_t)(temAngle * 76.8f));
						break;
					case 2:
						yaw[id / 5] = dataConvert.dataf;
						temAngle = 45.0f - dataConvert.dataf;
						if(temAngle < 0.0f)		temAngle = 0.0f;
						if(temAngle > 90.0f)		temAngle = 90.0f;
						PosCrl(6,0,(int32_t)(temAngle * 62.57778f));
						break;
					case 3:
						speed1[id / 5] = dataConvert.data32;
						VelCrl(11,4096*dataConvert.data32);
						break;
					case 4:
						speed2[id / 5] = dataConvert.data32;
						VelCrl(10,4096*dataConvert.data32);
						break;
					default:
						id = 0xff;
						break;
				}
				status = 0;
				id = 0xff;
				break;
			case 12:                              /*ACCT begin from here*/
				if (ch == 'T')
					status++;
				else
					status = 0;
				break;
			case 13:
				id = ch;
				status++;
				break;
			case 14:
				extraCounter++;
				if (extraCounter == 6)
				{
					status++;
					extraCounter = 0;
				}
				break;
			default:
				ACCTid = id;
				switch(ACCTid)
				{
					case 1:
						if (launcherStatus == 0)
					  {
							PosCrl(9,1,2048); 
							launcherStatus = 1;
						}
						ACCTid = 0;
						break;
  				case 2:
//						PosCrl(9,1,2048);
						ACCTid = 0;
						break;
				}
				status = 0;
				id = 0xff;
				break;					
		}
	 }
	OSIntExit();
}

/****************陀螺仪串口接受中断****start****************/

//static float pos_x=0;
//static float pos_y=0;
//static float zangle=0;
//static float xangle=0;
//static float yangle=0;
//static float w_z=0;
//void USART1_IRQHandler(void)
//{	 
//	static uint8_t ch;
//	static union
//  {
//	 uint8_t data[24];
//	 float ActVal[6];
//  }posture;
//	static uint8_t count=0;
//	static uint8_t i=0;
//	OS_CPU_SR  cpu_sr;
//	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();

//	if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
//	{
//		USART_ClearITPendingBit( USART3,USART_IT_RXNE);
//		ch=USART_ReceiveData(USART3);
//		 switch(count)
//		 {
//			 case 0:
//				 if(ch==0x0d)
//					 count++;
//				 else
//					 count=0;
//				 break;
//				 
//			 case 1:
//				 if(ch==0x0a)
//				 {
//					 i=0;
//					 count++;
//				 }
//				 else if(ch==0x0d);
//				 else
//					 count=0;
//				 break;
//				 
//			 case 2:
//				 posture.data[i]=ch;
//			   i++;
//			   if(i>=24)
//				 {
//					 i=0;
//					 count++;
//				 }
//				 break;
//				 
//			 case 3:
//				 if(ch==0x0a)
//					 count++;
//				 else
//					 count=0;
//				 break;
//				 
//			 case 4:
//				 if(ch==0x0d)
//				 {
//  				 zangle=posture.ActVal[0];
//	  		   xangle=posture.ActVal[1];
//		  	   yangle=posture.ActVal[2];
//			     pos_x =posture.ActVal[3];
//			     pos_y =posture.ActVal[4];
//			     w_z   =posture.ActVal[5];
//					 
//					 xangle=xangle;
//					 yangle=yangle;
//					 pos_x =pos_x ;
//					 pos_y =pos_y ;
//					 w_z   =w_z;
//					 
//					 setAngle(zangle);
//				 }
//			   count=0;
//				 break;
//			 
//			 default:
//				 count=0;
//			   break;		 
//		 }
//		 
//		 
//	 }
//	OSIntExit();
//}

/******************蓝牙串口****************/
void UART4_IRQHandler(void)
{
  

	  
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	 if(USART_GetITStatus(UART4, USART_IT_RXNE)==SET)   
	 {
			USART_ClearITPendingBit( UART4,USART_IT_RXNE);
	 }
	 
   OSIntExit();
}

//调试串口中断
static uint8_t Cmd[5];
void UART5_IRQHandler(void)
{
	static int Uart5Status = 0;
	static uint8_t i = 0;
	uint8_t Msg = 0;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( UART5,USART_IT_RXNE);
		Msg = USART_ReceiveData(UART5);
		switch(Uart5Status)
		{
			case 0:
				if(Msg == 0x43)		//C
					Uart5Status = 1;		
				break;
			case 1:
				if(Msg == 0x4D)							//M	
					Uart5Status = 2;		
				else if(Msg == 0x43)				//C	
					Uart5Status = 1;
				else 
					Uart5Status = 0;
				break;
			case 2:
				Cmd[i] = Msg;
				i++;
				if(i >= 5){
					i = 0;
					Uart5Status = 3;
				}
				break;
			case 3:
				if(Msg == 0x4D)							//M	
					Uart5Status = 4;
				else
					Uart5Status = 0;
				break;
			case 4:
				if(Msg == 0x43)							//命令处理
				{
					if(Cmd[2] == 'U')
						IncSpeed(Cmd[0] - '0');
					if(Cmd[2] == 'D')
						DecSpeed(Cmd[0] - '0');
				}
				Uart5Status = 0;
				break;
			default:
				Uart5Status = 0;
				break;
		}
		
	}
	OSIntExit();	 
	 
}


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

