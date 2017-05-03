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
#include "stm32f4xx_dma.h"
#include "gpio.h"
#include "stm32f4xx_exti.h"
#include "elmo.h"
#include "gasvalvecontrol.h"
#include "movebase.h"
#include "robot.h"
#include "movebase2.h"
#include "dma.h"

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
		
		if(msg.data32[0] == 0x0000464D)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				//下面代码除以100为了将m/s转换为0.1m/s，fix me
				gRobot.moveBase.motorFailure.leftMotorFailure.motorFailure =(msg.data32[1]);
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.motorFailure.forwardMotorFailure.motorFailure =(msg.data32[1]);

			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.motorFailure.backwardMotorFailure.motorFailure =(msg.data32[1]);
			}
		}
		
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
		if(msg.data32[0] == 0x0000434C)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				gRobot.moveBase.driverCurrentLimitFlag.leftWheelDriverFlag = (msg.data32[1]);
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverCurrentLimitFlag.forwardWheelDriverFlag = (msg.data32[1]);
			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverCurrentLimitFlag.backwardWheelDriverFlag = (msg.data32[1]);
			}
		}
		if(msg.data32[0] == 0x00025644)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				gRobot.moveBase.driverCommandVelocity.leftDriverCommandVelocity = Pulse2Vel(msg.data32[1])/100.0f;
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverCommandVelocity.forwardDriverCommandVelocity = Pulse2Vel(msg.data32[1])/100.0f;
			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverCommandVelocity.backwardDriverCommandVelocity = Pulse2Vel(msg.data32[1])/100.0f;
			}
		}
		if(msg.data32[0] == 0x0000564A)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				gRobot.moveBase.driverJoggingVelocity.leftDriverJoggingVelocity = Pulse2Vel(msg.data32[1])/100.0f;
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverJoggingVelocity.forwardDriverJoggingVelocity = Pulse2Vel(msg.data32[1])/100.0f;
			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverJoggingVelocity.backwardDriverJoggingVelocity = Pulse2Vel(msg.data32[1])/100.0f;
			}
		}

	}
		//枪上传送带速度
	if(canNodeId == LEFT_GUN_LEFT_ID ||canNodeId == LEFT_GUN_RIGHT_ID || \
		canNodeId == RIGHT_GUN_LEFT_ID ||canNodeId == RIGHT_GUN_RIGHT_ID  || \
		canNodeId == UPPER_GUN_LEFT_ID)
	{
		for(i = 0; i < 8; i++)
		{
			msg.data8[i] = buffer[i];
		}
		if(msg.data32[0] == 0x00005856)
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
			if(canNodeId == UPPER_GUN_LEFT_ID) 
			{
				gRobot.upperGun.actualPose.speed1 = UpperGunLeftSpeedInverseTransform(msg.data32[1]);
			}
		}
	}

	//fix me,对于0x28x，可以统一处理，并不需要这么多复杂的判断
	if(canNodeId == LEFT_GUN_PITCH_ID || canNodeId == LEFT_GUN_ROLL_ID || canNodeId == LEFT_GUN_YAW_ID || \
		canNodeId == LEFT_GUN_LEFT_ID || canNodeId == LEFT_GUN_RIGHT_ID||canNodeId == RIGHT_GUN_PITCH_ID|| \
		canNodeId == RIGHT_GUN_ROLL_ID || canNodeId == RIGHT_GUN_YAW_ID || canNodeId == RIGHT_GUN_LEFT_ID|| \
		canNodeId ==RIGHT_GUN_RIGHT_ID || canNodeId == UPPER_GUN_PITCH_ID || canNodeId == UPPER_GUN_YAW_ID)
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
	USART_SendData(UART5, (uint8_t)111);		
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
	USART_SendData(UART5, (uint8_t)111);	
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
	uint8_t canNodeId = 0;
	int32_t i = 0;

	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN2, &StdId, buffer, 8);
	canNodeId = StdId - SDO_RESPONSE_COB_ID_BASE;
	//读取电机表面温度，一个uint8_t数
	if(canNodeId == LEFT_WHEEL_TEMPERATURE_ID)
	{		
		gRobot.moveBase.motorTemperature.leftWheelMotorTemperature = buffer[0];
	}
	if(canNodeId == FORWARD_WHEEL_TEMPERATURE_ID)
	{		
		gRobot.moveBase.motorTemperature.forwardWheelMotorTemperature = buffer[0];
	}	
	if(canNodeId == BACKWARD_WHEEL_TEMPERATURE_ID)
	{		
		gRobot.moveBase.motorTemperature.backwardWheelMotorTemperature = buffer[0];
	}	
	
	if(canNodeId==LEFT_WHEEL_ID || canNodeId==FORWARD_WHEEL_ID || canNodeId==BACKWARD_WHEEL_ID)     //get speed value
	{
		//fix me, if length not 8
		for(i = 0; i < 8; i++)
			msg.data8[i] = buffer[i];
		if(msg.data32[0] == 0x0000464D)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{

				gRobot.moveBase.motorFailure.leftMotorFailure.motorFailure =(msg.data32[1]);
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.motorFailure.forwardMotorFailure.motorFailure =(msg.data32[1]);

			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.motorFailure.backwardMotorFailure.motorFailure =(msg.data32[1]);
			}
		}
		if(msg.data32[0] == 0x00005856)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				//下面代码除以100为了将m/s转换为0.1m/s，fix me
				gRobot.moveBase.actualSpeed.leftWheelSpeed =((msg.data32[1]))/100;
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.actualSpeed.forwardWheelSpeed =((msg.data32[1]))/100;

			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.actualSpeed.backwardWheelSpeed =((msg.data32[1]))/100;
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
				//msg.data32[1]为单位为摄氏度的整数，范围是25~135，参考elmo手册
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
		if(msg.data32[0] == 0x0000434C)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				gRobot.moveBase.driverCurrentLimitFlag.leftWheelDriverFlag = (msg.data32[1]);
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverCurrentLimitFlag.forwardWheelDriverFlag = (msg.data32[1]);
			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverCurrentLimitFlag.backwardWheelDriverFlag = (msg.data32[1]);
			}
		}
		if(msg.data32[0] == 0x00025644)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				gRobot.moveBase.driverCommandVelocity.leftDriverCommandVelocity =(msg.data32[1])/100.0f;
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverCommandVelocity.forwardDriverCommandVelocity = (msg.data32[1])/100.0f;
			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverCommandVelocity.backwardDriverCommandVelocity = Pulse2Vel(msg.data32[1])/100.0f;
			}
		}
		if(msg.data32[0] == 0x0000564A)
		{
			if(canNodeId == LEFT_WHEEL_ID) 
			{
				gRobot.moveBase.driverJoggingVelocity.leftDriverJoggingVelocity = (msg.data32[1])/100.0f;
			}
			if(canNodeId == FORWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverJoggingVelocity.forwardDriverJoggingVelocity = (msg.data32[1])/100.0f;
			}
			if(canNodeId == BACKWARD_WHEEL_ID) 
			{
				gRobot.moveBase.driverJoggingVelocity.backwardDriverJoggingVelocity = (msg.data32[1])/100.0f;
			}
		}

	}
	//fix me,对于0x28x，可以统一处理，并不需要这么多复杂的判断
	if(canNodeId == LEFT_GUN_PITCH_ID || canNodeId == LEFT_GUN_ROLL_ID || canNodeId == LEFT_GUN_YAW_ID || \
		canNodeId == LEFT_GUN_LEFT_ID || canNodeId == LEFT_GUN_RIGHT_ID||canNodeId == RIGHT_GUN_PITCH_ID|| \
		canNodeId == RIGHT_GUN_ROLL_ID || canNodeId == RIGHT_GUN_YAW_ID || canNodeId == RIGHT_GUN_LEFT_ID|| \
		canNodeId ==RIGHT_GUN_RIGHT_ID || canNodeId == UPPER_GUN_PITCH_ID || canNodeId == UPPER_GUN_YAW_ID)
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

extern  OS_EVENT 		*PeriodSem;
extern  OS_EVENT 		*DebugPeriodSem;

extern float moveTimer;
extern uint8_t moveTimFlag;
extern uint8_t canErrCode;
void TIM2_IRQHandler(void)
{
	#define PERIOD_COUNTER 10
	#define DEBUG_PERIOD_COUNTER 200
	
	//用来计数10次，产生10ms的定时器
	static uint8_t periodCounter = PERIOD_COUNTER;
	static uint8_t debugPeriodCounter = DEBUG_PERIOD_COUNTER;
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
		debugPeriodCounter--;
		if(debugPeriodCounter == 0)
		{
			OSSemPost(DebugPeriodSem);
			debugPeriodCounter = DEBUG_PERIOD_COUNTER;
		}
		if(gRobot.leftGun.commandState == GUN_NO_COMMAND)
		{
			gRobot.leftGun.noCommandTimer++;
		}
		if(gRobot.rightGun.commandState == GUN_NO_COMMAND)
		{
			gRobot.rightGun.noCommandTimer++;
		}
		if(moveTimFlag==1)
		{
			moveTimer+=0.001f;
		}
		velTimerCounting();
		
		
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
				else if(ch == 'S')
				{
					status +=12;
				}
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
				//判断目标停车点位置
				if(gRobot.moveBase.targetPoint != id2 / 80 + 1)
				{
					gRobot.moveBase.targetPoint = id2 / 80 + 1;
//					OSTaskSuspend(LEFT_GUN_SHOOT_TASK_PRIO);
//					OSTaskSuspend(RIGHT_GUN_SHOOT_TASK_PRIO);
//					OSTaskSuspend(UPPER_GUN_SHOOT_TASK_PRIO);
//					OSTaskResume(Walk_TASK_PRIO);
					status = 0;
				}
				id2 = id2 % 80;				
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
							//射击参数模式，左右枪没有打盘
							gRobot.leftGun.shootParaMode = id2 / 6;
							break;
							case 1:
							gRobot.rightGun.targetPose.pitch = targetAngle;
							gRobot.rightGun.shootParaMode = id2 / 6;
							break;
 							case 2:
							gRobot.upperGun.targetPose.pitch = targetAngle;
							//射击参数模式，上枪只有打盘
							gRobot.rightGun.shootParaMode = id2 / 3 - 1;
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
								gRobot.leftGun.aim = GUN_START_AIM;
//								OSTaskSuspend(Walk_TASK_PRIO);
								OSTaskResume(LEFT_GUN_SHOOT_TASK_PRIO);
							break;
							case 1:
								gRobot.rightGun.targetPose.speed2 = data.data32;
								gRobot.rightGun.aim = GUN_START_AIM;
//								OSTaskSuspend(Walk_TASK_PRIO);
								OSTaskResume(RIGHT_GUN_SHOOT_TASK_PRIO);							
							break;
							case 2:
								gRobot.upperGun.aim = GUN_START_AIM;
//								OSTaskSuspend(Walk_TASK_PRIO);
								OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
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
				if(id < 10)
				{
					switch(id)
					{
						case 1:
							//通知左枪开枪任务执行开枪动作
							gRobot.leftGun.shoot = GUN_START_SHOOT;
	//						OSTaskSuspend(Walk_TASK_PRIO);
							OSTaskResume(LEFT_GUN_SHOOT_TASK_PRIO);						
							break;
						case 2:
							//通知右枪开枪任务执行开枪动作
							gRobot.rightGun.shoot = GUN_START_SHOOT;
	//						OSTaskSuspend(Walk_TASK_PRIO);
							OSTaskResume(RIGHT_GUN_SHOOT_TASK_PRIO);
							break;
						case 3:
							//通知上面枪开枪任务执行开枪动作
							gRobot.upperGun.shoot = GUN_START_SHOOT;
	//						OSTaskSuspend(Walk_TASK_PRIO);
							OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
							break;
						case 4:
							//左枪自动模式
							gRobot.leftGun.mode = GUN_AUTO_MODE;
							OSTaskResume(LEFT_GUN_SHOOT_TASK_PRIO);
						break;
						case 5:
							//右枪自动模式
							gRobot.rightGun.mode = GUN_AUTO_MODE;
							OSTaskResume(RIGHT_GUN_SHOOT_TASK_PRIO);
						break;
						case 6:
							//上枪自动模式
							gRobot.upperGun.mode = GUN_ATTACK_MODE;
							OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
							break;
						case 7:
							//左枪手动模式
							gRobot.leftGun.mode = GUN_MANUAL_MODE;
							gRobot.leftGun.modeChangeFlag = 1;
//							LeftBack();
							break;
						case 8:
							//右枪手动模式
							gRobot.rightGun.mode = GUN_MANUAL_MODE;
							gRobot.rightGun.modeChangeFlag = 1;
//							RightBack();
							break;
						case 9:
							//上枪手动模式
							gRobot.upperGun.mode = GUN_MANUAL_MODE;
							break;
					}
				}
				else
				{
					//此部分为打完第一轮后接收补弹命令
					switch(id/10)
					{
						//id 10-16 为打球 ，0 - 6为1 - 7 号柱子
						case 1:
							if(gRobot.plantState[id - 10].ballState == COMMAND_DONE)
							{
								gRobot.plantState[id - 10].ball = 1;
							}
							break;
						//id 20-26 为落盘 ，0 - 6为1 - 7 号柱子
						case 2:
							if(gRobot.plantState[id - 20].plateState == COMMAND_DONE)
							{
								gRobot.plantState[id - 20].plate = 1;
							}								
							else
							{
								if(id-20==PLANT6)
								{
									gRobot.plantState[id - 20].plate = 1;									
								}
							}							
							break;
					}
				}
				status=0;
			break;
			case 14:
				if(ch == 'T')
				{
					status ++;
				}
				else
				{
					status = 0;
				}
				break;
			case 15:
				gRobot.plantState[PLANT1].ball = (ch&0x01)==0x01;
				gRobot.plantState[PLANT2].ball = (ch&0x02)==0x02;
				gRobot.plantState[PLANT3].ball = (ch&0x04)==0x04;
				gRobot.plantState[PLANT4].ball = (ch&0x08)==0x08;
				gRobot.plantState[PLANT5].ball = (ch&0x10)==0x10;
				gRobot.plantState[PLANT6].ball = (ch&0x20)==0x20;
				gRobot.plantState[PLANT7].ball = (ch&0x40)==0x40;
				status++;
				break;
			case 16:
				gRobot.plantState[PLANT1].plate = (ch&0x01)==0x01;
				gRobot.plantState[PLANT2].plate = (ch&0x02)==0x02;
				gRobot.plantState[PLANT3].plate = (ch&0x04)==0x04;
				gRobot.plantState[PLANT4].plate = (ch&0x08)==0x08;
				gRobot.plantState[PLANT5].plate = (ch&0x10)==0x10;
				gRobot.plantState[PLANT6].plate = (ch&0x20)==0x20;
				gRobot.plantState[PLANT7].plate = (ch&0x40)==0x40;
				status = 0;
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

extern  OS_EVENT 		*GyroSem;
void USART6_IRQHandler(void)       //更新频率200Hz
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

	if(USART_GetITStatus(USART6, USART_IT_RXNE)==SET)   
	{
		OSSemPost(GyroSem);		
		USART_ClearITPendingBit( USART6,USART_IT_RXNE);
		ch=USART_ReceiveData(USART6);
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
					gRobot.moveBase.actualAngle = -posture.ActVal[0];
					posture.ActVal[1]           = posture.ActVal[1];
					posture.ActVal[2]           = posture.ActVal[2];
					gRobot.moveBase.actualXPos  = posture.ActVal[3];
					gRobot.moveBase.actualYPos  = posture.ActVal[4];
					posture.ActVal[5]           = posture.ActVal[5];
					UpdateXYAngle(posture.ActVal[0], posture.ActVal[3], posture.ActVal[4]);
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
			USART_ClearITPendingBit( USART6,USART_IT_PE);
			USART_ClearITPendingBit( USART6,USART_IT_TXE);
			USART_ClearITPendingBit( USART6,USART_IT_TC);
			USART_ClearITPendingBit( USART6,USART_IT_ORE_RX);
			USART_ClearITPendingBit( USART6,USART_IT_IDLE);
			USART_ClearITPendingBit( USART6,USART_IT_LBD);
			USART_ClearITPendingBit( USART6,USART_IT_CTS);
			USART_ClearITPendingBit( USART6,USART_IT_ERR);
			USART_ClearITPendingBit( USART6,USART_IT_ORE_ER);
			USART_ClearITPendingBit( USART6,USART_IT_NE);
			USART_ClearITPendingBit( USART6,USART_IT_FE);
			USART_ReceiveData(USART6);
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
void USART3_IRQHandler(void)        
{	 
#define HEADER1 0x80
#define HEADER2 0x80

#define SELF_HEADER1 0x81
#define SELF_HEADER2 0x81
	
#define POS_HEADER1 0x88	
#define POS_HEADER2 0x88	

#define READY_HEADER 0x8A
	
#define HEADER_STATE1 0
#define HEADER_STATE2 1
#define DATA_STATE 2
#define HEADER_STATE3 3
#define POS_DATA_STATE1  4
#define POS_DATA_STATE2 5
#define SELF_DATA_STATE 6
	
#define SELF_NEED_PLATE 0x90
#define SELF_ALREADY_HAVE 0x91	
	static uint8_t data = 0;
 	static int state = 0;
	static union
    {
		uint8_t data[2];
		uint16_t ActPos;
    }posInfo;
	
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)   
	{
		USART_ClearITPendingBit( USART3,USART_IT_RXNE);
		data = USART_ReceiveData(USART3);	

		switch(state)
		{
			case HEADER_STATE1:
				if(data == HEADER1)
				{
					state++;
				}
				else if(data == POS_HEADER1)
				{
					state += 3;
				}
				else if(data == READY_HEADER)
				{
					state ++;
				}
				else if(data == SELF_HEADER1)
				{
					state ++;
				}
				else
				{
					state = 0;
				}
				break;
			case HEADER_STATE2:
				if(data == HEADER2)
				{
					state = DATA_STATE;
				}
				else if(data == READY_HEADER)
				{
					//fix me 摄像头已经初始化完毕，还没有加蜂鸣器提醒
					state = 0;
				}
				else if(data == SELF_HEADER2)
				{
					state = SELF_DATA_STATE;
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
				if(data != 0x00)
				{
					OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
				}
				state = 0;
				break;
			case HEADER_STATE3:
				if(data == POS_HEADER2)
				{
					state = POS_DATA_STATE1;
				}
			break;
				//到场地中央后初始化摄像头，返回位置信息16位 0-512
			case POS_DATA_STATE1:
				//低8位
				posInfo.data[0] = data;
				state ++;
			break;
			case POS_DATA_STATE2:
				//高8位
				posInfo.data[1] = data;
				gRobot.moveBase.relativePos = posInfo.ActPos;
				state = 0;
			break;
			case SELF_DATA_STATE:
				if(data == SELF_NEED_PLATE)
				{
					gRobot.upperGun.isSelfEmpty = SELF_EMPTY;
				}
				else if(data == SELF_ALREADY_HAVE)
				{
					gRobot.upperGun.isSelfEmpty = SELF_OK;				
				}
				state = 0;
				break;
			default:
				break;
		}	
	}
	
	OSIntExit();
}

void UART5_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)   
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		
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
	   UART5_OUT((uint8_t *)"NMI exception !!!!!!!!!!!!!\r\n");
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
	  UART5_OUT((uint8_t *)"Hard Fault exception!!!!!!!!!!\r\n");
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
	  UART5_OUT((uint8_t *)"Memory Manage exception occurs!!!!!!!!!\r\n");
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
	  UART5_OUT((uint8_t *)"Bus Fault exception!!!!!!!!\r\n");
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
	  UART5_OUT((uint8_t *)"Usage Fault exception!!!!!!!!!\r\n");
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

