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
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "elmo.h"
#include "movebase.h"
#include "movebase2.h"

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

//声明外部变量
extern robot_t gRobot;


//用来处理CAN接收数据
union MSG
{
	uint8_t data8[8];
	int data32[2];
	float dataf[2];
}msg;

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
		canNodeId == UPPER_GUN_LEFT_ID || canNodeId == UPPER_GUN_RIGHT_ID)
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
			if(canNodeId == UPPER_GUN_RIGHT_ID)
			{
				gRobot.upperGun.actualPose.speed2 = UpperGunRightSpeedInverseTransform(msg.data32[1]);
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
	UART5_OUT((uint8_t *)"CAN1 BUS OFF %d!!\r\n" ,CAN_GetLastErrorCode(CAN1));
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
	UART5_OUT((uint8_t *)"CAN2 BUS OFF %d!!\r\n" ,CAN_GetLastErrorCode(CAN2));
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
				gRobot.moveBase.driverCommandVelocity.backwardDriverCommandVelocity = (msg.data32[1])/100.0f;
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
	#define DEBUG_PERIOD_COUNTER 20
	#define BLE_CHECK_COUNTER 3000
	//用来计数10次，产生10ms的定时器
	static uint8_t periodCounter = PERIOD_COUNTER;
	static uint8_t debugPeriodCounter = DEBUG_PERIOD_COUNTER;
	static uint16_t bleCheckCounter = BLE_CHECK_COUNTER;
	static int lastBleHeartBeat = 0;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();


	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{

		//实现10ms 发送1次信号量
		periodCounter--;
		if (periodCounter == 0)
		{
			gRobot.reloadTimer++;
			if(gRobot.reloadTimer==65535)
			{
				gRobot.reloadTimer = 0;		
			}
			gRobot.shootTimer++;
			if(gRobot.shootTimer==65535)
			{
				gRobot.shootTimer = 0;		
			}			
			OSSemPost(PeriodSem);
			periodCounter = PERIOD_COUNTER;
		}
		//实现200ms 发送1次信号量
		debugPeriodCounter--;
		if(debugPeriodCounter == 0)
		{
			OSSemPost(DebugPeriodSem);
			debugPeriodCounter = DEBUG_PERIOD_COUNTER;
		}

		//记录左枪无命令时间 1ms 递增1
		if(gRobot.leftGun.commandState == GUN_NO_COMMAND)
		{
			gRobot.leftGun.noCommandTimer++;
		}
		//记录右枪无命令时间 1ms 递增1
		if(gRobot.rightGun.commandState == GUN_NO_COMMAND)
		{
			gRobot.rightGun.noCommandTimer++;
		}

		/*走形用计时*/
		//在moveTimeFlag置位时moveTimer 增加 单位：在数值上是秒
		//fix me 使用float计时并不准确
		if(moveTimFlag == 1)
		{
			moveTimer += 0.001f;
		}

		/*计时 判断ble失联*/
		if(gRobot.isBleOk.bleCheckStartFlag == BLE_CHECK_START)
		{
			//满足计数条件时 每1ms 递减1 实现3000ms 检查一次
			bleCheckCounter--;
			if(bleCheckCounter == 0)
			{
				if(gRobot.isBleOk.bleHeartBeat - lastBleHeartBeat <= 0)
				{
					gRobot.isBleOk.noBleFlag = BLE_LOST;
					UART5_OUT((uint8_t *)"BLE LOST\r\n");
				}
				else
				{
					if(gRobot.isBleOk.noBleFlag == BLE_LOST)
					{
						gRobot.manualCmdQueue.headNum = gRobot.manualCmdQueue.tailNum;
					}
					gRobot.isBleOk.noBleFlag = BLE_OK;
				}
				lastBleHeartBeat = gRobot.isBleOk.bleHeartBeat;
				bleCheckCounter = BLE_CHECK_COUNTER;
			}
		}
		else
		{
			bleCheckCounter = BLE_CHECK_COUNTER;
		}
		velTimerCounting();

		gunTimCnt ++;

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
static int8_t msgId = -1;
static uint8_t bleUseNum = 0;
static uint8_t bleIsStopDefend = BLE_START_DEFEND;
#define MSG_ID_LIMIT  (101)
typedef union
{
	uint8_t data8[4];
	uint32_t data32;
	float dataf;
}data_32bit_t;

void ActionCommunicate(uint8_t* ch, int* status, uint8_t* cmdFlag,uint8_t* id, uint8_t* id2, data_32bit_t* data, cmd_t* manualCmd)
{
			switch (*status)
			{
				case 0:
				{
					if (*ch == 'A')
					{
						if(gRobot.isBleOk.bleCheckStartFlag == BLE_CHECK_START)
						{
							gRobot.isBleOk.bleHeartBeat++;
							gRobot.isBleOk.noBleFlag = BLE_OK;
						}
						*cmdFlag = 0;
						(*status)++;
					}
					break;
				}
				case 1:
				{
					if (*ch == 'C')
						(*status)++;
					else
						*status = 0;
					break;
				}
				case 2:
				{
					if (*ch == 'P')
						(*status)++;				   //ACPC + [id1] + [id2] + data[4]
					else if (*ch == 'C')
						(*status) += 10;			   //ACCT + [id]
					else if(*ch == 'H')
					{
						*status +=12;
					}
					else
						(*status) = 0;
					break;
				}
				case 3: 						   /*ACPC begin from here*/
				{
					if (*ch == 'C')
						(*status)++;
					else
						(*status) = 0;
					break;
				}
				case 4:
				{
					*id = *ch;
					(*status)++;
					break;
				}
				case 5:
				{
					*id2 = *ch;				//左	打球0 打盘3 扔6  右 打球1 打盘4 扔7	上 打球2 打盘5 扔8
					*id2 = *id2 % 80;
					(*status)++;
					break;
				}
				case 6:
				case 7:
				case 8:
				case 9:
				case 10:
				{
					if((*status) < 10)
						data->data8[*status - 6] = *ch;
					(*status)++;
					break;
				}
				case 11:
				{
					switch(*id % 5)
					{
						//roll
						case 0:
							switch(*id2 % 3)
							{
								case 0:
								//左枪
								gRobot.leftGun.targetPose.roll = data->dataf;
								//1~7表示7个着陆台，转换为0~6
								gRobot.leftGun.targetPlant = *id/5;
								break;
								case 1:
								gRobot.rightGun.targetPose.roll = data->dataf;
								//1~7表示7个着陆台，转换为0~6
								gRobot.rightGun.targetPlant = *id/5;
								break;
								case 2:
								break;
								default:
									*id2 = 0xff;
								break;
							}
							break;
						case 1:
							//pitch
							switch(*id2 % 3)
							{
								case 0:
								gRobot.leftGun.targetPose.pitch = data->dataf;
								//射击参数模式，左右枪没有打盘
								gRobot.leftGun.shootParaMode = *id2 / 6;
								break;
								case 1:
								gRobot.rightGun.targetPose.pitch = data->dataf;
								gRobot.rightGun.shootParaMode = *id2 / 6;
								break;
								case 2:
								gRobot.upperGun.targetPose.pitch = data->dataf;
								//射击参数模式，上枪只有打盘
								gRobot.rightGun.shootParaMode = *id2 / 3 - 1;
								break;
								default:
									*id2=0xff;
								break;
							}
							break;
						case 2:
							//yaw
							switch(*id2 % 3)
							{
								case 0:
								gRobot.leftGun.targetPose.yaw = data->dataf;
								break;
								case 1:
								gRobot.rightGun.targetPose.yaw = data->dataf;
								break;
								case 2:
								gRobot.upperGun.targetPose.yaw = data->dataf;
								break;
								default:
									*id2 = 0xff;
								break;
							}
							break;
						case 3:
							switch(*id2 % 3)
							{
								case 0:
									gRobot.leftGun.targetPose.speed1 = data->dataf;
								break;
								case 1:
									gRobot.rightGun.targetPose.speed1 = data->dataf;
								break;
								case 2:
									gRobot.upperGun.targetPose.speed1 = data->dataf;
								break;
								default:
									*id2 = 0xff;
								break;
							}
							break;
						case 4:
							switch(*id2 % 3)
							{
								case 0:
									gRobot.leftGun.targetPose.speed2 = data->dataf;
									gRobot.leftGun.aim = GUN_START_AIM;
	//								OSTaskSuspend(Walk_TASK_PRIO);
									OSTaskResume(LEFT_GUN_SHOOT_TASK_PRIO);
								break;
								case 1:
									gRobot.rightGun.targetPose.speed2 = data->dataf;
									gRobot.rightGun.aim = GUN_START_AIM;
	//								OSTaskSuspend(Walk_TASK_PRIO);
									OSTaskResume(RIGHT_GUN_SHOOT_TASK_PRIO);
								break;
								case 2:
									gRobot.upperGun.targetPose.speed2 = data->dataf;
									gRobot.upperGun.aim = GUN_START_AIM;
	//								OSTaskSuspend(Walk_TASK_PRIO);
									OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
								break;
								default:
									*id2 = 0xff;
								break;
							}
							break;
						default:
							*id = 0xff;
						break;
					}
					*status = 0;
					*id = 0xff;
					*id2 = 0xff;
					break;
				}
				case 12:							  /*ACCT begin from here*/
				{
					if (*ch == 'T')
						(*status)++;
					else
						*status = 0;
					break;
				}
				case 13:
				{
					*id = *ch;
				//当试场时三个平板同时发送命令，序号的顺序不一致
#ifdef TEST_RUN
					if(*id < 10)
					{

						switch(*id)
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
#endif
					//*id在10~30区段为接收补弹命令
					if(*id>=10&&*id<30)
					{
						switch(*id/10)
						{
							//*id 10-16 为打球 ，0 - 6为1 - 7 号柱子
							case 1:
	//							if(gRobot.plantState[*id - 10].ballState == COMMAND_DONE)
	//							{
	//								gRobot.plantState[*id - 10].ball = 1;
	//							}
	//							if((gRobot.manualCmdQueue.cmdBallState&(0x01<<(*id - 10)))==0)
	//							{
								manualCmd->plantNum = *id - 10;
								manualCmd->method = SHOOT_METHOD3;
								*cmdFlag = 1;
	//								InCmdQueue(manualCmd);
	//								CheckCmdQueueState();
	//							}
							break;
							//*id 20-26 为落盘 ，0 - 6为1 - 7 号柱子
							case 2:
	//							if(gRobot.plantState[*id - 20].plateState == COMMAND_DONE)
	//							{
	//								gRobot.plantState[*id - 20].plate = 1;
	//							}
	//							else
	//							{
	//								if(*id-20==PLANT6)
	//								{
	//									gRobot.plantState[*id - 20].plate = 1;
	//								}
	//							}
	//							if((gRobot.manualCmdQueue.cmdPlateState&(0x01<<(*id - 20)))==0 || (*id - 20 == PLANT6))
	//							{
								manualCmd->plantNum = *id - 20;
								manualCmd->method = SHOOT_METHOD4;
								*cmdFlag = 1;
	//								InCmdQueue(manualCmd);
	//								CheckCmdQueueState();
	//							}
							break;
						}
					}
					else
					{
						*cmdFlag = 0;
					}
					UART5_OUT((uint8_t *)"cmd%d",*cmdFlag);
					*status=15;
					break;
				}
				case 14:
				{
					if(*ch == 'B')
					{
						if(gRobot.isBleOk.bleCheckStartFlag == BLE_CHECK_START)
						{
							gRobot.isBleOk.bleHeartBeat++;
						}
						*status = 22;
					}
					else
					{
						*status = 0;
					}
					break;
				}
				case 15:
					(*status)++;
					break;
				case 16:
					(*status)++;
					break;
				case 17:
					(*status)++;
					break;
				case 18:
					(*status)++;
					break;
				case 19:
					(*status)++;
					break;
				case 20:
					(*status)++;
					break;
				case 21:
				{
					UART5_OUT((uint8_t *)"CMD%d",*cmdFlag);
					UART5_OUT((uint8_t *)"ID%dR%d",*ch,msgId);
					if(*cmdFlag == 1)
					{
						if((*ch - msgId < 10u && *ch - msgId > 0u)\
							|| (msgId - *ch > 90u))
						{
							UART5_OUT((uint8_t *)"BLE");
							if((manualCmd->plantNum != PLANT3 && manualCmd->plantNum != PLANT7)||gRobot.upperGun.bulletNumber == GUN_NO_BULLET_ERROR)
							{
								InCmdQueue(*manualCmd);
								UART5_OUT((uint8_t *)"%d %d\r\n",gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum - 1].plantNum,\
								gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum - 1].method);
								if(gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum - 1].method%2)
								{
									gRobot.manualCmdQueue.cmdPlateState |= (0x01<<gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum - 1].plantNum);
								}
								else
								{
									gRobot.manualCmdQueue.cmdBallState |= (0x01<<gRobot.manualCmdQueue.cmdArr[gRobot.manualCmdQueue.tailNum - 1].plantNum); 		
								}
							}
							else
							{
								switch(manualCmd->plantNum)
								{
									case PLANT3:
									{
										if(manualCmd->method == SHOOT_METHOD3)
										{
											if(gRobot.plantState[PLANT3].ballState == COMMAND_DONE)
											{
												gRobot.plantState[PLANT3].ball = 1;
												gRobot.manualCmdQueue.cmdBallState |= 0x04;
												UART5_OUT((uint8_t *)"upperplant3method3\r\n");
											}
										}
										else if(manualCmd->method == SHOOT_METHOD4)
										{
											if(gRobot.plantState[PLANT3].plateState == COMMAND_DONE)
											{
												gRobot.plantState[PLANT3].plate = 1;
												gRobot.manualCmdQueue.cmdPlateState |= 0x04;
												UART5_OUT((uint8_t *)"upperplant3method4\r\n");
											}
										}
										break;
									}
									case PLANT7:
									{
										if(manualCmd->method == SHOOT_METHOD3)
										{
											if(gRobot.plantState[PLANT7].ballState == COMMAND_DONE)
											{
												gRobot.plantState[PLANT7].ball = 1;
												gRobot.manualCmdQueue.cmdBallState |= 0x40;
												UART5_OUT((uint8_t *)"upperplant7method3\r\n");
											}
										}
										else if(manualCmd->method == SHOOT_METHOD4)
										{
											if(gRobot.plantState[PLANT7].plateState == COMMAND_DONE)
											gRobot.plantState[PLANT7].plate = 1;
											gRobot.manualCmdQueue.cmdPlateState |= 0x40;
											UART5_OUT((uint8_t *)"upperplant7method4\r\n");
										}
										break;
									}
									default:
										break;
								}
							}
							msgId = *ch;
		//					CheckCmdQueueState();
		//					DelTailQueue();
		//					CheckCmdQueueState();
						}
					}
					else
					{
						if((*ch - msgId < 10u && *ch - msgId > 0u)\
							|| (msgId - *ch > 90u))
						{
							if(*id < 10)
							{
#ifndef TEST_RUN
								switch(*id)
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
#endif
							}
							else if(*id == 30)
							{
								//重启
								gRobot.isReset = ROBOT_RESET;
							}
							else if(*id < 50)
							{
								//40~45对应6个防守分区
								if(*id >= 40)
								{
									gRobot.upperGun.defendZone1 = *id - 40 + 1;
									gRobot.upperGun.defendZone2 = 0x00;
									gRobot.upperGun.isManualDefend = UPPER_MANUAL_DEFEND;
									OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
								}
							}
							else if(*id < 60)
							{
								if(*id == 50)
								{
									gRobot.leftGun.bulletNumber = LEFT_BULLET_NUM - gRobot.leftGun.shootTimes;
								}
								if(*id == 51)
								{
									gRobot.leftGun.bulletNumber = GUN_NO_BULLET_ERROR;
								}
								if(*id == 52)
								{
									gRobot.rightGun.bulletNumber = RIGHT_BULLET_NUM - gRobot.rightGun.shootTimes;
								}
								if(*id == 53)
								{
									gRobot.rightGun.bulletNumber = GUN_NO_BULLET_ERROR;
								}
							}
							else
							{
								//ID == 60 停止摄像头防守
								if(*id == 60)
								{
									bleIsStopDefend = BLE_STOP_DEFEND;
									gRobot.isBleOk.isStopDefend = BLE_STOP_DEFEND;
									gRobot.upperGun.isManualDefend = UPPER_MANUAL_DEFEND;
									gRobot.upperGun.defendZone1 = 0x00;
									gRobot.upperGun.defendZone2 = 0x00; 					
								}
								//ID == 61 恢复摄像头防守
								if(*id == 61)
								{
									bleIsStopDefend = BLE_START_DEFEND;
									gRobot.isBleOk.isStopDefend = BLE_START_DEFEND;
									gRobot.upperGun.isManualDefend = UPPER_AUTO_DEFEND;
								}
								//ID == 70 从出发区出发
								if(*id == 70)
								{
									gRobot.isLeaveSZ = ROBOT_LEAVE_SZ;
								}
								//ID == 71 重新装弹
								if(*id == 71)
								{
									gRobot.isReload = ROBOT_RELOAD;
								}
								if(*id>=80&&*id<90)
								{
									switch(*id)
									{
										case 80:
											gRobot.moveBase.targetPoint = SHOOT_POINT1;//Left
										break;
										case 81:
											gRobot.moveBase.targetPoint = SHOOT_POINT2;//Center
										break;
										case 82:
											gRobot.moveBase.targetPoint = SHOOT_POINT3;//Right
										break;
										default:break;
									}
								}							
								//ID=255，全部切换为自动模式
								if(*id == 255)
								{
									//左枪自动模式
									gRobot.leftGun.mode = GUN_AUTO_MODE;
									OSTaskResume(LEFT_GUN_SHOOT_TASK_PRIO);
									//右枪自动模式
									gRobot.rightGun.mode = GUN_AUTO_MODE;
									OSTaskResume(RIGHT_GUN_SHOOT_TASK_PRIO);
									//上枪自动模式
									gRobot.upperGun.mode = GUN_ATTACK_MODE;
									OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
								}
							}
							msgId = *ch;
						}
					}
					*status = 0;
					break;
				}
				case 22:
					(*status)++;
					break;
				case 23:
					(*status)++;
					break;
				case 24:
					(*status)++;
					break;
				case 25:
					(*status)++;
					break;
				case 26:
					(*status)++;
					break;
				case 27:
					//向平板发送是否停止防守
					*ch = bleIsStopDefend;
					(*status)++;
					break;
				case 28:
					//向平板发送打球命令状态
					*ch = gRobot.manualCmdQueue.cmdBallState;
					(*status)++;
					break;
				case 29:
					//向平板发送落盘命令状态
					*ch = gRobot.manualCmdQueue.cmdPlateState;
					UART5_OUT((uint8_t *)"%d %d\r\n",gRobot.manualCmdQueue.cmdBallState,gRobot.manualCmdQueue.cmdPlateState);
					*status = 0;
					break;
				default:
					*status = 0;
					*id = 0xff;
					break;
			}

}


void UART4_IRQHandler(void)
{
	static int	status = 0;
	static uint8_t id = 0xff ,id2 = 0xff;
	static uint8_t bleNumCountFlag = 1;
	static uint8_t cmdFlag = 0;
	static cmd_t manualCmd = {INVALID_PLANT_NUMBER , INVALID_SHOOT_METHOD};
	static uint8_t bleMsg[12]={0};
	static uint8_t bleMsgCounter = 0;
	static data_32bit_t data;


	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		uint8_t ch;
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		ch = USART_ReceiveData(UART4);
		bleMsg[bleMsgCounter]=ch;
		if(bleMsgCounter == 11)
		{
			UART5_OUT((uint8_t *)"UART4%d %d %d %d %d %d %d %d %d %d %d %d\r\n",bleMsg[0],\
			bleMsg[1],bleMsg[2],bleMsg[3],bleMsg[4],bleMsg[5],bleMsg[6],bleMsg[7],\
			bleMsg[8],bleMsg[9],bleMsg[10],bleMsg[11]);
		}			
		bleMsgCounter = (bleMsgCounter + 1)%12;

		if(bleNumCountFlag == 1)
		{
			bleUseNum++;
			bleNumCountFlag = 0;
		}
		gRobot.isBleOk.noBleTimer = 0;
		ActionCommunicate(&ch, &status, &cmdFlag, &id, &id2, &data, &manualCmd);
		USART_SendData(UART4, ch);
	 }
	else
	{
		//虽然没有使能其他中断，但是查看是否有其他中断
		if(USART_GetITStatus(UART4, USART_IT_PE) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_PE);
			UART5_OUT((uint8_t*)"USART_IT_PE");
		}
		if(USART_GetITStatus(UART4, USART_IT_TXE) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_TXE);
			UART5_OUT((uint8_t*)"USART_IT_TXE");
		}
		if(USART_GetITStatus(UART4, USART_IT_TC) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_TC);
			UART5_OUT((uint8_t*)"USART_IT_TC");
		}
		if(USART_GetITStatus(UART4, USART_IT_ORE_RX) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_ORE_RX);
			UART5_OUT((uint8_t*)"USART_IT_ORE_RX");
		}
		if(USART_GetITStatus(UART4, USART_IT_IDLE) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_IDLE);
			UART5_OUT((uint8_t*)"USART_IT_IDLE");
		}
		if(USART_GetITStatus(UART4, USART_IT_LBD) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_LBD);
			UART5_OUT((uint8_t*)"USART_IT_LBD");
		}
		if(USART_GetITStatus(UART4, USART_IT_CTS) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_CTS);
			UART5_OUT((uint8_t*)"USART_IT_CTS");
		}
		if(USART_GetITStatus(UART4, USART_IT_ERR) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_ERR);
			UART5_OUT((uint8_t*)"USART_IT_ERR");
		}
		if(USART_GetITStatus(UART4, USART_IT_ORE_ER) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_ORE_ER);
			UART5_OUT((uint8_t*)"USART_IT_ORE_ER");
		}
		if(USART_GetITStatus(UART4, USART_IT_NE) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_NE);
			UART5_OUT((uint8_t*)"USART_IT_NE");
		}
		if(USART_GetITStatus(UART4, USART_IT_FE) == SET)
		{
			USART_ClearITPendingBit( UART4,USART_IT_FE);
			UART5_OUT((uint8_t*)"USART_IT_FE");
		}

		USART_ReceiveData(UART4);
		UART5_OUT((uint8_t*)"UART4_Err");
	}
	OSIntExit();
}
/***************************试场调参数用蓝牙串口中断*****************************************************/
void USART1_IRQHandler(void)
{
	static int	status = 0;
	static uint8_t id = 0xff ,id2 = 0xff;
	static uint8_t bleNumCountFlag = 1;
	static uint8_t cmdFlag = 0;
	static cmd_t manualCmd = {INVALID_PLANT_NUMBER , INVALID_SHOOT_METHOD};
	static uint8_t bleMsg[12]={0};
	static uint8_t bleMsgCounter = 0;
	static data_32bit_t data;


	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t ch;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		ch = USART_ReceiveData(USART1);
		bleMsg[bleMsgCounter]=ch;
		if(bleMsgCounter == 11)
		{
			UART5_OUT((uint8_t *)"USART1%d %d %d %d %d %d %d %d %d %d %d %d\r\n",bleMsg[0],\
			bleMsg[1],bleMsg[2],bleMsg[3],bleMsg[4],bleMsg[5],bleMsg[6],bleMsg[7],\
			bleMsg[8],bleMsg[9],bleMsg[10],bleMsg[11]);
		}			
		bleMsgCounter = (bleMsgCounter + 1)%12;

		if(bleNumCountFlag == 1)
		{
			bleUseNum++;
			bleNumCountFlag = 0;
		}
		gRobot.isBleOk.noBleTimer = 0;
		ActionCommunicate(&ch, &status, &cmdFlag, &id, &id2, &data, &manualCmd);
		USART_SendData(USART1, ch);
	 }
	else
	{
		//虽然没有使能其他中断，但是查看是否有其他中断
		if(USART_GetITStatus(USART1, USART_IT_PE) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_PE);
			UART5_OUT((uint8_t*)"USART_IT_PE");
		}
		if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_TXE);
			UART5_OUT((uint8_t*)"USART_IT_TXE");
		}
		if(USART_GetITStatus(USART1, USART_IT_TC) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_TC);
			UART5_OUT((uint8_t*)"USART_IT_TC");
		}
		if(USART_GetITStatus(USART1, USART_IT_ORE_RX) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_ORE_RX);
			UART5_OUT((uint8_t*)"USART_IT_ORE_RX");
		}
		if(USART_GetITStatus(USART1, USART_IT_IDLE) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_IDLE);
			UART5_OUT((uint8_t*)"USART_IT_IDLE");
		}
		if(USART_GetITStatus(USART1, USART_IT_LBD) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_LBD);
			UART5_OUT((uint8_t*)"USART_IT_LBD");
		}
		if(USART_GetITStatus(USART1, USART_IT_CTS) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_CTS);
			UART5_OUT((uint8_t*)"USART_IT_CTS");
		}
		if(USART_GetITStatus(USART1, USART_IT_ERR) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_ERR);
			UART5_OUT((uint8_t*)"USART_IT_ERR");
		}
		if(USART_GetITStatus(USART1, USART_IT_ORE_ER) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_ORE_ER);
			UART5_OUT((uint8_t*)"USART_IT_ORE_ER");
		}
		if(USART_GetITStatus(USART1, USART_IT_NE) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_NE);
			UART5_OUT((uint8_t*)"USART_IT_NE");
		}
		if(USART_GetITStatus(USART1, USART_IT_FE) == SET)
		{
			USART_ClearITPendingBit( USART1,USART_IT_FE);
			UART5_OUT((uint8_t*)"USART_IT_FE");
		}

		USART_ReceiveData(USART1);
		UART5_OUT((uint8_t*)"USART1_Err");
	}
	OSIntExit();
}

void USART2_IRQHandler(void)
{
	static int	status = 0;
	static uint8_t id = 0xff ,id2 = 0xff;
	static uint8_t bleNumCountFlag = 1;
	static uint8_t cmdFlag = 0;
	static cmd_t manualCmd = {INVALID_PLANT_NUMBER , INVALID_SHOOT_METHOD};
	static uint8_t bleMsg[12]={0};
	static uint8_t bleMsgCounter = 0;
	static data_32bit_t data;


	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		uint8_t ch;
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		ch = USART_ReceiveData(USART2);
		bleMsg[bleMsgCounter]=ch;
		if(bleMsgCounter == 11)
		{
			UART5_OUT((uint8_t *)"USART1%d %d %d %d %d %d %d %d %d %d %d %d\r\n",bleMsg[0],\
			bleMsg[1],bleMsg[2],bleMsg[3],bleMsg[4],bleMsg[5],bleMsg[6],bleMsg[7],\
			bleMsg[8],bleMsg[9],bleMsg[10],bleMsg[11]);
		}			
		bleMsgCounter = (bleMsgCounter + 1)%12;

		if(bleNumCountFlag == 1)
		{
			bleUseNum++;
			bleNumCountFlag = 0;
		}
		gRobot.isBleOk.noBleTimer = 0;
		ActionCommunicate(&ch, &status, &cmdFlag, &id, &id2, &data, &manualCmd);
		USART_SendData(USART2, ch);
	 }
	else
	{
		//虽然没有使能其他中断，但是查看是否有其他中断
		if(USART_GetITStatus(USART2, USART_IT_PE) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_PE);
			UART5_OUT((uint8_t*)"USART_IT_PE");
		}
		if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_TXE);
			UART5_OUT((uint8_t*)"USART_IT_TXE");
		}
		if(USART_GetITStatus(USART2, USART_IT_TC) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_TC);
			UART5_OUT((uint8_t*)"USART_IT_TC");
		}
		if(USART_GetITStatus(USART2, USART_IT_ORE_RX) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_ORE_RX);
			UART5_OUT((uint8_t*)"USART_IT_ORE_RX");
		}
		if(USART_GetITStatus(USART2, USART_IT_IDLE) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_IDLE);
			UART5_OUT((uint8_t*)"USART_IT_IDLE");
		}
		if(USART_GetITStatus(USART2, USART_IT_LBD) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_LBD);
			UART5_OUT((uint8_t*)"USART_IT_LBD");
		}
		if(USART_GetITStatus(USART2, USART_IT_CTS) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_CTS);
			UART5_OUT((uint8_t*)"USART_IT_CTS");
		}
		if(USART_GetITStatus(USART2, USART_IT_ERR) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_ERR);
			UART5_OUT((uint8_t*)"USART_IT_ERR");
		}
		if(USART_GetITStatus(USART2, USART_IT_ORE_ER) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_ORE_ER);
			UART5_OUT((uint8_t*)"USART_IT_ORE_ER");
		}
		if(USART_GetITStatus(USART2, USART_IT_NE) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_NE);
			UART5_OUT((uint8_t*)"USART_IT_NE");
		}
		if(USART_GetITStatus(USART2, USART_IT_FE) == SET)
		{
			USART_ClearITPendingBit( USART2,USART_IT_FE);
			UART5_OUT((uint8_t*)"USART_IT_FE");
		}

		USART_ReceiveData(USART2);
		UART5_OUT((uint8_t*)"USART2_Err");
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
	static float lastPosY = 0.0f;
	static float lastPosYDerivative = 0.0f;
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
					lastPosY = gRobot.moveBase.actualYPos;
					lastPosYDerivative = gRobot.moveBase.posYDerivative;
					gRobot.moveBase.actualAngle = -posture.ActVal[0];
					posture.ActVal[1]           = posture.ActVal[1];
					posture.ActVal[2]           = posture.ActVal[2];
					gRobot.moveBase.actualXPos  = posture.ActVal[3];
					gRobot.moveBase.actualYPos  = posture.ActVal[4];
					posture.ActVal[5]           = posture.ActVal[5];
					UpdateXYAngle(posture.ActVal[0], posture.ActVal[3], posture.ActVal[4]);
					gRobot.moveBase.posYDerivative = fabs(lastPosY - posture.ActVal[4])/0.01f;
					gRobot.moveBase.posYSecondDerivative = fabs(lastPosYDerivative - gRobot.moveBase.posYDerivative)/0.01f;
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
/*******************摄像头串口中断************************************************/
/*
* name:
* function:
* params:
* notes:
*/
u8 receive_data=0;
u8 receiveDataTrust = 0u;
void USART3_IRQHandler(void)
{
#define HEADER1 0x80
#define HEADER2 0x80

#define SELF_HEADER1 0x81
#define SELF_HEADER2 0x81

#define POS_HEADER1 0x88
#define POS_HEADER2 0x88

#define READY_HEADER 0x8A

#define PLAT_HEADER1 0x8B
#define PLAT_HEADER2 0x8B


#define HEADER_STATE1 0
#define HEADER_STATE2 1
#define DATA_STATE 2
#define HEADER_STATE3 3
#define POS_DATA_STATE1  4
#define POS_DATA_STATE2 5
#define SELF_DATA_STATE 6
#define PLAT_DATA_STATE1 7
#define PLAT_DATA_STATE2 8

#define SELF_NEED_PLATE 0x90
#define SELF_ALREADY_HAVE 0x91

#define PLAT_DATA_STABLE 1
#define PLAT_DATA_UNSTABLE 0
	static uint8_t data = 0;
 	static int state = 0;
	static union
    {
		uint8_t data[2];
		uint16_t ActPos;
    }posInfo;
//	static plant_t isPlateDataOk[LAND_NUMBER]={PLAT_DATA_UNSTABLE};
	cmd_t cameraCmd = {INVALID_PLANT_NUMBER,INVALID_SHOOT_METHOD};
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
				else if(data == PLAT_HEADER1)
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
				else if(data == PLAT_HEADER2)
				{
					state = PLAT_DATA_STATE1;
				}
				else
				{
					state=0;
				}
				break;
			case DATA_STATE:
				//更新7号着陆台飞盘位置, fix me
				if(gRobot.isReset != ROBOT_RESET)
				{
					if(gRobot.upperGun.shootTimes >= 7u)
					{
						if(gRobot.upperGun.isManualDefend != UPPER_MANUAL_DEFEND)
						{
							if(gRobot.moveBase.actualStopPoint == SHOOT_POINT2)
							{
								if ((data & 0x0f) <= 0x06 && ((data & 0xf0) >> 4) <= 0x06)
								{
									gRobot.upperGun.defendZone1 = ((data & 0x0f)     );
									gRobot.upperGun.defendZone2 = ((data & 0xf0) >> 4);
								}
							}
						}
						if(data != 0x00)
						{
							OSTaskResume(UPPER_GUN_SHOOT_TASK_PRIO);
						}
					}
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
					if(gRobot.plateShootTimes[PLANT6] > 2)
					{
						gRobot.upperGun.isSelfEmpty = SELF_EMPTY;
						gRobot.leftGun.isSelfEmpty = SELF_EMPTY;
						gRobot.rightGun.isSelfEmpty = SELF_EMPTY;
					}
				}
				else if(data == SELF_ALREADY_HAVE)
				{
					gRobot.upperGun.isSelfEmpty = SELF_OK;
				}
				state = 0;
				break;
			case PLAT_DATA_STATE1:
				receiveDataTrust = data;
//				isPlateDataOk[PLANT1].ball = (data&0x01)==0x01;
//				isPlateDataOk[PLANT1].plate = (data&0x02)==0x02;
//				isPlateDataOk[PLANT2].ball = (data&0x04)==0x04;
//				isPlateDataOk[PLANT2].plate = (data&0x08)==0x08;
//				isPlateDataOk[PLANT4].ball = (data&0x10)==0x10;
//				isPlateDataOk[PLANT4].plate = (data&0x20)==0x20;
//				isPlateDataOk[PLANT5].ball = (data&0x40)==0x40;
//				isPlateDataOk[PLANT5].plate = (data&0x80)==0x80;
				state = PLAT_DATA_STATE2;
				break;
			case PLAT_DATA_STATE2:
				receive_data=data;
//				if(isPlateDataOk[PLANT1].ball==PLAT_DATA_STABLE)
//				{
//					gRobot.cameraInfo[PLANT1].ball = (data&0x01)==0x01;
//					if(!((data&0x01)==0x01))
//					{
//						gRobot.autoCommand[PLANT1].ball = (data&0x01)==0x01;
//					}
//				}
//				if(isPlateDataOk[PLANT1].plate==PLAT_DATA_STABLE)
//				{
//					if(!((data&0x02)==0x02))
//					{
//						if(gRobot.plateShootTimes[PLANT1]!=0)
//						{
//							gRobot.cameraInfo[PLANT1].plate = (data&0x02)==0x02;
//							gRobot.autoCommand[PLANT1].plate = (data&0x02)==0x02;
//						}
//					}
//					else
//					{
//						gRobot.cameraInfo[PLANT1].plate = (data&0x02)==0x02;
//						gRobot.autoCommand[PLANT1].plate = (data&0x02)==0x02;
//					}
//				}
//				if(isPlateDataOk[PLANT2].ball==PLAT_DATA_STABLE)
//				{
//					gRobot.cameraInfo[PLANT2].ball = (data&0x04)==0x04;
//					if(!((data&0x04)==0x04))
//					{
//						gRobot.autoCommand[PLANT2].ball = (data&0x04)==0x04;
//					}
//				}
//				if(isPlateDataOk[PLANT2].plate==PLAT_DATA_STABLE)
//				{
//					if(!((data&0x08)==0x08))
//					{
//						if(gRobot.plateShootTimes[PLANT2]!=0)
//						{
//							gRobot.cameraInfo[PLANT2].plate = (data&0x08)==0x08;
//							gRobot.autoCommand[PLANT2].plate = (data&0x08)==0x08;
//						}
//					}
//					else
//					{
//						gRobot.cameraInfo[PLANT2].plate = (data&0x08)==0x08;
//						gRobot.autoCommand[PLANT2].plate = (data&0x08)==0x08;
//					}
//				}
//				if(isPlateDataOk[PLANT4].ball==PLAT_DATA_STABLE)
//				{
//					gRobot.cameraInfo[PLANT4].ball = (data&0x10)==0x10;
//					if(!((data&0x10)==0x10))
//					{
//						gRobot.autoCommand[PLANT4].ball = (data&0x10)==0x10;
//					}
//				}
//				if(isPlateDataOk[PLANT4].plate==PLAT_DATA_STABLE)
//				{
//					if(!((data&0x20)==0x20))
//					{
//						if(gRobot.plateShootTimes[PLANT4]!=0)
//						{
//							gRobot.cameraInfo[PLANT4].plate = (data&0x20)==0x20;
//							gRobot.autoCommand[PLANT4].plate = (data&0x20)==0x20;
//						}
//					}
//					else
//					{
//						gRobot.cameraInfo[PLANT4].plate = (data&0x20)==0x20;
//						gRobot.autoCommand[PLANT4].plate = (data&0x20)==0x20;
//					}
//				}
//				if(isPlateDataOk[PLANT5].ball==PLAT_DATA_STABLE)
//				{
//					gRobot.cameraInfo[PLANT5].ball = (data&0x40)==0x40;
//					if(!((data&0x40)==0x40))
//					{
//						gRobot.autoCommand[PLANT5].ball = (data&0x40)==0x40;
//					}
//				}
//				if(isPlateDataOk[PLANT5].plate==PLAT_DATA_STABLE)
//				{
//					if(!((data&0x80)==0x80))
//					{
//						if(gRobot.plateShootTimes[PLANT5]!=0)
//						{
//							gRobot.cameraInfo[PLANT5].plate = (data&0x80)==0x80;
//							gRobot.autoCommand[PLANT5].plate = (data&0x80)==0x80;
//						}
//					}
//					else
//					{
//						gRobot.cameraInfo[PLANT5].plate = (data&0x80)==0x80;
//						gRobot.autoCommand[PLANT5].plate = (data&0x80)==0x80;
//					}
//				}
//				if((data&0x0f)==0)
//				{
//					if(isPlateDataOk[PLANT1].ball == PLAT_DATA_STABLE && isPlateDataOk[PLANT1].plate == PLAT_DATA_STABLE &&\
//						isPlateDataOk[PLANT2].ball == PLAT_DATA_STABLE && isPlateDataOk[PLANT2].plate == PLAT_DATA_STABLE)
//					{
//						if(gRobot.plateShootTimes[PLANT1]!=0&&gRobot.plateShootTimes[PLANT2]!=0)
//						{
//							GPIO_ResetBits(GPIOC, GPIO_Pin_0);
//							gRobot.leftGun.gunCommand = (plant_t *)gRobot.plantState;
//						}
//					}
//				}
//				if((data&0xf0)==0)
//				{
//					if(isPlateDataOk[PLANT4].ball == PLAT_DATA_STABLE && isPlateDataOk[PLANT4].plate == PLAT_DATA_STABLE &&\
//						isPlateDataOk[PLANT5].ball == PLAT_DATA_STABLE && isPlateDataOk[PLANT5].plate == PLAT_DATA_STABLE)
//					{
//						if(gRobot.plateShootTimes[PLANT4]!=0&&gRobot.plateShootTimes[PLANT5]!=0)
//						{
//							BLUE_LED_ON;
//							gRobot.rightGun.gunCommand = (plant_t *)gRobot.plantState;
//						}
//					}
//				}
				if(gRobot.isBleOk.noBleFlag == BLE_LOST)
				{
					UART5_OUT((uint8_t*)"BLELOSTCAMERA");
					if(gRobot.leftGun.shootTimes >= LEFT_AUTO_NUMBER)
					{
						UART5_OUT((uint8_t*)"BLELOSTCAMERA");
						if((data&0x01)==0x01 && gRobot.plantState[PLANT1].ballState == COMMAND_DONE
							&& CheckShootPlantTimeDelay(PLANT1, SHOOT_METHOD3, 1100))
						{
							cameraCmd.method = SHOOT_METHOD3;
							cameraCmd.plantNum = PLANT1;
							if(CheckCmdInQueue(cameraCmd))
							{
								InCmdQueue(cameraCmd);
							}
							gRobot.plantState[PLANT1].ball = 1;
							UART5_OUT((uint8_t*)"BLELOSTCAMERA");
						}
						if((data&0x02)==0x02 && gRobot.plantState[PLANT1].plateState == COMMAND_DONE
							&& CheckShootPlantTimeDelay(PLANT1, SHOOT_METHOD4, 1100))
						{

							cameraCmd.method = SHOOT_METHOD4;
							cameraCmd.plantNum = PLANT1;
							if(CheckCmdInQueue(cameraCmd))
							{
								InCmdQueue(cameraCmd);
							}
							gRobot.plantState[PLANT1].plate = 1;
						}
						if((data&0x04)==0x04 && gRobot.plantState[PLANT2].ballState == COMMAND_DONE
							&& CheckShootPlantTimeDelay(PLANT2, SHOOT_METHOD3, 1100))
						{
							cameraCmd.method = SHOOT_METHOD3;
							cameraCmd.plantNum = PLANT2;
							if(CheckCmdInQueue(cameraCmd))
							{
								InCmdQueue(cameraCmd);
							}
							gRobot.plantState[PLANT2].ball = 1;
						}
						if((data&0x08)==0x08 && gRobot.plantState[PLANT2].plateState == COMMAND_DONE
							&& CheckShootPlantTimeDelay(PLANT2, SHOOT_METHOD4, 1100))
						{
							cameraCmd.method = SHOOT_METHOD4;
							cameraCmd.plantNum = PLANT2;
							if(CheckCmdInQueue(cameraCmd))
							{
								InCmdQueue(cameraCmd);
							}
							gRobot.plantState[PLANT2].plate = 1;
						}
					}
					if(gRobot.rightGun.shootTimes >= RIGHT_AUTO_NUMBER)
					{
						if((data&0x10)==0x10 && gRobot.plantState[PLANT4].ballState == COMMAND_DONE
							&& CheckShootPlantTimeDelay(PLANT4, SHOOT_METHOD3, 1100))
						{
							cameraCmd.method = SHOOT_METHOD3;
							cameraCmd.plantNum = PLANT4;
							if(CheckCmdInQueue(cameraCmd))
							{
								InCmdQueue(cameraCmd);
							}
							gRobot.plantState[PLANT4].ball = 1;
						}
						if((data&0x20)==0x20 && gRobot.plantState[PLANT4].plateState == COMMAND_DONE
							&& CheckShootPlantTimeDelay(PLANT4, SHOOT_METHOD4, 1100))
						{
							cameraCmd.method = SHOOT_METHOD4;
							cameraCmd.plantNum = PLANT4;
							if(CheckCmdInQueue(cameraCmd))
							{
								InCmdQueue(cameraCmd);
							}
							gRobot.plantState[PLANT4].plate = 1;
						}
						if((data&0x40)==0x40 && gRobot.plantState[PLANT5].ballState == COMMAND_DONE
							&& CheckShootPlantTimeDelay(PLANT5, SHOOT_METHOD3, 1100))
						{
							cameraCmd.method = SHOOT_METHOD3;
							cameraCmd.plantNum = PLANT5;
							if(CheckCmdInQueue(cameraCmd))
							{
								InCmdQueue(cameraCmd);
							}
							gRobot.plantState[PLANT5].ball = 1;
						}
						if((data&0x80)==0x80 && gRobot.plantState[PLANT5].plateState == COMMAND_DONE
							&& CheckShootPlantTimeDelay(PLANT5, SHOOT_METHOD4, 1100))
						{
							cameraCmd.method = SHOOT_METHOD4;
							cameraCmd.plantNum = PLANT5;
							if(CheckCmdInQueue(cameraCmd))
							{
								InCmdQueue(cameraCmd);
							}
							gRobot.plantState[PLANT5].plate = 1;
						}
					}
				}
				state = HEADER_STATE1;
				break;
			default:
				break;
		}
	}
	else
	{
		//虽然没有使能其他中断，但是查看是否有其他中断
		if(USART_GetITStatus(USART3, USART_IT_PE) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_PE);
			UART5_OUT((uint8_t*)"USART_IT_PE");
		}
		if(USART_GetITStatus(USART3, USART_IT_TXE) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_TXE);
			UART5_OUT((uint8_t*)"USART_IT_TXE");
		}
		if(USART_GetITStatus(USART3, USART_IT_TC) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_TC);
			UART5_OUT((uint8_t*)"USART_IT_TC");
		}
		if(USART_GetITStatus(USART3, USART_IT_ORE_RX) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_ORE_RX);
			UART5_OUT((uint8_t*)"USART_IT_ORE_RX");
		}
		if(USART_GetITStatus(USART3, USART_IT_IDLE) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_IDLE);
			UART5_OUT((uint8_t*)"USART_IT_IDLE");
		}
		if(USART_GetITStatus(USART3, USART_IT_LBD) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_LBD);
			UART5_OUT((uint8_t*)"USART_IT_LBD");
		}
		if(USART_GetITStatus(USART3, USART_IT_CTS) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_CTS);
			UART5_OUT((uint8_t*)"USART_IT_CTS");
		}
		if(USART_GetITStatus(USART3, USART_IT_ERR) == SET)
		{
			USART_ClearITPendingBit(USART3,USART_IT_ERR);
			UART5_OUT((uint8_t*)"USART_IT_ERR");
		}
		if(USART_GetITStatus(USART3, USART_IT_ORE_ER) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_ORE_ER);
			UART5_OUT((uint8_t*)"USART_IT_ORE_ER");
		}
		if(USART_GetITStatus(USART3, USART_IT_NE) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_NE);
			UART5_OUT((uint8_t*)"USART_IT_NE");
		}
		if(USART_GetITStatus(USART3, USART_IT_FE) == SET)
		{
			USART_ClearITPendingBit( USART3,USART_IT_FE);
			UART5_OUT((uint8_t*)"USART_IT_FE");
		}

		USART_ReceiveData(USART3);
		UART5_OUT((uint8_t*)"USART3_Err");
	}

	OSIntExit();
}
/**************WIFI串口中断作为备用的蓝牙串口中断*******************************/
void UART5_IRQHandler(void)
{
	static int	status = 0;
	static uint8_t id = 0xff ,id2 = 0xff;
	static cmd_t manualCmd = {INVALID_PLANT_NUMBER , INVALID_SHOOT_METHOD};

	static data_32bit_t data;

	static uint8_t bleMsg[12]={0};
	static uint8_t bleMsgCounter = 0;
	static uint8_t cmdFlag = 0;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
		uint8_t ch;
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		ch = USART_ReceiveData(UART5);
		bleMsg[bleMsgCounter]=ch;
		if(bleMsgCounter == 11)
		{
			UART5_OUT((uint8_t *)"Wifi%d %d %d %d %d %d %d %d %d %d %d %d\r\n",bleMsg[0],\
			bleMsg[1],bleMsg[2],bleMsg[3],bleMsg[4],bleMsg[5],bleMsg[6],bleMsg[7],\
			bleMsg[8],bleMsg[9],bleMsg[10],bleMsg[11]);
		}			
		bleMsgCounter = (bleMsgCounter + 1)%12;
		gRobot.isBleOk.noBleTimer = 0;
		ActionCommunicate(&ch, &status, &cmdFlag, &id, &id2, &data, &manualCmd);
	 }
	else
	{
		//虽然没有使能其他中断，但是查看是否有其他中断
		if(USART_GetITStatus(UART5, USART_IT_PE) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_PE);
			UART5_OUT((uint8_t*)"USART_IT_PE");
		}
		if(USART_GetITStatus(UART5, USART_IT_TXE) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_TXE);
			UART5_OUT((uint8_t*)"USART_IT_TXE");
		}
		if(USART_GetITStatus(UART5, USART_IT_TC) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_TC);
			UART5_OUT((uint8_t*)"USART_IT_TC");
		}
		if(USART_GetITStatus(UART5, USART_IT_ORE_RX) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_ORE_RX);
			UART5_OUT((uint8_t*)"USART_IT_ORE_RX");
		}
		if(USART_GetITStatus(UART5, USART_IT_IDLE) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_IDLE);
			UART5_OUT((uint8_t*)"USART_IT_IDLE");
		}
		if(USART_GetITStatus(UART5, USART_IT_LBD) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_LBD);
			UART5_OUT((uint8_t*)"USART_IT_LBD");
		}
		if(USART_GetITStatus(UART5, USART_IT_CTS) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_CTS);
			UART5_OUT((uint8_t*)"USART_IT_CTS");
		}
		if(USART_GetITStatus(UART5, USART_IT_ERR) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_ERR);
			UART5_OUT((uint8_t*)"USART_IT_ERR");
		}
		if(USART_GetITStatus(UART5, USART_IT_ORE_ER) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_ORE_ER);
			UART5_OUT((uint8_t*)"USART_IT_ORE_ER");
		}
		if(USART_GetITStatus(UART5, USART_IT_NE) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_NE);
			UART5_OUT((uint8_t*)"USART_IT_NE");
		}
		if(USART_GetITStatus(UART5, USART_IT_FE) == SET)
		{
			USART_ClearITPendingBit( UART5,USART_IT_FE);
			UART5_OUT((uint8_t*)"USART_IT_FE");
		}

		USART_ReceiveData(UART5);
		UART5_OUT((uint8_t*)"UART5_Err");
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

