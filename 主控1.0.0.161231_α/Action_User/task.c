#include <includes.h>
#include <app_cfg.h>
#include "robot.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"


uint8_t CAN1ErrorFlag = 0;
uint8_t CAN2ErrorFlag = 0;
/*
===============================================================
                        信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *CANSendPeriodSem;
//定义互斥型信号量用于管理CAN发送资源
OS_EVENT *CANSendMutex;

canMsg_t CAN1Message[MAX_CAN_CAPACITY] = {0};
canMsg_t CAN2Message[MAX_CAN_CAPACITY] = {0};
int CAN1BuffFront = 0;
int CAN1BuffRear = 0;
int CAN2BuffFront = 0;
int CAN2BuffRear = 0;

int CAN1MsgCapacity = 0;
int CAN2MsgCapacity = 0;

extern robot_t gRobot;

static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];
static  OS_STK  CANSendTaskStk[CAN_SEND_TASK_STK_SIZE];

void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;          /*防止警告...*/

	/*创建信号量*/
    CANSendPeriodSem				=	OSSemCreate(0);
	
	//创建互斥型信号量
	CANSendMutex            =   OSMutexCreate(9,&os_err);

    /*创建任务*/
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,				/*初始化任务*/
	                      	(void          * ) 0,
													(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],
													(INT8U           ) Config_TASK_START_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) CANSendTask,
	                      	(void          * ) 0,
													(OS_STK        * )&CANSendTaskStk[CAN_SEND_TASK_STK_SIZE-1],
													(INT8U           ) CAN_SEND_TASK_PRIO);


}

/*
===============================================================
                        初始化任务
===============================================================
*/
uint32_t canErrCode = 0;
void ConfigTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//定时器初始化
	TIM_Init(TIM2, 499, 83, 0, 0);   //0.5ms主定时器
	GPIO_Init_Pins(GPIOA,GPIO_Pin_8,GPIO_Mode_OUT);
	GPIO_Init_Pins(GPIOA,GPIO_Pin_9,GPIO_Mode_OUT);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);

	
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

	OSTaskSuspend(OS_PRIO_SELF);
}

void CANSendTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
//    OSSemSet(CANSendPeriodSem, 0, &os_err);
	while(1)
	{
		
		//当检测到总线关闭时，重新初始化CAN
		if(CAN1ErrorFlag == CAN_BUS_OFF)
		{
			CAN_DeInit(CAN1);
			CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
			CAN1ErrorFlag = CAN_BUS_OK;
		}
		if(CAN2ErrorFlag == CAN_BUS_OFF)
		{
			CAN_DeInit(CAN2);
			CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
			CAN2ErrorFlag = CAN_BUS_OK;
		}

		//当队列非空时发送数据
		if(CAN1BuffFront!=CAN1BuffRear)
		{
			if(CAN1BuffFront + 1 == MAX_CAN_CAPACITY)
			{
				CAN1BuffFront = (CAN1BuffFront + 1)%MAX_CAN_CAPACITY;
				CAN_TxMsg(CAN2,CAN1Message[MAX_CAN_CAPACITY-1].canSendId, CAN1Message[MAX_CAN_CAPACITY-1].message,8);
			}
			else
			{
				CAN1BuffFront++;
				CAN_TxMsg(CAN2,CAN1Message[CAN1BuffFront-1].canSendId, CAN1Message[CAN1BuffFront-1].message,8);
			}
		}
		
		if(CAN2BuffFront!=CAN2BuffRear)
		{
			if(CAN2BuffFront + 1 == MAX_CAN_CAPACITY)
			{
				CAN2BuffFront = (CAN2BuffFront + 1)%MAX_CAN_CAPACITY;
				CAN_TxMsg(CAN1,CAN2Message[MAX_CAN_CAPACITY-1].canSendId, CAN2Message[MAX_CAN_CAPACITY-1].message,8);
			}
			else
			{
				CAN2BuffFront++;
				CAN_TxMsg(CAN1,CAN2Message[CAN2BuffFront-1].canSendId, CAN2Message[CAN2BuffFront-1].message,8);
			}
		}	
	}
}

