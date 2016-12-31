#include  <includes.h>
#include  <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "walk.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "cylinder.h"
#include "robs.h"
#include "action_math.h"
#include "GET_SET.h"
#include "stm32f4xx_usart.h"
#include "encoder.h"

//////////////////Area of defining semaphore////////////////////////
 OS_EVENT 		*PeriodSem;

void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err; /* prevent warning... */
	
	/******************Create Semaphore***********************/
   PeriodSem				=	OSSemCreate(0);

  /******************Create Task**************************/	
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,					//Initial Task
	                      	(void          * ) 0,							
													(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],		
													(INT8U           ) Config_TASK_START_PRIO  );	
						
													
	os_err = OSTaskCreate(	(void (*)(void *)) WalkTask,					
	                      	(void          * ) 0,							
													(OS_STK        * )&WalkTaskStk[Walk_TASK_STK_SIZE-1],		
													(INT8U           ) Walk_TASK_PRIO  );													
}

void ConfigTask(void)
{	
	CPU_INT08U  os_err;	
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,999,839,0,0);					//主周期定时10ms	
	TIM_Delayms(TIM5,1800);
	
	USART1_Init(115200);
	UART5_Init(115200);		//调试用蓝牙
	
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);

	elmo_Init();
	
//	elmo_Enable(1);
//	elmo_Enable(2);
//	elmo_Enable(3);
//	elmo_Enable(4);
//	elmo_Enable(5);
	elmo_Enable(6);
	elmo_Enable(7);
	elmo_Enable(8);
	elmo_Enable(9);
	elmo_Enable(10);
	elmo_Enable(11);
	
//	Vel_cfg(1,300000,300000);
//	Vel_cfg(2,300000,300000);
//	Vel_cfg(3,300000,300000);
//	Vel_cfg(4,300000,300000);
	
//	Pos_cfg(5,300000,300000,10000);
	
	Pos_cfg(6,5000,5000,30000);//航向
	Pos_cfg(7,5000,5000,30000);//翻滚
	Pos_cfg(8,5000,5000,30000);//俯仰
	
	Pos_cfg(9,250000,250000,10000);//推盘(9,250000,250000,25000)
	
	Vel_cfg(10,300000,300000);	//后 发射 
	Vel_cfg(11,300000,300000);	//
	
	TIM_Delayms(TIM5,50);
	OSTaskSuspend(OS_PRIO_SELF);
}

uint8_t status = 0;
float cl_angle(float ex,float act);	
void WalkTask(void)
{
	CPU_INT08U  os_err;
	os_err=os_err;
  OSSemSet(PeriodSem,0,&os_err);

	while(1)
	{
//		OSSemPend(PeriodSem,0,&os_err);     //
		
//		USART_OUT(UART5,(uint8_t *)"%d\t%d\t%d\t%d\t\n",EncVel[0],(int)(GetSpeed(1)),EncVel[1],(int)(GetSpeed(2)));
	} 
}	
