#include <includes.h>
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
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
#include "gasvalvecontrol.h"
#include "movebase.h"
#include "wifi.h"

/*
===============================================================
                        �ź�������
===============================================================
*/
OS_EVENT *PeriodSem;

void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;          /*��ֹ����...*/
	
	/*�����ź���*/
    PeriodSem				=	OSSemCreate(0);

    /*��������*/	
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,				/*��ʼ������*/
	                      	(void          * ) 0,							
													(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],		
													(INT8U           ) Config_TASK_START_PRIO  );	
						
													
	os_err = OSTaskCreate(	(void (*)(void *)) WalkTask,					
	                      	(void          * ) 0,							
													(OS_STK        * )&WalkTaskStk[Walk_TASK_STK_SIZE-1],		
													(INT8U           ) Walk_TASK_PRIO  );													
}

/*
===============================================================
                        ��ʼ������
===============================================================
*/
void ConfigTask(void)
{	
	CPU_INT08U  os_err;	
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	
	//��ʱ����ʼ��
	TIM_Init(TIM2, 999, 839, 0, 0);		//�����ڶ�ʱ10ms
	TIM_Init(TIM3, 99 , 839, 0, 0);     //�������ٶ�1msһ��
	TIM_Delayms(TIM5, 1500);	
	
	//���ڳ�ʼ��
//	USART1_Init(115200);
//	UART5_Init(115200);		//����������	
	USART3_Init(115200);	
	TIM_Delayms(TIM5, 10000);	
	
//	atk_8266_init();
	KeyInit();
	PhotoelectricityInit();
	BeepInit();
	
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

    //�����ʼ����ʹ��
//	elmo_Init();
//	
//	elmo_Enable(1);
//	elmo_Enable(2);
//	elmo_Enable(3);

//	elmo_Enable(4);
//	elmo_Enable(5);
//	elmo_Enable(6);
//	elmo_Enable(7);
//	elmo_Enable(8);
//	elmo_Enable(9);
//	elmo_Enable(10);
//	elmo_Enable(11);
	
//	Vel_cfg(1, 100000, 100000);
//	Vel_cfg(2, 100000, 100000);
//	Vel_cfg(3, 100000, 100000);
	
//	Vel_cfg(4, 100000, 100000);
//	
//	Pos_cfg(5, 300000, 300000, 12000);
//	
//	Pos_cfg(6,5000,5000,30000);//����
//	Pos_cfg(7,5000,5000,30000);//����
//	Pos_cfg(8,5000,5000,30000);//����
//	
//	Pos_cfg(9,250000,250000,15000);//����(9,250000,250000,25000)
//	
//	Vel_cfg(10,300000,300000);	//�� ���� 
//	Vel_cfg(11,300000,300000);	//
	
	TIM_Delayms(TIM5, 50);

//	ClampOpen();
	
	OSTaskSuspend(OS_PRIO_SELF);
}

/*
===============================================================
                        �����ƶ�����
===============================================================
*/
uint8_t launcherStatus = 0;
extern int32_t launcherPos;
//static uint8_t launcherCounter = 0;
extern float speed;
extern float position[4];
float cl_angle(float ex, float act);
static uint8_t timeCounter = 0;
//static float lastX = 0;
//static float presentX = 0;
//static float robotSpeed = 0;
//static uint8_t stopCounter = 0;
int a;

typedef enum
{
	goToLoadingArea,
	load,
	goToLaunchingArea,
	launch
}Status_t;

Status_t status = goToLoadingArea;

void WalkTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
    OSSemSet(PeriodSem, 0, &os_err);

	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);

		a = GetVel();
		a = a;
//		GasValveControl(1 , 1 , 1);//��������
//		GasValveControl(1 , 2 , 1);//����
//		GasValveControl(1 , 3 , 1);//����
//		GasValveControl(1 , 4 , 1);//����
//		GasValveControl(1 , 5 , 1);//��
//		GasValveControl(1 , 6 , 1);//��
//		GasValveControl(1 , 8 , 1);//���Ϸ�
	
		switch (status)
		{
			//�ӳ���������װ����
			case goToLoadingArea:
//				VelCrl(1,5000);
//				VelCrl(2,5000);
//				VelCrl(3,5000);
			
//			    Move(-1000.0f, 0.0f, -12686.0f, 1000.0f);
//			    
//			    if (GetPosX() <= -12686.0f)
//				{
//					LockWheel();
//					status++;
//				}
//			    if (PHOTOSENSORLEFTUP || PHOTOSENSORRIGHTUP)
//				{
//					BEEP_ON;
//				}
//				else
//				{
//					BEEP_OFF;
//				}
				break;
				
			//װ�ط���
			case load:
//				LockWheel();
//                
//  			    timeCounter++;
//			    
//			    if (timeCounter >= 100)
//				{
//					timeCounter = 0;
//					status++;
//				}
//				ClampClose();
//				if (KEYSWITCHLEFT && KEYSWITCHRIGHT)
//				{
//					status ++;
//				}
				break;
			
//            //��װ������������				
			case goToLaunchingArea:
//                Move(1000.0f, -12686.0f, /*-6343.0f*/0.0f, 1000.0f);
//			
//			    if (GetPosX() >= /*-6343.0f*/0.0f)
//				{
//					LockWheel();
//					status++;
//				}
				break;
//			
//			//�������
			case launch:
				break;
			
//			default:
				break;		
//		}

        //���� or wifi�������		
//		ReadActualPos(6);
//		ReadActualPos(7);
//		ReadActualPos(8);
//		ReadActualPos(9);
//		ReadActualVel(9);
		
//		u5_printf("%d\r\n",(int)(1000*GetPosX()));
		
//		lastXCounter++;
//		if (lastXCounter == 10)
//		{
//			presentX = GetPosX();
//			robotSpeed = (presentX - lastX) / 100.0f;
//			USART_OUT(UART5, (const uint8_t *)"%d.%d   %d   %d  %d\r\n",
//				      (int)robotSpeed, (int)((robotSpeed - (int)robotSpeed) * 1000),
//				      (int)presentX, (int)GetPosY(), (int)GetAngle());
//			lastX = presentX;
//			lastXCounter = 0;
//		}
		
//		u5_printf("Roll*10 = %d, Pitch*10 = %d, Yaw*10 = %d   LauncherVel = %d   EncVel0 = %d   EncVel1 = %d\r\n", 
//							(int)(position[1] * 10), (int)(position[2] * 10), (int)(position[0] * 10), 
//								speed, GetEncVel(0), GetEncVel(1));

		
//		//����һ��������˻�
//		if (launcherStatus == 1)
//		{
//			launcherCounter++;
//			if (launcherCounter == 100)
//			{
//				launcherPos += 2048;
//				PosCrl(9, 0, launcherPos);
//				launcherStatus = 0;
//				launcherCounter = 0;
//			}
//		}
		}
	} 
}	
