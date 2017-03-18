/**
  ******************************************************************************
  * @file    can.c
  * @author  Calcus Lee
  * @version V1.0.1
  * @date    9-August-2013
  * @brief   functions of CAN
  ******************************************************************************
  * @attention
  * 1.the default CAN mode is normal
  * 2.by default, the CAN Rx Interrupt is ON
  * 3.CAN_IT_FMP0, CAN_IT_FMP1 IRQ is auto cleared.
  ******************************************************************************
**/

#include "can.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "String.h"
#include "ucos_ii.h"
#include "cpu.h"
#include "timer.h"
#include "gpio.h"

extern OS_EVENT *CANSendMutex;

/**
  * @brief  Initialize the CANx as encoder
  * @param  CANx:  CANx, where x can be 1,2
  * @param  CAN_BaudRate:  CAN BaudRate, the entity is Kb, its value is one of following values:10,20,50,10,125,250,500,1000
  * @param  GPIOx: The specific pins you want to select in group GPIOx.
			ref the datasheet->(Pinouts and pin description) Pin AF to identity GPIOx's x.
  * @param  CAN_RxPin
  * @param  CAN_TxPin
			@note   do not use this function to inialize CAN pin, PI10, PH13
  * @retval None
  * @author Calcus Lee
**/
void CAN_Config(CAN_TypeDef* CANx, 
				uint32_t CAN_BaudRate,
				GPIO_TypeDef * GPIOx,
				uint16_t CAN_RxPin,
				uint16_t CAN_TxPin)
{
	GPIO_InitTypeDef       GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  	   NVIC_InitStructure;
	uint8_t CAN_RxSource=0;
	uint8_t CAN_TxSource=0;
    uint8_t GPIO_AF_CANx=0;

  /* CAN GPIOs configuration */
	
	//确定CAN_RxPin
	
	switch(CAN_RxPin)
	{    
		case GPIO_Pin_0:
		{
           CAN_RxSource=GPIO_PinSource0;
		   break;
		}
		case GPIO_Pin_1:
		{
           CAN_RxSource=GPIO_PinSource1;
		   break;
		}
		case GPIO_Pin_2:
		{
           CAN_RxSource=GPIO_PinSource2;
		   break;
		}
		case GPIO_Pin_3:
		{
           CAN_RxSource=GPIO_PinSource3;
		   break;
		}
        case GPIO_Pin_4:
		{
           CAN_RxSource=GPIO_PinSource4;
		   break;
		}
		case GPIO_Pin_5:
		{
           CAN_RxSource=GPIO_PinSource5;
		   break;
		}
		case GPIO_Pin_6:
		{
           CAN_RxSource=GPIO_PinSource6;
		   break;
		}
		case GPIO_Pin_7:
		{
           CAN_RxSource=GPIO_PinSource7;
		   break;
		}
		case GPIO_Pin_8:
		{
           CAN_RxSource=GPIO_PinSource8;
		   break;
		}
		case GPIO_Pin_9:
		{
           CAN_RxSource=GPIO_PinSource9;
		   break;
		}
		case GPIO_Pin_10:
		{
           CAN_RxSource=GPIO_PinSource10;
		   break;
		}
		case GPIO_Pin_11:
		{
           CAN_RxSource=GPIO_PinSource11;
		   break;
		}
		case GPIO_Pin_12:
		{
           CAN_RxSource=GPIO_PinSource12;
		   break;
		}
		case GPIO_Pin_13:
		{
           CAN_RxSource=GPIO_PinSource13;
		   break;
		}
		case GPIO_Pin_14:
		{
           CAN_RxSource=GPIO_PinSource14;
		   break;
		}
		case GPIO_Pin_15:
		{
           CAN_RxSource=GPIO_PinSource15;
		   break;
		}
		
		default: break;
	}

	//确定CAN_TxPin
	switch(CAN_TxPin)
	{    
		case GPIO_Pin_0:
		{
           CAN_TxSource=GPIO_PinSource0;
		   break;
		}
		case GPIO_Pin_1:
		{
           CAN_TxSource=GPIO_PinSource1;
		   break;
		}
		case GPIO_Pin_2:
		{
           CAN_TxSource=GPIO_PinSource2;
		   break;
		}
		case GPIO_Pin_3:
		{
           CAN_TxSource=GPIO_PinSource3;
		   break;
		}
        case GPIO_Pin_4:
		{
           CAN_TxSource=GPIO_PinSource4;
		   break;
		}
		case GPIO_Pin_5:
		{
           CAN_TxSource=GPIO_PinSource5;
		   break;
		}
		case GPIO_Pin_6:
		{
           CAN_TxSource=GPIO_PinSource6;
		   break;
		}
		case GPIO_Pin_7:
		{
           CAN_TxSource=GPIO_PinSource7;
		   break;
		}
		case GPIO_Pin_8:
		{
           CAN_TxSource=GPIO_PinSource8;
		   break;
		}
		case GPIO_Pin_9:
		{
           CAN_TxSource=GPIO_PinSource9;
		   break;
		}
		case GPIO_Pin_10:
		{
           CAN_TxSource=GPIO_PinSource10;
		   break;
		}
		case GPIO_Pin_11:
		{
           CAN_TxSource=GPIO_PinSource11;
		   break;
		}
		case GPIO_Pin_12:
		{
           CAN_TxSource=GPIO_PinSource12;
		   break;
		}
		case GPIO_Pin_13:
		{
           CAN_TxSource=GPIO_PinSource13;
		   break;
		}
		case GPIO_Pin_14:
		{
           CAN_TxSource=GPIO_PinSource14;
		   break;
		}
		case GPIO_Pin_15:
		{
           CAN_TxSource=GPIO_PinSource15;
		   break;
		}
		
		default: break;
	}
  /* CANx clock source enable */ 
  switch((uint32_t)CANx)
  {
	//CANs on APB1
    case CAN1_BASE: 
    {
		GPIO_AF_CANx=GPIO_AF_CAN1;
		CAN_FilterInitStructure.CAN_FilterNumber=0;	    //Filter 0
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		break;
    }
	case CAN2_BASE: 
    {
		GPIO_AF_CANx=GPIO_AF_CAN2;
		CAN_FilterInitStructure.CAN_FilterNumber=14;	//Filter 14
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
		break;
    }
	
    default: break;
  }
  /* Enable GPIOx, clock */  
	switch((uint32_t)GPIOx)
	{
		case GPIOA_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		break;
		}
		case GPIOB_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
		}
		case GPIOC_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
		}
		case GPIOD_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		break;
		}
		case GPIOE_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		break;
		}
		case GPIOF_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
		break;
		}
		case GPIOG_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
		break;
		}
		case GPIOH_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
		break;
		}
		case GPIOI_BASE: 
		{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
		break;
		}

		default: break;
	}
	

	/* Connect CAN pins to AF */
	GPIO_PinAFConfig(GPIOx, CAN_RxSource, GPIO_AF_CANx);  
	GPIO_PinAFConfig(GPIOx, CAN_TxSource, GPIO_AF_CANx); 

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = CAN_RxPin | CAN_TxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	
	/* CAN register init */
	CAN_DeInit(CANx);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;         //time triggered communication mode
	CAN_InitStructure.CAN_ABOM = DISABLE;         //automatic bus-off management
	CAN_InitStructure.CAN_AWUM = DISABLE;         //automatic wake-up mode
	CAN_InitStructure.CAN_NART = DISABLE;         //non-automatic retransmission mode
	CAN_InitStructure.CAN_RFLM = DISABLE;         //Receive FIFO Locked mode
	CAN_InitStructure.CAN_TXFP = DISABLE;         //transmit FIFO priority
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //CAN operating mode
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;   // keep CAN_SJW == 1, never change it
  CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq; //max=16
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq; //max=8
	/* CAN Baudrate =APB1_CLK/((CAN_SJW_tq+CAN_BS1_tq+CAN_BS2_tq)*CAN_Prescaler) */ //?
    switch(CAN_BaudRate)
	{			
		case 10:
		{
			CAN_InitStructure.CAN_Prescaler = 200;
			break;
		}
		case 20:
		{
			CAN_InitStructure.CAN_Prescaler = 100;
			break;
		}
		case 50:
		{
			CAN_InitStructure.CAN_Prescaler = 40;	
			break;
		}
		case 100:
		{
			CAN_InitStructure.CAN_Prescaler = 20;			
			break;
		}
		case 125:
		{
			CAN_InitStructure.CAN_Prescaler = 16;			
			break;
		}
		case 250:
		{
			CAN_InitStructure.CAN_Prescaler = 8;	
			break;
		}
		case 500:
		{
			CAN_InitStructure.CAN_Prescaler = 4;			
			break;
		}
		case 1000:
		{
			CAN_InitStructure.CAN_Prescaler = 2;	
			break;
		}

		default: break;
	}
    CAN_Init(CANx, &CAN_InitStructure);
	
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32 Bit
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;               //32 Bis ID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;           //32 Bit Mask
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;           //activate Filter
	CAN_FilterInit(&CAN_FilterInitStructure);                        //intialize Filter

	/* Enable FIFO 0 message pending Interrupt */


  switch((uint32_t)CANx)
  {
	//CANs on APB1
    case CAN1_BASE: 
    {
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
		
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		CAN_ITConfig(CANx, CAN_IT_TME, ENABLE);
		
		break;
    }
	case CAN2_BASE: 
    {
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
		
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		CAN_ITConfig(CANx, CAN_IT_TME, ENABLE);
		break;
    }
	
    default: break;
  }
		
}	



/**
  * @brief  transmit an array 
  * @param  CANx:  CANx, where x can be 1,2.
  * @param  StdId: the StdId you want to select as StdId.
  * @param  Msg:   a pointer that point the array you want to transmit.
  * @param  len:   the length of the the array that you want to transmit.
  * @retval 1, if transmit successful. 
  * @author Calcus Lee
**/
uint8_t CAN_TxMsg(CAN_TypeDef* CANx,
				  uint32_t StdId,
				  uint8_t * Msg,
				  uint8_t len)
{
	uint8_t mbox;
	uint16_t i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=StdId;					 // standard identifier=0
	TxMessage.ExtId=StdId;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=len;						 // 发送两帧信息
	for(i=0;i<len;i++)
		TxMessage.Data[i]=Msg[i];			 // 第一帧信息 
  
	mbox= CAN_Transmit(CANx, &TxMessage);         	
	i=0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok));
	return 1;		
}


/**
  * @brief  Receive an array 
  * @param  CANx:  CANx, where x can be 1,2.
  * @param  StdId: the StdId you want to select as StdId.
  * @param  Msg:   a array you want to transmit.
  * @param  len:   the length of the the array that you want to transmit.
  * @retval 1, if receive successful
  * @author Calcus Lee
**/
uint8_t CAN_RxMsg(CAN_TypeDef* CANx,
				  uint32_t * StdId,
				  uint8_t * buf,
				  uint8_t len)
{
	uint8_t i = 0;
	CanRxMsg RxMessage;
    if(CAN_MessagePending(CANx, CAN_FIFO0) == 0)
	{
		return 0;		//if there is no data, get out of this function
	}
    CAN_Receive(CANx, CAN_FIFO0, &RxMessage);				//reveive data	
    for(i = 0; i < len; i++)
	{
		buf[i] = RxMessage.Data[i];
	}
	*StdId = RxMessage.StdId;
	
	return 1;
}

/**
  * @brief  利用操作系统互斥型信号量管理CAN发送资源，CAN发送函数
  * @param  CANx:  CANx, where x can be 1,2.
  * @param  TxMessage:   a array you want to transmit.
  * @retval CAN_SEND_OK(whose value is 1), if receive successful
  * @retval CAN_SEND_ERR(which value is -1), if receive unsuccessful
**/

int OSCANSendCmd(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
{
	CPU_INT08U os_err;
	uint8_t mailBox = INVALID_CANSEND_MAILBOX;
	int timeOut = 100;
	//等待互斥型信号量
	OSMutexPend(CANSendMutex,0,&os_err);
	//发送CAN消息
	mailBox = CAN_Transmit(CANx,TxMessage);
	//等待发送成功，此处设置了超时限制 100*200us = 20ms
	while(timeOut--)
	{
		TIM_Delayus(TIM5,200);
		if(CAN_TransmitStatus(CANx,mailBox) == CAN_TxStatus_Ok)
		{
			//释放互斥型信号量
			OSMutexPost(CANSendMutex);
			return CAN_SEND_OK;
		}
		else
		{
			continue;
		}
	}
	//释放互斥型信号量
	OSMutexPost(CANSendMutex);
	return CAN_SEND_ERR;
	
}





#define CAN_QUEUE_CAPACITY		6

typedef int8_t CanTxQueueStatus_t;

#define CAN_QUEUE_EMPTY			0  //must be 0 因为初始化为0
#define CAN_QUEUE_NORMAL		1
#define CAN_QUEUE_OVERFLOW		2
#define CAN_QUEUE_UNDERFLOW		-2

typedef struct
{
	int8_t front;
	int8_t rear;
	int8_t qsize;
	CanTxQueueStatus_t status;
	CanTxMsg element[CAN_QUEUE_CAPACITY];
	
}CanTxMsgQueue_t;

static CanTxMsgQueue_t can1TxMsgQueue = {0}, can2TxMsgQueue = {0};

static void CanErrorBeepWarning(void)
{
	for(;;)
	{
		BEEP_ON;
		delay_ms(1500);
		BEEP_OFF;
		delay_ms(1000);
	}
}


/**
  * @brief  Check if the CAN transmit queue is empty or underflow or normal
  * @param  None
  * @retval CAN_QUEUE_OVERFLOW if overflow happen
  *         CAN_QUEUE_UNDERFLOW if underflow happen
  *         CAN_QUEUE_NORMAL   if qsize is smaller than CAN queue capacity
  */
static CanTxQueueStatus_t Can1TxMsgQueueCheckStatus(void)
{
	if(can1TxMsgQueue.qsize < 0 )
	{
		can1TxMsgQueue.status = CAN_QUEUE_UNDERFLOW;
	}
	else if(can1TxMsgQueue.qsize == 0 )
	{
		can1TxMsgQueue.status = CAN_QUEUE_EMPTY;
	}
	else if(can1TxMsgQueue.qsize >= CAN_QUEUE_CAPACITY)
	{
		can1TxMsgQueue.status = CAN_QUEUE_OVERFLOW;
	}
	else
	{
		can1TxMsgQueue.status = CAN_QUEUE_NORMAL;
	}
	return can1TxMsgQueue.status;
}


/**
  * @brief  CAN transmit messege Enqueue 
  * @param  txMsg: pointer to a structure which contains CAN Id, CAN DLC and CAN data.
  * @retval CAN_QUEUE_OVERFLOW   if overflow happen
  *         CAN_QUEUE_UNDERFLOW  if underflow happen
  *         CAN_QUEUE_NORMAL     if qsize is smaller than CAN queue capacity
  */
static int8_t CanTxMsgEnqueue(CanTxMsg* txMsg)
{
	if(can1TxMsgQueue.qsize < CAN_QUEUE_CAPACITY)
	{
		can1TxMsgQueue.qsize++;
		can1TxMsgQueue.rear = (can1TxMsgQueue.rear + 1) % CAN_QUEUE_CAPACITY;
		can1TxMsgQueue.element[can1TxMsgQueue.rear] = *txMsg;
	}
	else{
		CanErrorBeepWarning();
	}
	return can1TxMsgQueue.status;
}



/**
  * @brief  CAN transmit messege Dequeue 
  * @param  Mone
  * @retval a structure which contains CAN Id, CAN DLC and CAN data.
  */
static CanTxMsg CANTxMsgDequeue(void)
{
	CanTxMsg TxMsg;
	if(can1TxMsgQueue.qsize > 0)
	{
		TxMsg = can1TxMsgQueue.element[can1TxMsgQueue.front];
		can1TxMsgQueue.qsize--;
		can1TxMsgQueue.front = (can1TxMsgQueue.front + 1)%CAN_QUEUE_CAPACITY;
	}
	else{
		CanErrorBeepWarning();
	}
	
	return TxMsg;
}

/**
  * @brief  CAN transmit messege queue read front element
  *        此函数读取处于发送队列头处的元素 
  * @note   内部含有对于队列是否为空的判断
  * @param  Mone
  * @retval pointer to a structure which contains CAN Id, CAN DLC and CAN data.
  */
static CanTxMsg* CanTxMsgReadHeadData(void)
{
	if(can1TxMsgQueue.qsize > 0)	
	{
		return &can1TxMsgQueue.element[can1TxMsgQueue.front];
	}
	else
	{
//		return ;//fix me
	}
}


/**
  * @brief  CAN transmit messege queue pop head
  * @param  Mone
  * @retval a structure which contains CAN Id, CAN DLC and CAN data.
  */
static void CAN1TxMsgQueuePopHead(void)
{
	can1TxMsgQueue.qsize--;
	can1TxMsgQueue.front = (can1TxMsgQueue.front + 1) % CAN_QUEUE_CAPACITY;
}

/**
  * @brief  使用CAN1发送数据，执行此函数后首先检查现在的发送队列状态
  *        如果为空，则发送当前消息
  *        如果不为空，则把当前消息放在队尾，并且尝试发送队头的消息，
  *        发送当前消息时：
  *           如果成功放入邮箱，则返回The number of the mailbox 
  *           that is used for transmission
  *           如果没有空邮箱，则把当前消息放入队尾（假设队伍未满）
  *           且返回CAN_TxStatus_NoMailBox
  *        发送队头的消息时：
  *           如果成功放入邮箱，则把队头从队列中去除，并且返回The number
  *           of the mailbox that is used for transmission
  *           如果没有空邮箱， 则队头消息仍保持在队头，不去除，并且返回
  *           CAN_TxStatus_NoMailBox
  * @param  pointer to a structure which contains CAN Id, CAN DLC and CAN data.
  * @retval The number of the mailbox that is used for transmission or
  *         CAN_TxStatus_NoMailBox if there is no empty mailbox.
  */
uint8_t CAN1TxMsgQueueRQ(CanTxMsg* TxMessage)
{
	uint8_t mailBox = INVALID_CANSEND_MAILBOX;
	uint8_t  CAN1QueueStatus = Can1TxMsgQueueCheckStatus();
	switch(CAN1QueueStatus)
	{
		case CAN_QUEUE_EMPTY:
			//发送CAN消息
			mailBox = CAN_Transmit(CAN1, TxMessage);
			switch(mailBox)
			{
				case 0:
					break;
				case 1:
					break;
				case 2:
					break;
				case CAN_TxStatus_NoMailBox:
					if(Can1TxMsgQueueCheckStatus() != CAN_QUEUE_OVERFLOW)
						CanTxMsgEnqueue(TxMessage);
					
					break;
				default:
					break;
			}
			break;
		case CAN_QUEUE_NORMAL:
			CanTxMsgEnqueue(TxMessage);
			TxMessage = &can1TxMsgQueue.element[can1TxMsgQueue.front];
			//发送CAN消息
			mailBox = CAN_Transmit(CAN1, TxMessage);
			switch(mailBox)
			{
				case 0:
					CAN1TxMsgQueuePopHead();
					break;
				case 1:
					CAN1TxMsgQueuePopHead();
					break;
				case 2:
					CAN1TxMsgQueuePopHead();
					break;
				case CAN_TxStatus_NoMailBox:

					break;
				default:
					break;
			}
			break;
			
		default :
			CanErrorBeepWarning();
		break;
	}

	return mailBox;
}


/**
  * @brief  执行此函数后首先检查现在的发送队列状态
  *        如果为空，则退出返回 INVALID_CANSEND_MAILBOX
  *        如果不为空，则尝试发送队头的消息，
  *        发送队头的消息时：
  *           如果成功放入邮箱，则把队头从队列中去除，并且返回The number
  *           of the mailbox that is used for transmission
  *           如果没有空邮箱， 则队头消息仍保持在队头，不去除，并且返回
  *           CAN_TxStatus_NoMailBox
  * @param  None
  * @retval The number of the mailbox that is used for transmission or
  *         CAN_TxStatus_NoMailBox if there is no empty mailbox.
  */
uint8_t CAN1_TxMsgSendQueueHead(void)
{
	int8_t mailBox = INVALID_CANSEND_MAILBOX;
	
	if(can1TxMsgQueue.qsize > 0)
	{
		//发送CAN消息
		mailBox = CAN_Transmit(CAN1, &can1TxMsgQueue.element[can1TxMsgQueue.front]);
		switch(mailBox)
		{
			case 0:
			case 1:
			case 2:
				CAN1TxMsgQueuePopHead();
				break;
			case CAN_TxStatus_NoMailBox:
				break;
			default:
				break;
		}
	}
	return mailBox;
}

//uint8_t CAN2_TxMsgSendQueueHead(void)
//{
//	uint8_t mailBox;
//	
//	if(CAN2_TxMsgQueue.qsize > 0)
//	{
//		//发送CAN消息
//		mailBox = CAN_Transmit(CAN2, &CAN2_TxMsgQueue.element[CAN2_TxMsgQueue.front]);
//		switch(mailBox)
//		{
//			case 0:
//			case 1:
//			case 2:
//				CAN2_TxMsgQueuePopHead();
//				break;
//			case CAN_TxStatus_NoMailBox:
//				break;
//			default:
//				break;
//		}
//	}
//	return mailBox;
//}


