#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "usart.h"

#define BUFF_CAPACITY 500
uint8_t UART5SendBuff[BUFF_CAPACITY] = {0};

void UART5DMAInit(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	UART5_Init(115200);
	
	DMA_DeInit(DMA1_Stream7);
	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}
		
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;     
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(UART5->DR); 				// peripheral address, = & USART3->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UART5SendBuff;							// memory address to save DMA data
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;               	// data dirction: peripheral to memory, ie receive maggage from peripheral
	DMA_InitStructure.DMA_BufferSize = BUFF_CAPACITY;                    		//the buffer size, in data unit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8 bit data
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        //8 bit data  32??MCU?1?half-word?16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream7, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream7, ENABLE);
		
	DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);  	

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);
}

void USART5_DMA_Send(void)
{
	DMA_Cmd(DMA1_Stream7, ENABLE);
	while(DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7) == RESET);
	DMA_Cmd(DMA1_Stream7, DISABLE);
	DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);
}

void DMA1_Stream7_IRQHandler(void) 
{
  if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7) != RESET)  
  {
        DMA_Cmd(DMA1_Stream7, DISABLE ); 
        DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);
        DMA_SetCurrDataCounter(DMA1_Stream7,sizeof(UART5SendBuff));
  }
}

