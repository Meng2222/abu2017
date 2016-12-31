#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"

void USART1_Init(uint32_t BaudRate);
void USART3_Init(uint32_t BaudRate);
void UART5_Init(uint32_t BaudRate);
void USART_OUT(USART_TypeDef* USARTx,const uint8_t *Data,...);
char *itoa(int value, char *string, int radix);
void setAngle(float val);
float getAngle(void);

#endif

