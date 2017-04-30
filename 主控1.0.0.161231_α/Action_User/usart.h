#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"


#define USART5_MAX_RECV_LEN		400					//最大接收缓存字节数
#define USART5_MAX_SEND_LEN		400					//最大发送缓存字节数
#define USART5_RX_EN 			1					//0,不接收;1,接收.

extern u8  USART5_RX_BUF[USART5_MAX_RECV_LEN]; 		//接收缓冲,最大USART3_MAX_RECV_LEN字节
extern u8  USART5_TX_BUF[USART5_MAX_SEND_LEN]; 		//发送缓冲,最大USART3_MAX_SEND_LEN字节
extern u16 USART5_RX_STA;   						//接收数据状态

void u5_printf(char* fmt, ...);

void USART1_Init(uint32_t BaudRate);
void UART4_Init(uint32_t BaudRate);
void USART3_Init(uint32_t BaudRate);
void UART5_Init(uint32_t BaudRate);
void USART6_Init(uint32_t BaudRate);

void USART_OUT(USART_TypeDef* USARTx, const uint8_t *Data,...);
char *itoa(int value, char *string, int radix);
void UART5_OUT(const uint8_t *Data, ...);

#endif

