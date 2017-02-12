#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"


#define USART5_MAX_RECV_LEN		400					//�����ջ����ֽ���
#define USART5_MAX_SEND_LEN		400					//����ͻ����ֽ���
#define USART5_RX_EN 			1					//0,������;1,����.

extern u8  USART5_RX_BUF[USART5_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern u8  USART5_TX_BUF[USART5_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
extern u16 USART5_RX_STA;   						//��������״̬

void u5_printf(char* fmt, ...);

void USART1_Init(uint32_t BaudRate);
void USART3_Init(uint32_t BaudRate);
void UART5_Init(uint32_t BaudRate);
void USART_OUT(USART_TypeDef* USARTx, const uint8_t *Data,...);
char *itoa(int value, char *string, int radix);

#endif

