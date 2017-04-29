/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H
#define __DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void UART5DMAInit(void);
void USART5_DMA_Send(void);
void DMA1_Stream7_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H */
