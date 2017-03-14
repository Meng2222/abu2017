#include "stm32f4xx_can.h"

#define CAN_ENABLE 1

#define INVALID_CANSEND_MAILBOX  10
#define CAN_SEND_OK 1
#define CAN_SEND_ERR -1

void CAN_Config(CAN_TypeDef* CANx, 
				uint32_t CAN_BaudRate,
				GPIO_TypeDef * GPIOx,
				uint16_t CAN_RxPin,
				uint16_t CAN_TxPin);

uint8_t CAN_RxMsg(CAN_TypeDef* CANx,
				  uint32_t * StdId,
				  uint8_t * buf,
				  uint8_t len);

uint8_t CAN_TxMsg(CAN_TypeDef* CANx,
				  uint32_t StdId,
				  uint8_t * buf,
				  uint8_t len);
/**
  * @brief  利用操作系统互斥型信号量管理CAN发送函数
  * @param  CANx:  CANx, where x can be 1,2.
  * @param  TxMessage:   a array you want to transmit.
  * @retval CAN_SEND_OK(whose value is 1), if receive successful
  * @retval CAN_SEND_ERR(which value is -1), if receive unsuccessful
**/
int OSCANSendCmd(CAN_TypeDef* CANx, CanTxMsg* TxMessage);



