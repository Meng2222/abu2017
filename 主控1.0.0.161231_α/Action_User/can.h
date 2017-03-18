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
uint8_t CAN1TxMsgQueueRQ(CanTxMsg* TxMessage);


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
uint8_t CAN1_TxMsgSendQueueHead(void);




