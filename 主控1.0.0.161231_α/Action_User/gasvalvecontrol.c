#include  "can.h"
#include "gasvalvecontrol.h"
#include "gpio.h"
#include "robot.h"
#include "usart.h"
extern robot_t gRobot;
/**
* @brief  气阀控制
* @param  boardNum：气阀板号
* @param  valveNum：气阀号
* @param  valveState： 气阀状态，0为关，1为开
* @author ACTION
*/
void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState)
{
	uint8_t data = 0x00;
//	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x0001 ;					 // standard identifier=1
	TxMessage.ExtId = 0x0001 ;				 	 // extended identifier=StdId
	TxMessage.IDE = CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR = CAN_RTR_Data;			 	 // the type of frame for the message that will be transmitted
	TxMessage.DLC = 1;
	
	data = boardNum<<5|valveNum<<1|valveState;
	
	TxMessage.Data[0] = data;

	OSCANSendCmd(CAN2, &TxMessage);
//	mbox = CAN_Transmit(CAN2, &TxMessage);         
//	while ((CAN_TransmitStatus(CAN2, mbox) != CAN_TxStatus_Ok));
}
//夹子开
void ClampOpen(void)
{
	if (!KEYSWITCH)
	{
		GasValveControl(CLAMP_CLOSE_BOARD_ID , CLAMP_CLOSE_IO_ID , 0);
		GasValveControl(CLAMP_OPEN_BOARD_ID , CLAMP_OPEN_IO_ID , 1);
	}
}
//夹子关
void ClampClose(void)
{
		GasValveControl(CLAMP_CLOSE_BOARD_ID , CLAMP_CLOSE_IO_ID , 1);
		GasValveControl(CLAMP_OPEN_BOARD_ID , CLAMP_OPEN_IO_ID , 0);
}
//夹子翻
void ClampRotate(void)
{
		GasValveControl(CLAMP_ROTATE_BOARD_ID , CLAMP_ROTATE_IO_ID , 1);//往上翻
}
//夹子翻转复位
void ClampReset(void)
{
	GasValveControl(CLAMP_ROTATE_BOARD_ID , CLAMP_ROTATE_IO_ID , 0);//往上翻
}

//左上弹
void LeftPush(void)
{
	UART5_OUT((uint8_t *)"leftPush%d\r\n",gRobot.reloadTimer);
	GasValveControl(LEFT_RELOAD_RESET_BOARD_ID , LEFT_RELOAD_RESET_IO_ID , 0);		
	GasValveControl(LEFT_RELOAD_BOARD_ID , LEFT_RELOAD_IO_ID , 1);
}
//左上弹复位
void LeftBack(void)
{
	UART5_OUT((uint8_t *)"leftBack%d\r\n",gRobot.reloadTimer);
	GasValveControl(LEFT_RELOAD_BOARD_ID , LEFT_RELOAD_IO_ID , 0);
	GasValveControl(LEFT_RELOAD_RESET_BOARD_ID , LEFT_RELOAD_RESET_IO_ID , 1);	
}
//左上弹保持
void LeftHold(void)
{
	GasValveControl(LEFT_RELOAD_BOARD_ID , LEFT_RELOAD_IO_ID , 0);
	GasValveControl(LEFT_RELOAD_RESET_BOARD_ID , LEFT_RELOAD_RESET_IO_ID , 0);	
}
//右上弹
void RightPush(void)
{
	UART5_OUT((uint8_t *)"RightPush%d\r\n",gRobot.reloadTimer);
	GasValveControl(RIGHT_RELOAD_RESET_BOARD_ID , RIGHT_RELOAD_RESET_IO_ID , 0);		
	GasValveControl(RIGHT_RELOAD_BOARD_ID ,RIGHT_RELOAD_IO_ID , 1);
}
//右上弹复位
void RightBack(void)
{
	UART5_OUT((uint8_t *)"RightBack%d\r\n",gRobot.reloadTimer);
	GasValveControl(RIGHT_RELOAD_BOARD_ID ,RIGHT_RELOAD_IO_ID , 0);
	GasValveControl(RIGHT_RELOAD_RESET_BOARD_ID , RIGHT_RELOAD_RESET_IO_ID , 1);		
}
//右上弹保持
void RightHold(void)
{
	GasValveControl(RIGHT_RELOAD_BOARD_ID , RIGHT_RELOAD_IO_ID , 0);
	GasValveControl(RIGHT_RELOAD_RESET_BOARD_ID , RIGHT_RELOAD_RESET_IO_ID , 0);	
}
//左枪发射
void LeftShoot(void)
{
	UART5_OUT((uint8_t *)"LeftShoot%d\r\n",gRobot.shootTimer);
	GasValveControl(LEFT_SHOOT_BOARD_ID,LEFT_SHOOT_IO_ID, 1);
}
//左枪发射复位
void LeftShootReset(void)
{
	UART5_OUT((uint8_t *)"LeftShootReset%d\r\n",gRobot.shootTimer);
	GasValveControl(LEFT_SHOOT_BOARD_ID,LEFT_SHOOT_IO_ID, 0);
}
//右枪发射
void RightShoot(void)
{
	UART5_OUT((uint8_t *)"RightShoot%d\r\n",gRobot.shootTimer);
	GasValveControl(RIGHT_SHOOT_BOARD_ID,RIGHT_SHOOT_IO_ID, 1);
}
//右枪发射复位
void RightShootReset(void)
{
	UART5_OUT((uint8_t *)"RightShootReset%d\r\n",gRobot.shootTimer);
	GasValveControl(RIGHT_SHOOT_BOARD_ID,RIGHT_SHOOT_IO_ID, 0);
}
//上枪发射
void UpperShoot(void)
{
	UART5_OUT((uint8_t *)"UpperShoot%d\r\n",gRobot.shootTimer);
	GasValveControl(UPPER_SHOOT_BOARD_ID,UPPER_SHOOT_IO_ID, 1);
}
//上枪发射复位
void UpperShootReset(void)
{
	UART5_OUT((uint8_t *)"UpperShootReset%d\r\n",gRobot.shootTimer);
	GasValveControl(UPPER_SHOOT_BOARD_ID,UPPER_SHOOT_IO_ID, 0);
}

