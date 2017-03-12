#include  "can.h"
#include "gasvalvecontrol.h"
#include "gpio.h"

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
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x0001 ;					 // standard identifier=0
	TxMessage.ExtId = 0x0001 ;				 	 // extended identifier=StdId
	TxMessage.IDE = CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR = CAN_RTR_Data;			 	 // the type of frame for the message that will be transmitted
	TxMessage.DLC = 1;
	
	data = boardNum<<5|valveNum<<1|valveState;
	
	TxMessage.Data[0] = data;

	mbox = CAN_Transmit(CAN2, &TxMessage);         
	while ((CAN_TransmitStatus(CAN2, mbox) != CAN_TxStatus_Ok));
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
	GasValveControl(LEFT_RELOAD_RESET_BOARD_ID , LEFT_RELOAD_RESET_IO_ID , 0);//左推盘收
	GasValveControl(LEFT_RELOAD_BOARD_ID , LEFT_RELOAD_IO_ID , 1);//左推	
}
//左上弹复位
void LeftBack(void)
{
	GasValveControl(LEFT_RELOAD_RESET_BOARD_ID , LEFT_RELOAD_RESET_IO_ID , 1);//左推盘收
	GasValveControl(LEFT_RELOAD_BOARD_ID , LEFT_RELOAD_IO_ID , 0);//左推	
}
//右上弹
void RightPush(void)
{
	GasValveControl(RIGHT_RELOAD_RESET_BOARD_ID , RIGHT_RELOAD_RESET_IO_ID , 0);//右收
	GasValveControl(RIGHT_RELOAD_BOARD_ID ,RIGHT_RELOAD_IO_ID , 1);//右推
}
//右上弹复位
void RightBack(void)
{
	GasValveControl(RIGHT_RELOAD_RESET_BOARD_ID , RIGHT_RELOAD_RESET_IO_ID , 1);//右收
	GasValveControl(RIGHT_RELOAD_BOARD_ID ,RIGHT_RELOAD_IO_ID , 0);//右推
}
//左枪发射
void LeftShoot(void)
{
	GasValveControl(LEFT_SHOOT_BOARD_ID,LEFT_SHOOT_IO_ID, 1);
}
//左枪发射复位
void LeftShootReset(void)
{
	GasValveControl(LEFT_SHOOT_BOARD_ID,LEFT_SHOOT_IO_ID, 0);
}
//右枪发射
void RightShoot(void)
{
	GasValveControl(RIGHT_SHOOT_BOARD_ID,RIGHT_SHOOT_IO_ID, 1);
}
//右枪发射复位
void RightShootReset(void)
{
	GasValveControl(RIGHT_SHOOT_BOARD_ID,RIGHT_SHOOT_IO_ID, 0);
}
//上枪发射
void UpperShoot(void)
{
	GasValveControl(UPPER_SHOOT_BOARD_ID,UPPER_SHOOT_IO_ID, 1);
}
//上枪发射复位
void UpperShootReset(void)
{
	GasValveControl(UPPER_SHOOT_BOARD_ID,UPPER_SHOOT_IO_ID, 0);
}

