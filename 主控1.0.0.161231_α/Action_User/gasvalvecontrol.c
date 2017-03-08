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
		GasValveControl(1 , 6 , 0);
		GasValveControl(2 , 12 , 1);
	}
}
//夹子关
void ClampClose(void)
{
 	GasValveControl(1 , 6 , 1);
	GasValveControl(2 , 12 , 0);
}
//夹子翻
void ClampRotate(void)
{
		GasValveControl(1 , 8 , 1);//往上翻
}
//夹子复位
void ClampReset(void)
{
	GasValveControl(1 , 8 , 0);//往上翻
}

//左推盘

void LeftPush(void)
{
	GasValveControl(1 , 1 , 0);//左推盘收
	GasValveControl(1 , 2 , 1);//左推	
}
void LeftBack(void)
{
	GasValveControl(1 , 1 , 1);//左推盘收
	GasValveControl(1 , 2 , 0);//左推
}
//右推盘
void RightPush(void)
{
	GasValveControl(1 , 3 , 0);//右收
	GasValveControl(1 , 4 , 1);//右推
}
void RightBack(void)
{
	GasValveControl(1 , 3 , 1);//右收
	GasValveControl(1 , 4 , 0);//右推
}

