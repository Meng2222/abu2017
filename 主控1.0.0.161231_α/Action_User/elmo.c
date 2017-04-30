#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

 /**************初始化驱动器********************/
void elmo_Init(CAN_TypeDef* CANx)
{
	uint32_t data[1][2]={0x00000001,00000000};
	CAN_TxMsg(CANx,0x000,(uint8_t*)&data[0],8);
}

/****************使能电机***************************/
void elmo_Enable(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x00004F4D,0x00000001,      //MO  1
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

 	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in Elmo Enable!!!!!!!!!\r\n");
		}
	}
}
/**************失能电机***************************/
void elmo_Disable(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	uint8_t i=0;
	uint32_t data[1][2]={
						0x00004F4D,0x00000000,      //MO  0
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	for(i=0;i<1;i++)
	{

		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				UART5_OUT((uint8_t *)"CAN error in Elmo Disable!!!!!!!!!\r\n");
			}
		}
	}

}
/***************速度环配置*************************/
void Vel_cfg(CAN_TypeDef* CANx, uint8_t ElmoNum,uint32_t acc,uint32_t dec)
{
	 uint8_t i=0;
	 int32_t data[7][2]={
							0x00004D55,0x00000002,    //UM  2
							0x00004653,0x00000000,		//SF  0
							0x00004F4D,0x00000001,    //MO  1
							0x00004341,0x00000000,		//AC  3000000
							0x00004344,0x00000000,		//DC  3000000
							0x00024856,0x00000000,		//VH[2]  15,000,000
							0x00024C56,0x00000000,		//VL[2]  -15,000,000
						 };


	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
	data[3][1]= acc;
	data[4][1]= dec;
	data[5][1]=	 15000000;
	data[6][1]=	 -15000000;

	for(i=0;i<7;i++)
	{

			TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
			TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
			TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
			TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
			TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
			TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
			TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
			TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;

			mbox= CAN_Transmit(CANx, &TxMessage);
			uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				UART5_OUT((uint8_t *)"CAN error in Vel_Cfg!!!!!!!!!\r\n");
			}
		}
	}
}

/************速度控制***************/
void VelCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,int vel)
{
	 uint8_t i=0;
	 uint32_t data[2][2]={

							0x0000564A,0x00000000,		//JV  10000
							0x40004742,0x00000000,    //BG
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
	data[0][1]= vel;
	for(i=0;i<2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;

		mbox= CAN_Transmit(CANx, &TxMessage);
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				UART5_OUT((uint8_t *)"CAN error in VelCrl!!!!!!!!!\r\n");
			}
		}
	}
}

/***************位置环配置*************************/
void Pos_cfg(CAN_TypeDef* CANx, uint8_t ElmoNum,uint32_t acc,uint32_t dec,uint32_t vel)
{
	uint8_t i=0;
	int32_t data[16][2]={
							0x00004D55,0x00000005,    //UM  5
							0x00004653,0x00000000,		//SF  0
							0x00014D58,0x00000000,    //XM[1]   -5x10
							0x00024D58,0x00000000,    //XM[2]   5x10
							0x00004F4D,0x00000001,    //MO  1
							0x00004341,0x00000000,		//AC
							0x00004341,0x00000000,		//AC
							0x00004341,0x00000000,		//AC
							0x00004344,0x00000000,		//DC
							0x00004344,0x00000000,		//DC
							0x00004344,0x00000000,		//DC
							0x00005053,0x00000000,		//SP
							0x00005053,0x00000000,		//SP
							0x00005053,0x00000000,		//SP
							0x00034856,0x00000000,		//VH[3]  999,999,990
							0x00034C56,0x00000000,		//VL[3]  -99,999,990

						 };

	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	data[5][1]= acc;
	data[6][1]= acc;
	data[7][1]= acc;
	data[8][1]= dec;
	data[9][1]= dec;
	data[10][1]= dec;
	data[11][1]= vel;
	data[12][1]= vel;
	data[13][1]= vel;
	data[14][1]= 999999990;
	data[15][1]= -999999990;
	data[2][1]= -999999990;
	data[3][1]= 999999990;
	for(i=0;i<16;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;

		mbox= CAN_Transmit(CANx, &TxMessage);
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				UART5_OUT((uint8_t *)"CAN error in Pos_Cfg!!!!!!!!!\r\n");
			}
		}
	}
}

/*****************位置控制*********************/
/*******************1代表相对模式****************************/
/*******************0代表绝对模式****************************/
void PosCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t rel_abs,int pos)
{
	uint8_t i=0;
	uint32_t data[2][2]={
							0x00000000,0x00000000,      //PA  10000
							0x40004742,0x00000000,      //BG
						 };

	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	if(rel_abs==0){
		data[0][0]= 0x00004150;  //绝对
	}else{
		data[0][0]= 0x00005250;   //相对
	}

	data[0][1]= pos;
	for(i=0;i<2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;

		mbox= CAN_Transmit(CANx, &TxMessage);         //1.4us
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				UART5_OUT((uint8_t *)"CAN error in PosCrl!!!!!!!!!\r\n");
			}
		}
	}

}
/* 读取电机电压 */
void ReadActualVoltage(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005155,0x00000000,      //UQ
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadVoltage!!!!!!!!!\r\n");
		}
	}
 }

/* 读取电机电流 */
void ReadActualCurrent(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005149,0x00000000,      //IQ
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadCurrent!!!!!!!!!\r\n");
		}
	} }

/* 读取电机位置 */
void ReadActualPos(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005850,0x00000000,      //PX
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadPos!!!!!!!!!\r\n");
		}
	} }

/* 读取电机速度 */
void ReadActualVel(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40005856,0x00000000,      //VX
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadVel!!!!!!!!!\r\n");
		}
	}}

void ReadActualTemperature(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40014954,0x00000000,      //TI[1]
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadTemperature!!!!!!!!!\r\n");
		}
	}
}
void ReadCurrentLimitFlag(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000434C,0x00000000,      //LC
						 };
  uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	    //msg[4].data=*(unsigned long long*)&data[i];
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         //1.4us
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadCurrentLimitFlag!!!!!!!!!\r\n");
		}
	}
}

void ReadVelocityError(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004556,0x00000000,      //LC
						 };
  uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	    //msg[4].data=*(unsigned long long*)&data[i];
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         //1.4us
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadVelError!!!!!!!!!\r\n");
		}
	}
}

void ReadCommandVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40025644,0x00000000,      //DV[2]
						 };
  uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	    //msg[4].data=*(unsigned long long*)&data[i];
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         //1.4us
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadDV!!!!!!!!!\r\n");
		}
	}
}

void ReadJoggingVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000564A,0x00000000,      //JV
						 };
  uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	    //msg[4].data=*(unsigned long long*)&data[i];
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         //1.4us
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadJV!!!!!!!!!\r\n");
		}
	}
}

void ReadUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004D55,0x00000000,      //UM
						 };
  uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	    //msg[4].data=*(unsigned long long*)&data[i];
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         //1.4us
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadUnitMode!!!!!!!!!\r\n");
		}
	}
}

void ReadReferenceMode(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004D52,0x00000000,      //RM
						 };
  uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	    //msg[4].data=*(unsigned long long*)&data[i];
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         //1.4us
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadRM!!!!!!!!!\r\n");
		}
	}
}

void ReadMotorFailure(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000464D,0x00000000,      //MF
						 };
  uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x300 + ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=0x300 + ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	    //msg[4].data=*(unsigned long long*)&data[i];
	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         //1.4us
		uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			UART5_OUT((uint8_t *)"CAN error in ReadMotorFailure!!!!!!!!!\r\n");
		}
	}
}

/**
  * @brief  配置电机速度
  * @param  DriverNum: 电机号
  * @param  velData: 速度值。单位为：脉冲每秒。（每圈4096脉冲）
  * @retval None
  * @author Tmax Sco
**/
void velCrl(CAN_TypeDef* CANx, uint8_t DriverNum,int velData)
{
	CanTxMsg TxMessage;
	uint8_t mbox;
	union Vel
	{
		uint8_t Data8[4];
		int32_t Data32;
	}vel;

	vel.Data32 = velData;
	TxMessage.StdId= 0x300+DriverNum;					 // standard identifier=0
	TxMessage.ExtId= 0x300+DriverNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = vel.Data8[0];
	TxMessage.Data[1] = vel.Data8[1];
	TxMessage.Data[2] = vel.Data8[2];
	TxMessage.Data[3] = vel.Data8[3];

	mbox= CAN_Transmit(CANx, &TxMessage);
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok));
}

