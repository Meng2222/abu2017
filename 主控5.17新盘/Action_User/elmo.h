#ifndef __ELMO_H
#define __ELMO_H
#include "stm32f4xx.h"
#define   POS_REL  1    //相对位置方式POS_REL = 1 绝对位置方式POS_ABS = 0
#define   POS_ABS  0 
#define SDO_RESPONSE_COB_ID_BASE 0x280

union can_message
{
	uint8_t data8[4];
	int data32_t;
	float dataf;
};

void elmo_Init(CAN_TypeDef* CANx);
void elmo_Enable(CAN_TypeDef* CANx, uint8_t ElmoNum);
void elmo_Disable(CAN_TypeDef* CANx, uint8_t ElmoNum);

void Vel_cfg(CAN_TypeDef* CANx, uint8_t ElmoNum,uint32_t acc,uint32_t dec) ;
void Pos_cfg(CAN_TypeDef* CANx, uint8_t ElmoNum,uint32_t acc,uint32_t dec,uint32_t vel) ;
void VelCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,int vel);
void PosCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t rel_abs,int pos);

void ReadActualPos(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadActualVoltage(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadActualCurrent(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadActualVel(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadActualTemperature(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadCurrentLimitFlag(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadVelocityError(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadCommandVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadJoggingVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadReferenceMode(CAN_TypeDef* CANx, uint8_t ElmoNum);
void ReadMotorFailure(CAN_TypeDef* CANx, uint8_t ElmoNum);


void velCrl(CAN_TypeDef* CANx, uint8_t DriverNum,int vel);

#endif 


