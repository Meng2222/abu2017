#ifndef __GASVALVECONTROL_H
#define __GASVALVECONTROL_H


#include  "can.h"

void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState);
void ClampOpen(void);
void ClampClose(void);
void ClampRotate(void);
void ClampReset(void);
void LeftPush(void);
void LeftBack(void);
void RightPush(void);
void RightBack(void);
#endif
