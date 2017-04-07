#ifndef __GASVALVECONTROL_H
#define __GASVALVECONTROL_H


#include  "can.h"
#define CLAMP_OPEN_IO_ID 1
#define CLAMP_CLOSE_IO_ID 1
#define CLAMP_ROTATE_IO_ID 4
#define LEFT_RELOAD_IO_ID 3
#define LEFT_RELOAD_RESET_IO_ID 3
#define RIGHT_RELOAD_IO_ID 2
#define RIGHT_RELOAD_RESET_IO_ID 2
#define LEFT_SHOOT_IO_ID 8
#define RIGHT_SHOOT_IO_ID 6
#define UPPER_SHOOT_IO_ID 5

#define CLAMP_OPEN_BOARD_ID 1
#define CLAMP_CLOSE_BOARD_ID 2
#define CLAMP_ROTATE_BOARD_ID 1
#define LEFT_RELOAD_BOARD_ID 1
#define LEFT_RELOAD_RESET_BOARD_ID 2
#define RIGHT_RELOAD_BOARD_ID 1
#define RIGHT_RELOAD_RESET_BOARD_ID 2
#define LEFT_SHOOT_BOARD_ID 2
#define RIGHT_SHOOT_BOARD_ID 1
#define UPPER_SHOOT_BOARD_ID 1

void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState);
void ClampOpen(void);
void ClampClose(void);
void ClampRotate(void);
void ClampReset(void);
void LeftPush(void);
void LeftBack(void);
void LeftHold(void);
void RightPush(void);
void RightBack(void);
void RightHold(void);
void LeftShoot(void);
void LeftShootReset(void);
void RightShoot(void);
void RightShootReset(void);
void UpperShoot(void);
void UpperShootReset(void);
#endif
