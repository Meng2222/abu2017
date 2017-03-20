#ifndef __DATABASE_H
#define __DATABASE_H
#include "robot.h"

extern gun_pose_t gLeftGunPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER];
extern gun_pose_t gRightGunPosDatabase[SHOOT_METHOD_NUMBER][LAND_NUMBER];
extern gun_pose_t gUpperGunPosDatabase[SHOOT_METHOD_NUMBER][ZONE_NUMBER];
extern shoot_command_t gLeftGunShootCmds;
extern shoot_command_t gRightGunShootCmds;
extern shoot_command_t gUpperGunShootCmds;

#endif
