#ifndef __DATABASE_H
#define __DATABASE_H
#include "robot.h"

	
extern gun_pose_t gLeftGunPosDatabase[SHOOT_POINT_NUMBER][SHOOT_METHOD_NUMBER][LAND_NUMBER];
extern gun_pose_t gRightGunPosDatabase[SHOOT_POINT_NUMBER][SHOOT_METHOD_NUMBER][LAND_NUMBER];
extern gun_pose_t gUpperGunPosDatabase[SHOOT_METHOD_NUMBER][ZONE_NUMBER];
extern shoot_command_t gLeftGunShootCmds[MAX_AUTO_BULLET_NUMBER];
extern shoot_command_t gRightGunShootCmds[MAX_AUTO_BULLET_NUMBER];
//extern shoot_command_t gUpperGunShootCmds;

#define LEFTGUNPOSDATABASE_FLOAT_NUM	(sizeof(gLeftGunPosDatabase)/sizeof(float))
#define RIGHTGUNPOSDATABASE_FLOAT_NUM	(sizeof(gRightGunPosDatabase)/sizeof(float))
#define UPPERGUNPOSDATABASE_FLOAT_NUM	(sizeof(gUpperGunPosDatabase)/sizeof(float))

#endif
