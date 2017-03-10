#include "robot.h"
#include "elmo.h"
#include "database.h"
#include "gasvalvecontrol.h"
#include "timer.h"
#include "ucos_ii.h"

robot_t gRobot = {0};


static void LeftGunInit(void)
{
	gRobot.leftGun.actualPose.pitch = 0.0f;
	gRobot.leftGun.actualPose.yaw = 0.0f;
	gRobot.leftGun.actualPose.roll = 0.0f;

	gRobot.leftGun.targetPose.pitch = 0.0f;
	gRobot.leftGun.targetPose.yaw = 0.0f;
	gRobot.leftGun.targetPose.roll = 0.0f;
	
	gRobot.leftGun.maxPoseLimit.pitch = 40.0f;
	gRobot.leftGun.maxPoseLimit.yaw = 50.0f;
	gRobot.leftGun.maxPoseLimit.roll = 45.0f;
	gRobot.leftGun.maxPoseLimit.speed1=200.0f;
	gRobot.leftGun.maxPoseLimit.speed2=200.0f;

	
	gRobot.leftGun.minPoseLimit.pitch = 15.0f;
	gRobot.leftGun.minPoseLimit.yaw = -50.0f;
	gRobot.leftGun.minPoseLimit.roll = 0.0f;
	gRobot.leftGun.minPoseLimit.speed1=0.0f;
	gRobot.leftGun.minPoseLimit.speed2=0.0f;	
	
	//fix me, should be defined as macro
	gRobot.leftGun.bulletNumber = MAX_BULLET_NUMBER;
	//�������Ϊ0
	gRobot.leftGun.shootTimes = 0;
	//ǹֹͣ���
	gRobot.leftGun.shoot = GUN_STOP_SHOOT;
	//ǹδ������׼
	gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;
	//�Զ�ģʽ
	gRobot.leftGun.mode = GUN_AUTO_MODE;
	gRobot.leftGun.gunPoseDatabase = (gun_pose_t **)gLeftGunPosDatabase;
	//��ǹ�Զ���������ϣ�����ΪͶ�����ӵ�˳��
	gRobot.leftGun.shootCommand = (shoot_command_t *)&gLeftGunShootCmds;
	//ǹ���ӵ�״̬�����ӵ�
	gRobot.leftGun.champerBulletState = CHAMPER_BULLET_EMPTY_STATE;
	//fix me
	gRobot.leftGun.champerErrerState = 0;
	
	elmo_Enable(LEFT_GUN_LEFT_ID);
	elmo_Enable(LEFT_GUN_RIGHT_ID);
	elmo_Enable(LEFT_GUN_PITCH_ID);
	elmo_Enable(LEFT_GUN_ROLL_ID);
	elmo_Enable(LEFT_GUN_YAW_ID);


	Vel_cfg(LEFT_GUN_LEFT_ID, 300000,300000);	
	Vel_cfg(LEFT_GUN_RIGHT_ID, 300000,300000);	

	Pos_cfg(LEFT_GUN_PITCH_ID, 5000,5000,30000);//����
	Pos_cfg(LEFT_GUN_ROLL_ID, 5000,5000,30000);//����
	Pos_cfg(LEFT_GUN_YAW_ID,5000,5000,30000);//����
	

}

static void RightGunInit(void)
{
	gRobot.rightGun.actualPose.pitch = 0.0f;
	gRobot.rightGun.actualPose.yaw = 0.0f;
	gRobot.rightGun.actualPose.roll = 0.0f;

	gRobot.rightGun.targetPose.pitch = 0.0f;
	gRobot.rightGun.targetPose.yaw = 0.0f;
	gRobot.rightGun.targetPose.roll = 0.0f;
	
	//fix me, current no data 
	gRobot.rightGun.maxPoseLimit.pitch = 0.0f;
	gRobot.rightGun.maxPoseLimit.yaw = 0.0f;
	gRobot.rightGun.maxPoseLimit.roll = 0.0f;
	gRobot.rightGun.maxPoseLimit.speed1=0.0f;
	gRobot.rightGun.maxPoseLimit.speed2=0.0f;
	
	gRobot.rightGun.minPoseLimit.pitch = 0.0f;
	gRobot.rightGun.minPoseLimit.yaw = 0.0f;
	gRobot.rightGun.minPoseLimit.roll = 0.0f;
	gRobot.rightGun.minPoseLimit.speed1=0.0f;
	gRobot.rightGun.minPoseLimit.speed2=0.0f;
	
	//fix me, should be defined as macro
	gRobot.rightGun.bulletNumber = MAX_BULLET_NUMBER;
	//�������Ϊ0
	gRobot.rightGun.shootTimes = 0;
	//ǹֹͣ���
	gRobot.rightGun.shoot = GUN_STOP_SHOOT;
	//ǹδ������׼
	gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
	//�Զ�ģʽ
	gRobot.rightGun.mode = GUN_AUTO_MODE;
	gRobot.rightGun.gunPoseDatabase = (gun_pose_t **)gRightGunPosDatabase;
	//��ǹ�Զ���������ϣ�����ΪͶ�����ӵ�˳��
	gRobot.rightGun.shootCommand = (shoot_command_t *)&gRightGunShootCmds;
	//Ŀ����½̨����Ϊ0,
	gRobot.rightGun.targetPlant = INVALID_PLANT_NUMBER;
	//ǹ���ӵ�״̬�����ӵ�
	gRobot.rightGun.champerBulletState = CHAMPER_BULLET_EMPTY_STATE;
	//fix me
	gRobot.rightGun.champerErrerState = 0;
	
	elmo_Enable(RIGHT_GUN_LEFT_ID);
	elmo_Enable(RIGHT_GUN_RIGHT_ID);
	elmo_Enable(RIGHT_GUN_PITCH_ID);
	elmo_Enable(RIGHT_GUN_ROLL_ID);
	elmo_Enable(RIGHT_GUN_YAW_ID);


	Vel_cfg(RIGHT_GUN_LEFT_ID, 300000,300000);	
	Vel_cfg(RIGHT_GUN_RIGHT_ID, 300000,300000);	

	Pos_cfg(RIGHT_GUN_PITCH_ID, 5000,5000,30000);//����
	Pos_cfg(RIGHT_GUN_ROLL_ID, 5000,5000,30000);//����
	Pos_cfg(RIGHT_GUN_YAW_ID,5000,5000,30000);//����
}

static void UpperGunInit(void)
{
	gRobot.upperGun.actualPose.pitch = 0.0f;
	gRobot.upperGun.actualPose.yaw = 0.0f;
	gRobot.upperGun.actualPose.roll = 0.0f;

	gRobot.upperGun.targetPose.pitch = 0.0f;
	gRobot.upperGun.targetPose.yaw = 0.0f;
	gRobot.upperGun.targetPose.roll = 0.0f;
	
	gRobot.upperGun.maxPoseLimit.pitch = 40.0f;
	gRobot.upperGun.maxPoseLimit.yaw = 20.0f;
	gRobot.upperGun.maxPoseLimit.roll = 0.0f;
	gRobot.upperGun.maxPoseLimit.speed1=200.0f;
	gRobot.upperGun.maxPoseLimit.speed2=0.0f;
	
	gRobot.upperGun.minPoseLimit.pitch = -10.0f;
	gRobot.upperGun.minPoseLimit.yaw = -20.0f;
	gRobot.upperGun.minPoseLimit.roll = 0.0f;
	gRobot.upperGun.minPoseLimit.speed1=0.0f;
	gRobot.upperGun.minPoseLimit.speed2=0.0f;
	
	//fix me, should be defined as macro
	gRobot.upperGun.bulletNumber = MAX_BULLET_NUMBER;
	//�������Ϊ0
	gRobot.upperGun.shootTimes = 0;
	//ǹֹͣ���
	gRobot.upperGun.shoot = GUN_STOP_SHOOT;
	//ǹδ������׼
	gRobot.upperGun.ready = GUN_AIM_IN_PROCESS;
	//�Զ�ģʽ
	gRobot.upperGun.mode = GUN_AUTO_MODE;
	gRobot.upperGun.gunPoseDatabase = (gun_pose_t **)gUpperGunPosDatabase;
	//����ǹ�Զ���������ϣ�����ΪͶ�����ӵ�˳��
	gRobot.upperGun.shootCommand = (shoot_command_t *)&gUpperGunShootCmds;
	//Ŀ����½̨����Ϊ7
	gRobot.upperGun.targetPlant = 7;
	//ǹ���ӵ�״̬�����ӵ�
	gRobot.upperGun.champerBulletState = CHAMPER_BULLET_EMPTY_STATE;
	//fix me
	gRobot.upperGun.champerErrerState = 0;
	
	elmo_Enable(UPPER_GUN_LEFT_ID);
	elmo_Enable(UPPER_GUN_YAW_ID);
	elmo_Enable(UPPER_GUN_PITCH_ID);
	
	Vel_cfg(UPPER_GUN_LEFT_ID,300000,300000);
	Pos_cfg(UPPER_GUN_YAW_ID,5000,5000,30000);//����
	Pos_cfg(UPPER_GUN_PITCH_ID,5000,5000,30000);//����
}

/*
*���ƣ�LeftGunYawTransform
*���ܣ���ǹyaw��Ƕ�ת����λ�ã���������͸���λ�û�
*������
*
*ע�⣺
*/
int32_t LeftGunYawTransform(float yaw)
{
	if(yaw > gRobot.leftGun.maxPoseLimit.yaw) yaw = gRobot.leftGun.maxPoseLimit.yaw;	
	if(yaw < gRobot.leftGun.minPoseLimit.yaw) yaw = gRobot.leftGun.minPoseLimit.yaw;
	return (int32_t)((50.0f + yaw) * 102.4f);
}

/*
*���ƣ�LeftGunYawInverseTransform
*���ܣ���ǹyaw��λ��ת�����Ƕ�
*������
*position:��ľ���λ��pulse
*ע�⣺
*/
float LeftGunYawInverseTransform(int position)
{
	return (float)position/102.4f - 50.0f;
}

/*
*���ƣ�LeftGunPitchTransform
*���ܣ���ǹpitch��Ƕ�ת����λ�ã���������͸���λ�û�
*������
*
*ע�⣺
*/
int32_t LeftGunPitchTransform(float pitch)
{
	if(pitch > gRobot.leftGun.maxPoseLimit.pitch) pitch = gRobot.leftGun.maxPoseLimit.pitch;	
	if(pitch < gRobot.leftGun.minPoseLimit.pitch) pitch = gRobot.leftGun.minPoseLimit.pitch;
	return (int32_t)((pitch - 15.0f) * 141.0844f);
}

/*
*���ƣ�LeftGunPitchInverseTransform
*���ܣ���ǹpitch��λ��ת�����Ƕ�
*������
*position:��ľ���λ��pulse
*ע�⣺
*/
float LeftGunPitchInverseTransform(int position)
{
	return (float)position/141.0844f + 15.0f;
}

/*
*���ƣ�LeftGunRollTransform
*���ܣ���ǹroll��Ƕ�ת����λ�ã���������͸���λ�û�
*������
*
*ע�⣺
*/
int32_t LeftGunRollTransform(float roll)
{
	if(roll > gRobot.leftGun.maxPoseLimit.roll) roll = gRobot.leftGun.maxPoseLimit.roll;	
	if(roll < gRobot.leftGun.minPoseLimit.roll) roll = gRobot.leftGun.minPoseLimit.roll;
	return (int32_t)(roll * 141.0844f);
}

/*
*���ƣ�LeftGunRollInverseTransform
*���ܣ���ǹroll��λ��ת�����Ƕ�
*������
*position:��ľ���λ��pulse
*ע�⣺
*/
float LeftGunRollInverseTransform(int position)
{
	return (float)position/141.0844f;
}

/*
*���ƣ�LeftGunLeftSpeedTransform
*���ܣ���ǹ���ʹ��ٶ�ת����m/s ��pulse/s
*������
*
*ע�⣺
*/
int32_t LeftGunLeftSpeedTransform(float speed)
{
	
	if(speed > gRobot.leftGun.maxPoseLimit.speed1) speed = gRobot.leftGun.maxPoseLimit.speed1;	
	if(speed < gRobot.leftGun.minPoseLimit.speed1) speed = gRobot.leftGun.minPoseLimit.speed1;
	return -4096*speed;
}

/*
*���ƣ�LeftGunLeftSpeedInverseTransform
*���ܣ���ǹ���ʹ��ٶ���任��pulse/s��m/s 
*������
*
*ע�⣺
*/
float LeftGunLeftSpeedInverseTransform(int speed)
{
	//fix me, ��Ӳ����Ϸ��Լ��
	return -speed/4096;
}

/*
*���ƣ�LeftGunRightSpeedTransform
*���ܣ���ǹ�Ҵ��ʹ��ٶ�ת����m/s ��pulse/s
*������
*
*ע�⣺
*/
int32_t LeftGunRightSpeedTransform(float speed)
{
	if(speed > gRobot.leftGun.maxPoseLimit.speed2) speed = gRobot.leftGun.maxPoseLimit.speed2;	
	if(speed < gRobot.leftGun.minPoseLimit.speed2) speed = gRobot.leftGun.minPoseLimit.speed2;
	return 4096*speed;
}

/*
*���ƣ�LeftGunRightSpeedInverseTransform
*���ܣ���ǹ�Ҵ��ʹ��ٶ���任��pulse/s��m/s 
*������
*
*ע�⣺
*/
float LeftGunRightSpeedInverseTransform(int speed)
{
	//fix me, ��Ӳ����Ϸ��Լ��
	return speed/4096;
}

/*
*���ƣ�RightGunYawTransform
*���ܣ���ǹyaw��Ƕ�ת����λ�ã���������͸���λ�û�
*������
*
*ע�⣺
*/
int32_t RightGunYawTransform(float yaw)
{
	//fix me ,tansform may be different from LeftGun 
	if(yaw > gRobot.rightGun.maxPoseLimit.yaw) yaw = gRobot.rightGun.maxPoseLimit.yaw;	
	if(yaw < gRobot.rightGun.minPoseLimit.yaw) yaw = gRobot.rightGun.minPoseLimit.yaw;
	return (int32_t)((50.0f + yaw) * 102.4f);
}

/*
*���ƣ�RightGunYawInverseTransform
*���ܣ���ǹyaw��Ƕȷ��任��������ת��Ϊ�Ƕ�
*������
*
*ע�⣺
*/
float RightGunYawInverseTransform(float position)
{
	//fix me ,tansform may be different from LeftGun 
	return (int32_t)(position/102.4f-50.0f);
}

/*
*���ƣ�RightGunPitchTransform
*���ܣ���ǹpitch��Ƕ�ת����λ�ã���������͸���λ�û�
*������
*
*ע�⣺
*/
int32_t RightGunPitchTransform(float pitch)
{
	//fix me ,tansform may be different from LeftGun 
	if(pitch > gRobot.rightGun.maxPoseLimit.pitch) pitch = gRobot.rightGun.maxPoseLimit.pitch;	
	if(pitch < gRobot.rightGun.minPoseLimit.pitch) pitch = gRobot.rightGun.minPoseLimit.pitch;
	return (int32_t)((pitch - 15.0f) * 141.0844f);	
}

/*
*���ƣ�RightGunPitchInverseTransform
*���ܣ���ǹpitch��Ƕȷ��任��������ת��Ϊ�Ƕ�
*������
*
*ע�⣺
*/
float RightGunPitchInverseTransform(float position)
{
	//fix me ,tansform may be different from LeftGun 
	return (int32_t)(position/141.0844f+15.0f);
}

/*
*���ƣ�RightGunRollTransform
*���ܣ���ǹroll��Ƕ�ת����λ�ã���������͸���λ�û�
*������
*
*ע�⣺
*/
int32_t RightGunRollTransform(float roll)
{
	//fix me ,tansform may be different from LeftGun 
	if(roll > gRobot.rightGun.maxPoseLimit.roll) roll = gRobot.rightGun.maxPoseLimit.roll;	
	if(roll < gRobot.rightGun.minPoseLimit.roll) roll = gRobot.rightGun.minPoseLimit.roll;
	return (int32_t)(roll * 141.0844f);
}

/*
*���ƣ�RightGunRollInverseTransform
*���ܣ���ǹroll��Ƕȷ��任��������ת��Ϊ�Ƕ�
*������
*
*ע�⣺
*/
float RightGunRollInverseTransform(float position)
{
	//fix me ,tansform may be different from LeftGun 
	return (int32_t)(position/141.0844f);
}

/*
*���ƣ�RightGunLeftSpeedTransform
*���ܣ���ǹ���ʹ��ٶ�ת������תÿ��ת��Ϊ����/s
*������
*
*ע�⣺
*/
int32_t RightGunLeftSpeedTransform(float speed)
{
	//fix me ,tansform may be different from LeftGun 
	if(speed > gRobot.rightGun.maxPoseLimit.speed1) speed = gRobot.rightGun.maxPoseLimit.speed1;	
	if(speed < gRobot.rightGun.minPoseLimit.speed1) speed = gRobot.rightGun.minPoseLimit.speed1;
	return -4096*speed;
}

/*
*���ƣ�RightGunLeftSpeedInverseTransform
*���ܣ���ǹ���ʹ��ٶȷ��任��������ת��Ϊתÿ��
*������
*
*ע�⣺
*/
float RightGunLeftSpeedInverseTransform(float speed)
{
	//fix me ,tansform may be different from LeftGun 
	return (int32_t)(speed/-4096.0f);
}

/*
*���ƣ�RightGunRightSpeedTransform
*���ܣ���ǹ�Ҵ��ʹ��ٶ�ת���� ��תÿ��ת��Ϊ����/s
*
*ע�⣺
*/
int32_t RightGunRightSpeedTransform(float speed)
{
	//fix me ,tansform may be different from LeftGun 
	if(speed > gRobot.rightGun.maxPoseLimit.speed2) speed = gRobot.rightGun.maxPoseLimit.speed2;	
	if(speed < gRobot.rightGun.minPoseLimit.speed2) speed = gRobot.rightGun.minPoseLimit.speed2;
	return -4096*speed;
}

/*
*���ƣ�RightGunRightSpeedInverseTransform
*���ܣ���ǹ���ʹ��ٶȷ��任��������ת��Ϊתÿ��
*������
*
*ע�⣺
*/
float RightGunRightSpeedInverseTransform(float speed)
{
	//fix me ,tansform may be different from LeftGun 
	return (int32_t)(speed/-4096.0f);
}
/*
*���ƣ�UpperGunYawTransform
*���ܣ�����ǹyaw��Ƕ�ת����λ�ã���������͸���λ�û�
*������
*
*ע�⣺
*/
int32_t UpperGunYawTransform(float yaw)
{
	//fix me ,tansform may be different from LeftGun 
	if(yaw > gRobot.upperGun.maxPoseLimit.yaw) yaw = gRobot.upperGun.maxPoseLimit.yaw;	
	if(yaw < gRobot.upperGun.minPoseLimit.yaw) yaw = gRobot.upperGun.minPoseLimit.yaw;
	return (int32_t)((20.0f + yaw) * 102.4f);
}

/*
*���ƣ�UpperGunPitchTransform
*���ܣ�����ǹpitch��Ƕ�ת����λ�ã���������͸���λ�û�
*������
*
*ע�⣺
*/
int32_t UpperGunPitchTransform(float pitch)
{
	//fix me ,tansform may be different from LeftGun 
	if(pitch > gRobot.upperGun.maxPoseLimit.pitch) pitch = gRobot.upperGun.maxPoseLimit.pitch;	
	if(pitch < gRobot.upperGun.minPoseLimit.pitch) pitch = gRobot.upperGun.minPoseLimit.pitch;
	return (int32_t)((10.0f+pitch) * 141.0844f);	
}

/*
*���ƣ�UpperGunLeftSpeedTransform
*���ܣ���ǹ���ʹ��ٶ�ת������
*������
*
*ע�⣺
*/
int32_t UpperGunLeftSpeedTransform(float speed)
{
	//fix me ,tansform may be different from LeftGun 
	if(speed > gRobot.upperGun.maxPoseLimit.speed1) speed = gRobot.upperGun.maxPoseLimit.speed1;	
	if(speed < gRobot.upperGun.minPoseLimit.speed1) speed = gRobot.upperGun.minPoseLimit.speed1;
	return -4096*speed;
}

/*
*���ƣ�ROBOT_Init
*���ܣ������˳�ʼ������ʼ�����̣���ʼ��ǹ����ʼ��
*������none
*ע�⣺�����ǹ����Ҫ���ӵ�����Ϊ���ֶ��ϵ�
*/
status_t ROBOT_Init(void)
{
	gRobot.stage = ROBOT_STAGE_POWER_ON;
	gRobot.shootTimes = 0;
	gRobot.status = ROBOT_STATUS_OK;
	
	LeftGunInit();
	RightGunInit();
	UpperGunInit();

	MOVEBASE_Init();
	
	gRobot.stage = ROBOT_STAGE_INIT;

	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_GunLoad
*���ܣ���װ���У���ץȡ�ӵ�����
*������
*status:GUN_NO_ERROR��GUN_RELOAD_ERROR
*ע�⣺�����ǹ����Ҫ���ӵ�����Ϊ���ֶ��ϵ�
*/
status_t ROBOT_GunLoad(void)
{
	ClampClose();

	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_GunOpenSafety
*���ܣ�����ǹ���գ��ӵ���װ�ú���ܽ��д˲���
*������
*status:GUN_NO_ERROR��GUN_OPEN_SAFETY_ERROR
*ע�⣺�����ǹ����Ҫ
*/
status_t ROBOT_GunOpenSafety(void)
{
	ClampRotate();
	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_GunReload
*���ܣ���ǹ�ϵ���ÿ�����ǰ��Ҫ�ϵ�һ��
*������
*gun :LEFT_GUN, RIGHT_GUN
*status:GUN_NO_ERROR��GUN_RELOAD_ERROR
*ע�⣺�����ǹ����Ҫ���ӵ�
*/
status_t ROBOT_GunReload(unsigned char gun)
{
	switch(gun)
	{
		case LEFT_GUN:
			LeftPush();
			OSTimeDly(100);
			LeftBack();
			break;
		case RIGHT_GUN:
			RightPush();
			OSTimeDly(100);
			RightBack();
			break;
		case UPPER_GUN:
			break;
		default:
			break;
	}
	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_GunReload
*���ܣ�����ǹ�Ŵ�����������ӵ�״̬���������濪ǹ�ľ������
*������
*gun :LEFT_GUN, RIGHT_GUN
*status:GUN_NO_ERROR��GUN_RELOAD_ERROR
*ע�⣺�����ǹ����Ҫ���ӵ�����Ϊ���ֶ��ϵ�
*/
status_t ROBOT_GunCheckBulletState(unsigned char gun)
{
	switch(gun)
	{
		case LEFT_GUN:
			//fix me, should be replaced by senser results
			gRobot.leftGun.champerBulletState = CHAMPER_BULLET_FEATURE0_STATE;
			break;
		case RIGHT_GUN:
			gRobot.rightGun.champerBulletState = CHAMPER_BULLET_FEATURE0_STATE;
			break;
		case UPPER_GUN:
			gRobot.upperGun.champerBulletState = CHAMPER_BULLET_FEATURE0_STATE;
			break;
		default:
			break;
	}
	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_GunAim
*���ܣ���׼��Ŀ��ı����Ҫ�ȵ��ô˽ӿ���������׼
*������
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*status:GUN_NO_ERROR
*ע�⣺�����ǹĿǰ��е��û��roll��û���Ҳഫ�ʹ�speed2
*/
status_t ROBOT_GunAim(unsigned char gun)
{
	//����Ӧ�ñ�֤ǹ�������ӵ�������,fix me���������Ϸ���
	
	switch(gun)
	{
		case LEFT_GUN:
			gRobot.leftGun.ready = GUN_AIM_IN_PROCESS;
			if(gRobot.leftGun.champerBulletState == CHAMPER_BULLET_EMPTY_STATE) return GUN_NO_BULLET_ERROR;

			PosCrl(LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(gRobot.leftGun.targetPose.yaw));
			PosCrl(LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(gRobot.leftGun.targetPose.pitch));			
			PosCrl(LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(gRobot.leftGun.targetPose.roll));	

			VelCrl(LEFT_GUN_LEFT_ID, LeftGunLeftSpeedTransform(gRobot.leftGun.targetPose.speed1));
			VelCrl(LEFT_GUN_RIGHT_ID,  LeftGunRightSpeedTransform(gRobot.leftGun.targetPose.speed2));

			break;
		case RIGHT_GUN:
			gRobot.rightGun.ready = GUN_AIM_IN_PROCESS;
			if(gRobot.rightGun.champerBulletState == CHAMPER_BULLET_EMPTY_STATE) return GUN_NO_BULLET_ERROR;
			PosCrl(RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(gRobot.rightGun.targetPose.yaw));
			PosCrl(RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(gRobot.rightGun.targetPose.pitch));			
			PosCrl(RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(gRobot.rightGun.targetPose.roll));	

			VelCrl(RIGHT_GUN_LEFT_ID, RightGunLeftSpeedTransform(gRobot.rightGun.targetPose.speed1));
			VelCrl(RIGHT_GUN_RIGHT_ID,  RightGunRightSpeedTransform(gRobot.rightGun.targetPose.speed2));
			break;
		case UPPER_GUN:
			if(gRobot.upperGun.champerBulletState == CHAMPER_BULLET_EMPTY_STATE) return GUN_NO_BULLET_ERROR;
			PosCrl(UPPER_GUN_YAW_ID, POS_ABS, UpperGunYawTransform(gRobot.upperGun.targetPose.yaw));
			PosCrl(UPPER_GUN_PITCH_ID, POS_ABS, UpperGunPitchTransform(gRobot.upperGun.targetPose.pitch));			

			VelCrl(UPPER_GUN_LEFT_ID, UpperGunLeftSpeedTransform(gRobot.upperGun.targetPose.speed1));
			break;
		default:
			break;
	}
	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_LeftGunCheckAim
*���ܣ������׼�Ƿ�����ɣ���ͬǹ�ֿ����Ϊ�˷�ֹ���룬
*��Ϊ�˺�������Ҫ��Ƴ�ʱ
*������
*none
*status:GUN_AIM_IN_PROCESS�� GUN_AIM_DONE
*ע�⣺
*/
status_t ROBOT_LeftGunCheckAim(void)
{
	//��ʱʱ��Ϊ100*5*10ms��1��
	int timeout = 100;

	while(timeout--)
	{
		//fix me,����5��������Ҫ200us*5�����Ϸ��ص�5֡���ݣ���ﵽ2ms���������ʹ����IDʵ�֣���Ҫ������֧��
		//fix me ����λ���Ѿ�֧����ID����ID��robot.h�ж���
		ReadActualPos(LEFT_GUN_GROUP_ID);		
		ReadActualVel(LEFT_GUN_LEFT_ID);
		ReadActualVel(LEFT_GUN_RIGHT_ID);
		OSTimeDly(5);
		//fix me,���ǹλ���Ƿ�λ��������Ҫ��ǹ�ṹ�������ӿ�������Ȼ���װ�ɺ������
		if(gRobot.leftGun.actualPose.pitch > gRobot.leftGun.targetPose.pitch + 0.5f || \
			gRobot.leftGun.actualPose.pitch < gRobot.leftGun.targetPose.pitch - 0.5f)
		{
			continue;
		}
		
		if(gRobot.leftGun.actualPose.roll > gRobot.leftGun.targetPose.roll + 0.5f || \
			gRobot.leftGun.actualPose.roll < gRobot.leftGun.targetPose.roll - 0.5f)
		{
			continue;
		}
		
		if(gRobot.leftGun.actualPose.yaw > gRobot.leftGun.targetPose.yaw + 0.5f || \
			gRobot.leftGun.actualPose.yaw < gRobot.leftGun.targetPose.yaw - 0.5f)
		{
			continue;
		}
		
		//�����鴫�ʹ����ٶȣ���ʱû�м�
		
		
		//���е������ʾ������ָ�꣬����ѭ��
		break;
	}
	gRobot.leftGun.ready = GUN_AIM_DONE;
	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_RightGunCheckAim
*���ܣ������׼�Ƿ�����ɣ���ͬǹ�ֿ����Ϊ�˷�ֹ���룬
*��Ϊ�˺�������Ҫ��Ƴ�ʱ
*������
*none
*status:GUN_AIM_IN_PROCESS�� GUN_AIM_DONE
*ע�⣺
*/
status_t ROBOT_RightGunCheckAim(void)
{
	//��ʱʱ��Ϊ100*5*10ms��1��
	int timeout = 100;

	while(timeout--)
	{
		//fix me,����3��������Ҫ200us*3�����Ϸ��ص�5֡���ݣ���ﵽ2ms���������ʹ����IDʵ�֣���Ҫ������֧��
		//fix me ����λ���Ѿ�֧����ID����ID��robot.h�ж���
		ReadActualPos(RIGHT_GUN_GROUP_ID);		
		ReadActualVel(RIGHT_GUN_LEFT_ID);
		ReadActualVel(RIGHT_GUN_RIGHT_ID);
		OSTimeDly(5);
		//fix me,���ǹλ���Ƿ�λ��������Ҫ��ǹ�ṹ�������ӿ�������Ȼ���װ�ɺ������
		if(gRobot.rightGun.actualPose.pitch > gRobot.rightGun.targetPose.pitch + 0.5f || \
			gRobot.rightGun.actualPose.pitch < gRobot.rightGun.targetPose.pitch - 0.5f)
		{
			continue;
		}
		
		if(gRobot.rightGun.actualPose.roll > gRobot.rightGun.targetPose.roll + 0.5f || \
			gRobot.rightGun.actualPose.roll < gRobot.rightGun.targetPose.roll - 0.5f)
		{
			continue;
		}
		
		if(gRobot.rightGun.actualPose.yaw > gRobot.rightGun.targetPose.yaw + 0.5f || \
			gRobot.rightGun.actualPose.yaw < gRobot.rightGun.targetPose.yaw - 0.5f)
		{
			continue;
		}
		
		//�����鴫�ʹ����ٶȣ���ʱû�м�
		
		
		//���е������ʾ������ָ�꣬����ѭ��
		break;
	}
	gRobot.rightGun.ready = GUN_AIM_DONE;
	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_GunShoot
*���ܣ���ǹ����ǹǰ��Ҫȷ���ӵ����ţ��������գ�ǹ֧���Ѿ�����
*������
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*mode: auto or manual
*status:GUN_NO_ERROR��GUN_CHAMPER_ERROR�� GUN_NO_BULLET_ERROR�� GUN_NO_READY_ERROR
*/
status_t ROBOT_GunShoot(unsigned char gun, unsigned char mode)
{
	switch(gun)
	{
		case LEFT_GUN:
			if(gRobot.leftGun.ready == GUN_AIM_DONE)
			{
				GasValveControl(1,5,1);	
				OSTimeDly(100);
				GasValveControl(1,5,0);	
				gRobot.leftGun.shootTimes++;
				//fix me, Ӧ�ü���ӵ��Ƿ�����
				gRobot.leftGun.bulletNumber--;
			}
			break;
		case RIGHT_GUN:
			if(gRobot.rightGun.ready == GUN_AIM_DONE)
			{
				//fix me there should be a GasValveControl
				OSTimeDly(100);
				//fix me there should be a GasValveControl
				gRobot.rightGun.shootTimes++;
				//fix me, Ӧ�ü���ӵ��Ƿ�����
				gRobot.rightGun.bulletNumber--;
			}
			break;
		case UPPER_GUN:
			GasValveControl(2,8,1);
			OSTimeDly(100);
			GasValveControl(2,8,0);
			gRobot.upperGun.shootTimes++;
			break;
		default:
			break;
	}

	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_GunHome
*���ܣ�ǹ��λ����ǹ��Ϊ�˸��õ�������Ҫ��λ
*������
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*status:GUN_NO_ERROR
*/
status_t ROBOT_GunHome(unsigned char gun)
{
	switch(gun)
	{
		case LEFT_GUN:
		PosCrl(LEFT_GUN_YAW_ID, POS_ABS, LeftGunYawTransform(0.0f));
		PosCrl(LEFT_GUN_PITCH_ID, POS_ABS, LeftGunPitchTransform(40.0f));			
		PosCrl(LEFT_GUN_ROLL_ID, POS_ABS, LeftGunRollTransform(0.0f));	

		//��ʱʱ��Ϊ100*5*10ms��1��
//	int timeout = 100;

//	while(timeout--)
//	{
//		//fix me,����5��������Ҫ200us*5�����Ϸ��ص�5֡���ݣ���ﵽ2ms���������ʹ����IDʵ�֣���Ҫ������֧��
//		ReadActualPos(LEFT_GUN_ROLL_ID);
//		ReadActualPos(LEFT_GUN_PITCH_ID);
//		ReadActualPos(LEFT_GUN_YAW_ID);
//		ReadActualVel(LEFT_GUN_LEFT_ID);
//		ReadActualVel(LEFT_GUN_RIGHT_ID);
//		OSTimeDly(5);
//		//fix me,���ǹλ���Ƿ�λ��������Ҫ��ǹ�ṹ�������ӿ�������Ȼ���װ�ɺ������
//		if(gRobot.leftGun.actualPose.pitch > gRobot.leftGun.targetPose.pitch + 0.5 || \
//			gRobot.leftGun.actualPose.pitch < gRobot.leftGun.targetPose.pitch - 0.5)
//		{
//			continue;
//		}
//		
//		if(gRobot.leftGun.actualPose.roll > gRobot.leftGun.targetPose.roll + 0.5 || \
//			gRobot.leftGun.actualPose.roll < gRobot.leftGun.targetPose.roll - 0.5)
//		{
//			continue;
//		}
//		
//		if(gRobot.leftGun.actualPose.yaw > gRobot.leftGun.targetPose.yaw + 0.5 || \
//			gRobot.leftGun.actualPose.yaw < gRobot.leftGun.targetPose.yaw - 0.5)
//		{
//			continue;
//		}
		
//		//�����鴫�ʹ����ٶȣ���ʱû�м�
//		
//		
//		//���е������ʾ������ָ�꣬����ѭ��
//		break;
//	}
		OSTimeDly(200);
			break;
		case RIGHT_GUN:
			PosCrl(RIGHT_GUN_YAW_ID, POS_ABS, RightGunYawTransform(0.0f));
			PosCrl(RIGHT_GUN_PITCH_ID, POS_ABS, RightGunPitchTransform(40.0f));			
			PosCrl(RIGHT_GUN_ROLL_ID, POS_ABS, RightGunRollTransform(0.0f));	
			OSTimeDly(200);
			break;
		case UPPER_GUN:
			break;
		default:
			break;
	}
	return GUN_NO_ERROR;
}

/*
*���ƣ�ROBOT_GunCheckMode
*���ܣ����ǹ��ģʽ
*������
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*status:
*/
status_t ROBOT_GunCheckMode(unsigned char gun)
{
	switch(gun)
	{
		case LEFT_GUN:
			return gRobot.leftGun.mode;
			break;
		case RIGHT_GUN:
			return gRobot.rightGun.mode;
			break;
		case UPPER_GUN:
			return gRobot.upperGun.mode;
			break;
		default:
			break;
	}
	return -1;
}






shootCtr_t shootParam[15]={{45.0f,16.0f,-45.2f,89.0f,21.0f,1},
						   {42.0f,33.6f,-47.5f,84.0f,21.0f,1},
						   {33.5f,17.7f,-23.8f,104.0f,19.0f,1},
						   {33.5f,32.5f,-22.1f,84.0f,20.0f,1},
						   {32.8f,32.5f,-22.6f,81.0f,24.0f,1},
						   {26.5f,26.9f,-3.7f,93.0f,15.0f,1},
						   {26.2f,34.0f,1.4f,83.0f,24.0f,1},
						   {36.3f,14.8f,36.1f,92.0f,23.0f,1},
						   {37.3f,15.0f,35.6f,90.0f,23.0f,1},
						   {34.0f,16.0f,-7.3f,110.0f,18.0f,1},
						   {37.9f,36.7f,-15.3f,106.0f,20.0f,1},
						   {34.0f,20.0f,-7.3f,111.0f,18.0f,1},
						   {37.4f,34.7f,-10.8f,110.0f,27.0f,1},
						   {32.0f,30.0f,-1.6f,106.0f,27.0f,1},
						   {23.3f,19.4f,0.0f,94.0f,8.0f,1}};
void ShootCtr(shootCtr_t *shootPara)
{
	if(shootPara->yawAng < -50.0f)shootPara->yawAng = -50.0f;
	if(shootPara->yawAng> 50.0f)shootPara->yawAng= 50.0f;
	if(shootPara->pitchAng < 15.0f)shootPara->pitchAng= 15.0f;
	if(shootPara->pitchAng> 40.0f)shootPara->pitchAng= 40.0f;
	if(shootPara->rollAng < 0.0f)shootPara->rollAng= 0.0f;
	if(shootPara->rollAng> 45.0f)shootPara->rollAng= 45.0f;
	switch(shootPara->gunNum)
	{
		case 1:
			PosCrl(8,0,(int32_t)((50.0f + shootPara->yawAng) * 102.4f));
			PosCrl(6,0,(int32_t)((shootPara->pitchAng - 15.0f) * 141.0844f));
			PosCrl(7,0,(int32_t)(shootPara->rollAng* 141.0844f));
			VelCrl(4, -4096*shootPara->vel1);
			VelCrl(5,  4096*shootPara->vel2);
			break;
		default:
			break;
	}
}

