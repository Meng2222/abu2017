#ifndef __ROBOT_H
#define __ROBOT_H
#include "movebase.h"
/**************************************************************************************
 �궨��
**************************************************************************************/

//��½̨����
#define LAND_NUMBER 7
//������ǹ�ĸ���
#define GUN_NUMBER 3
//�ӵ���ǹ����״̬���࣬�ɹ��ߴ�����
#define BULLET_TYPE_NUMBER 10

#define LEFT_GUN				1
#define RIGHT_GUN				2
#define UPPER_GUN				3

//��ǹ��ID��
#define LEFT_GUN_GROUP_ID		101
//��ǹ��ID��
#define RIGHT_GUN_GROUP_ID		102
//����ǹ��ID��
#define UPPER_GUN_GROUP_ID		103

#define GUN_AUTO_MODE			0
#define GUN_MANUAL_MODE			1

#define GUN_START_SHOOT 1
#define GUN_STOP_SHOOT 0

//��ǹ֧��roll��CAN ID
#define LEFT_GUN_ROLL_ID 7
//��ǹ֧��pitch��CAN ID
#define LEFT_GUN_PITCH_ID 6
//��ǹ֧��yaw��CAN ID
#define LEFT_GUN_YAW_ID 8
//��ǹ��ഫ�ʹ���CAN ID
#define LEFT_GUN_LEFT_ID 4
//��ǹ�Ҳഫ�ʹ���CAN ID
#define LEFT_GUN_RIGHT_ID 5

//��ǹ֧��roll��CAN ID
#define RIGHT_GUN_ROLL_ID 7
//��ǹ֧��pitch��CAN ID
#define RIGHT_GUN_PITCH_ID 6
//��ǹ֧��yaw��CAN ID
#define RIGHT_GUN_YAW_ID 8
//��ǹ��ഫ�ʹ���CAN ID
#define RIGHT_GUN_LEFT_ID 4
//��ǹ�Ҳഫ�ʹ���CAN ID
#define RIGHT_GUN_RIGHT_ID 5

//����ǹ֧��pitch��CAN ID
#define UPPER_GUN_PITCH_ID 11
//����ǹ֧��yaw��CAN ID
#define UPPER_GUN_YAW_ID 10
//����ǹ��ഫ�ʹ���CAN ID
#define UPPER_GUN_LEFT_ID 9


//������
#define GUN_NO_ERROR 0
//ǹ�ſ���
#define GUN_CHAMPER_ERROR -1
//���п�
#define GUN_NO_BULLET_ERROR -2
//ǹ��δ��λ�� fix me
#define GUN_NO_READY_ERROR -3
//�ϵ�ʧ��
#define GUN_RELOAD_ERROR -4
//�򿪱���ʧ��
#define GUN_OPEN_SAFETY_ERROR -5

//ǹ����ӵ���
#define MAX_BULLET_NUMBER 20
//ǹ����Զ������ӵ�����
#define MAX_AUTO_BULLET_NUMBER 10

//��׼δ���
#define GUN_AIM_IN_PROCESS 1
//��׼���
#define GUN_AIM_DONE 2

//�ӵ����������
#define CHAMPER_BULLET_MAX_FEATURE_STATE 10

//ǹ�����ӵ�
#define CHAMPER_BULLET_EMPTY_STATE 0xff

//ǹ�����ӵ�������0������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE0_STATE 0

//ǹ�����ӵ�������1������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE1_STATE 1

//ǹ�����ӵ�������2������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE2_STATE 2

//ǹ�����ӵ�������3������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE3_STATE 3

//ǹ�����ӵ�������4������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE4_STATE 4

//ǹ�����ӵ�������5������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE5_STATE 5

//ǹ�����ӵ�������6������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE6_STATE 6

//ǹ�����ӵ�������7������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE7_STATE 7

//ǹ�����ӵ�������8������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE8_STATE 8

//ǹ�����ӵ�������9������Ҫ��ϸ���������ݲ�ͬ��״̬�����ǹ����̬
#define CHAMPER_BULLET_FEATURE9_STATE 9

//���������ϵ�
#define ROBOT_STAGE_POWER_ON 0
//�������Ѿ���ʼ��
#define ROBOT_STAGE_INIT 1
//�����˴��ڼ��ٹ�����
#define ROBOT_STAGE_RUN_ACC 2
//�����˴��������˶��׶�
#define ROBOT_STAGE_RUN_UNIFORM 3
//�����˽�����ٽ׶�
#define ROBOT_STAGE_RUN_DEC 4
//������ץȡ�ӵ�
#define ROBOT_STAGE_LOAD_GUN 5
//�����˽�����ת���ϣ��൱��������
#define ROBOT_STAGE_OPEN_GUN_SAFETY 6
//������ֹͣ���������˶�
#define ROBOT_END_RUNNING 7
//�����˿�ʼ���
#define ROBOT_START_SHOOT 8

#define ROBOT_STATUS_OK 0
#define ROBOT_STATUS_OVER_LOAD 0x01
#define ROBOT_STATUS_OVER_TEMPERATURE 0x02
#define ROBOT_STATUS_UNDER_VOLTAGE 0x04
#define ROBOT_STATUS_GAS_LOW  0x08

#define INVALID_PLANT_NUMBER 7
#define PLANT1 0
#define PLANT2 1
#define PLANT3 2
#define PLANT4 3
#define PLANT5 4
#define PLANT6 5
#define PLANT7 6

//ǹ�Զ����ʱ��Ŀ������˳��
typedef struct
{
	unsigned char cmd[MAX_AUTO_BULLET_NUMBER];
}shoot_command_t;

/**************************************************************************************
 ���Ͷ���
**************************************************************************************/

typedef int status_t;

typedef struct
{
	//ǹ����Ƕ�
	float yaw;
	//ǹ�����Ƕ�
	float pitch;
	//ǹ����Ƕ�
	float roll;

	

	//���ʹ�ת�٣���λת/��
	float speed1;
	//�Ҵ��ʹ�ת�٣���λת/��
	float speed2;
}gun_pose_t;
/*
* ǹ�ṹ��
*/
typedef struct
{
	//ǹ��Ŀ����̬
	gun_pose_t targetPose;
	//ǹ��Ŀ����̬
	gun_pose_t actualPose;
	
	//ǹ��̬����
	gun_pose_t maxPoseLimit;
	
	//ǹ��̬����
	gun_pose_t minPoseLimit;
	
	//ǹ֧���Ƿ�׼���ã�1����0׼����
	unsigned char ready;
	
	//ǹ��ģʽ��0�Զ���1�ֶ�
	unsigned char mode;
	
	//�������ӵ�������Ϊ���23����������8λ
	unsigned char bulletNumber;
	
	//ǹ���ӵ�״̬�����ݲ�ͬ״̬������ǹ��ʽ
	unsigned char champerBulletState;
	
	//ǹ���Ƿ񿨵�:1������0����
	unsigned char champerErrerState;
	
	//������1�����0�����
	unsigned char shoot;
	//�������char��������4�ı�������Ҫʹ��dummy���룬dummyû���κκ���
	//unsigned char dummy[2];
	
	//gunPosDatabaseָ��ǹ����̬���ݿ⣬���ݿ�洢��database.c�У���ʼ��ʱ��Ҫָ��
	gun_pose_t **gunPoseDatabase;
	shoot_command_t *shootCommand;
	//Ŀ����½̨�ţ�ֻ�����ֶ�ģʽ�²���Ч���Զ�ģʽ�º���
	int targetPlant;
	//�������
	int shootTimes;
	
}gun_t;

//7#��½̨����λ�ýṹ��
typedef struct
{
	//����0�Ƿ����̣�0û�У�1��
	unsigned char area0;
	//����1�Ƿ����̣�0û�У�1��
	unsigned char area1;
	//����2�Ƿ����̣�0û�У�1��
	unsigned char area2;
	//����3�Ƿ����̣�0û�У�1��
	unsigned char area3;
	//����4�Ƿ����̣�0û�У�1��
	unsigned char area4;
	//����5�Ƿ����̣�0û�У�1��
	unsigned char area5;
	//����6�Ƿ����̣�0û�У�1��
	unsigned char area6;
}platePosOnLand7_t;

//�����˽ṹ���װ�˻����Ĺؼ����ݣ�Ϊȫ�����ݣ��˽ṹ����ʱ���ڴ˴�
typedef struct 
{
	
	movebase_t moveBase;
	//���������ǹ
	gun_t leftGun;
	//�������ұ�ǹ
	gun_t rightGun;
	//�������ϱ�ǹ
	gun_t upperGun;

	//���������漰������Ϊ(leftGun.shootTimes+rightGun.shootTimes + upperGun.shootTimes)
	int shootTimes;

	//�ߺ���½̨������λ��
	platePosOnLand7_t platePosOnLand7;
	
	//�����������Ľ׶Σ�δ��ʼ������ʼ�������٣����٣����٣�ȡ������ս������ǹ
	int stage;
	//������״̬���Ƿ���������ѹ�����������¡�
	int status;
}robot_t;

/*
* ��½̨����
*/
typedef struct
{
	int size;
}land_t;
/*
* ������Ϣ���ߴ磬������½̨
*/
typedef struct
{
	int size;
}field_t;


/**************************************************************************************
 �������壬���»����˲����������дrobot_t���͵Ļ�����ʵ������
����������������ÿ��ǹ�Ĳ���������Ҫ֧������
**************************************************************************************/
/*
*���ƣ�ROBOT_Init
*���ܣ������˳�ʼ������ʼ�����̣���ʼ��ǹ����ʼ��
*������none
*ע�⣺�����ǹ����Ҫ���ӵ�����Ϊ���ֶ��ϵ�
*/
status_t ROBOT_Init(void);

/*
*���ƣ�ROBOT_GunLoad
*���ܣ���װ���У���ץȡ�ӵ�����
*������
*status:GUN_NO_ERROR��GUN_RELOAD_ERROR
*ע�⣺�����ǹ����Ҫ���ӵ�����Ϊ���ֶ��ϵ�
*/
status_t ROBOT_GunLoad(void);

/*
*���ƣ�ROBOT_GunOpenSafety
*���ܣ�����ǹ���գ��ӵ���װ�ú���ܽ��д˲���
*������
*status:GUN_NO_ERROR��GUN_OPEN_SAFETY_ERROR
*ע�⣺�����ǹ����Ҫ
*/
status_t ROBOT_GunOpenSafety(void);

/*
*���ƣ�ROBOT_GunReload
*���ܣ���ǹ�ϵ���ÿ�����ǰ��Ҫ�ϵ�һ��
*������
*gun :LEFT_GUN, RIGHT_GUN
*status:GUN_NO_ERROR��GUN_RELOAD_ERROR
*ע�⣺�����ǹ����Ҫ���ӵ�
*/
status_t ROBOT_GunReload(unsigned char gun);

/*
*���ƣ�ROBOT_GunReload
*���ܣ�����ǹ�Ŵ�����������ӵ�״̬���������濪ǹ�ľ������
*������
*gun :LEFT_GUN, RIGHT_GUN
*status:GUN_NO_ERROR��GUN_RELOAD_ERROR
*ע�⣺�����ǹ����Ҫ���ӵ�����Ϊ���ֶ��ϵ�
*/
status_t ROBOT_GunCheckBulletState(unsigned char gun);

/*
*���ƣ�ROBOT_GunAim
*���ܣ���׼��Ŀ��ı����Ҫ�ȵ��ô˽ӿ���������׼
*������
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*landId:
*status:GUN_NO_ERROR
*ע�⣺�����ǹĿǰ��е��û��roll��û���Ҳഫ�ʹ�speed2
*/
status_t ROBOT_GunAim(unsigned char gun);
/*
*���ƣ�ROBOT_LeftGunCheckAim
*���ܣ������׼�Ƿ������
*������
*none
*status:GUN_AIM_IN_PROCESS�� GUN_AIM_DONE
*ע�⣺
*/
status_t ROBOT_LeftGunCheckAim(void);
/*
*���ƣ�ROBOT_RightGunCheckAim
*���ܣ������׼�Ƿ�����ɣ���ͬǹ�ֿ����Ϊ�˷�ֹ���룬
*��Ϊ�˺�������Ҫ��Ƴ�ʱ
*������
*none
*status:GUN_AIM_IN_PROCESS�� GUN_AIM_DONE
*ע�⣺
*/
status_t ROBOT_RightGunCheckAim(void);

/*
*���ƣ�ROBOT_GunShoot
*���ܣ���ǹ����ǹǰ��Ҫȷ���ӵ����ţ��������գ�ǹ֧���Ѿ�����
*������
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*mode: auto or manual
*status:GUN_NO_ERROR��GUN_CHAMPER_ERROR�� GUN_NO_BULLET_ERROR�� GUN_NO_READY_ERROR
*/
status_t ROBOT_GunShoot(unsigned char gun, unsigned char mode);


/*
*���ƣ�ROBOT_GunHome
*���ܣ�ǹ��λ����ǹ��Ϊ�˸��õ�������Ҫ��λ
*������
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*status:GUN_NO_ERROR
*/
status_t ROBOT_GunHome(unsigned char gun);

/*
*���ƣ�ROBOT_GunCheckMode
*���ܣ����ǹ��ģʽ
*������
*gun :LEFT_GUN, RIGHT_GUN, UPPER_GUN
*status:
*/
status_t ROBOT_GunCheckMode(unsigned char gun);

int LeftGunYawTransform(float position);
int LeftGunPitchTransform(float position);
int LeftGunRollTransform(float position);
int LeftGunLeftSpeedTransform(float speed);
int LeftGunRightSpeedTransform(float speed);

float LeftGunYawInverseTransform(int position);
float LeftGunPitchInverseTransform(int position);
float LeftGunRollInverseTransform(int position);
float LeftGunLeftSpeedInverseTransform(int speed);
float LeftGunRightSpeedInverseTransform(int speed);


int RightGunYawTransform(float yaw);
int RightGunPitchTransform(float pitch);
int RightGunRollTransform(float roll);
int RightGunLeftSpeedTransform(float speed);
int RightGunRightSpeedTransform(float speed);

float RightGunYawInverseTransform(float position);
float RightGunPitchInverseTransform(float position);
float RightGunRollInverseTransform(float position);
float RightGunLeftSpeedInverseTransform(float speed);
float RightGunRightSpeedInverseTransform(float speed);

int UpperGunLeftSpeedTransform(float speed);
int UpperGunYawTransform(float yaw);
int UpperGunPitchTransform(float pitch);


//temrary
void ShootCtr(shootCtr_t *shootPara);

#endif
