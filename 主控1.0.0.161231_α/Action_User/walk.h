
/**
  ******************************************************************************
  * @file    
  * @author  Tzy
  * @version 
  * @date   
  * @brief   This file contains the headers of action_math.c
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __WALK_H__
#define  __WALK_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
 #define ACTVEL       1600
 #define ACTROTATE    ACTVEL*0.2
 #define HIGHEST      210000
 
 #define X1           1350//1582
 #define X2           4800
 
 #define Y1           2670-400
 #define Y2           3800-400
 #define Y3           4600-450
 #define Y4           5278-420
 #define Y5           6700
 #define Y6           9187
 
 #define LASERVALUE    (15.9*(347-162)-777.4)
 
 #define RADIUS1       1264//1100
 #define RADIUS2      -1997
 #define RADIUS3      -2000
 
 #define RADIUS4     -4150
 #define RADIUS5     -3000
 
 #define P_POSITION         0.25
 #define P_POSITION_LASER   0.08
 #define I_POSITION_LASER   0.00
 #define P_ANGLE            40.0
 #define I_ANGLE            0.00
       

 #define BASIC_WIND_SPEED   0.075*2000
 #define BASIC_INIT_ANGLE   5.0     //函道舵机初始角度
 
 
 
 //以下宏定义一般不用改动
 #define ACCURATEX        Get_LaserDistance(Get_LaserValue_TEMP())
 #define ACCURATEY        7011
 
 #define ISROTATE            1   //要自转
 #define NOROTATE            0   //不自转
 
 #define PLEFT              -1   //与左边保持距离
 #define PRIGHT              1   //与右边保持距离
 
 #define CIRCLE_END          1   //画完圆
 
 #define XDIRECTION         90  //X方向
 #define YDIRECTION          0  //Y方向

 #define PID_TAN      ((Get_POS_Xtemp()-pos_x)/(pos_y-Get_POS_Xtemp()))
 
 #define SIGHT_RANGE         80.0
 #define UD_MAX_SPEED        250000
 
 #define LASER_RANGE         3500.0
 
 #define ROBS_HIGH_SPEED         60.0
 #define ROBS_MID_SPEED          45.0
 #define ROBS_LOW_SPEED          30.0
 
 #define PROTECT_Y           12000
 #define PROTECT_LASER_MAX   200
 #define PROTECT_LASER_MIN   50
 
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void LockWheel(void);
void BasicLine(int Vel,float ward,float Rotate,float selfAngle);
void CloseLoopLine(float Vel,int Ward,int Refvalue,int Curvalue,float Refangle,float Curangle ,float pPosition,float iPosition,int pAngle,float iAngle,int8_t Flag);
void StartLine(float Vel,int Ward,int Refvalue,int Curvalue,float Curangle,int PosX);
int8_t StartCircle(int Vel,int WardInit,int WardEnd,float Radius,int8_t IsRotate,int pAngle,float iAngle);
int8_t BasicCircle(int Vel,int WardInit,int WardEnd,float Radius,int8_t IsRotate,int pAngle,float iAngle);
void UpdateUpDown(float PosX,float PosY,int CurHeight);
void UpdateAngle(float PosX,float PosY);
void UpdateWindSpeed(float PosX,float PosY);
void AdjPosX(float PosX,int LaserValue);
void AdjPosY(float PosY);

void Height_CameraAdj(int height_now,int height_sight,int max,int min);

void PosCrl_mm(int Dis);
int Get_LaserDistance(float LaserValue);
#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/

