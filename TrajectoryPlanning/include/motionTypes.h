/**
  ***********************************UTF-8**************************************
  * @file    motionTypes.h
  * @author  Xiong
  * @version V1.0
  * @date    16-July-2020
  * @brief   此文件用于定义机器人运动控制的相关类型和结构
  ******************************************************************************  
  */ 

#ifndef _MOTION_TYPES_H
#define _MOTION_TYPES_H

#include "sys.h"

/* 机器人运动控制宏定义 ------------------------------------------------------*/
#define PI		( ( double ) 3.1415926535898 ) 

/* 机器人运动控制类型定义 ----------------------------------------------------*/
/* 腿部三个舵机角度结构体*/
typedef struct 
{
	double rootJoint;
	double midJoint;
	double endJoint;
	double stringJoint;
}LegAngle_t;

/* 三维点定义*/
typedef struct
{
	double x;
	double y;
	double z;
}Point3d;

/* 躯干rpy角度定义*/
typedef struct BodyPose
{
	double roll;
	double pitch;
	double yaw;
}BodyPose_t;


/* 机器人运动控制类型宏定义 ----------------------------------------------------*/
#define TipPosType			Point3d

#endif
