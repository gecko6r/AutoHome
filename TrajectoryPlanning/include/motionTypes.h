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


/* 机器人运动控制类型定义 ----------------------------------------------------*/
/* 腿部三个舵机角度结构体*/
typedef struct 
{
	float rootJoint;
	float midJoint;
	float endJoint;
}LegAngleType;

/* 三维点定义*/
typedef struct
{
	float x;
	float y;
	float z;
}Point3d;




/* 机器人运动控制类型宏定义 ----------------------------------------------------*/
#define TipPosType			Point3d

#endif
