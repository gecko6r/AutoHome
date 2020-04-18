/**
  ***********************************UTF-8**************************************
  * @file    dataAcquisition.c
  * @author  Xiong
  * @version V0.3
  * @date    16-July-2020
  * @brief   此文件用于定义机器人运动控制的相关类型和结构
  ******************************************************************************  
  */ 
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "motionTypes.h"

/* 机器人运动控制类型定义 ----------------------------------------------------*/

/* 机器人运动模式定义*/
typedef enum 
{
	/*连续模式*/
	Continues_Mode,
	/*单点模式*/
	Single_Point_Mode,
}ControlModeType;

/* 机器人类型定义*/
typedef enum
{
	GeckoRobot,
	HexapodRobot
}RobotType_t;

/* 机器人运动控制宏定义 ------------------------------------------------*/
#define GECKO_ROBOT
/*
#define HEXAPOD_ROBOT
*/

#ifdef GECKO_ROBOT
#define SERVO_NUM				( ( uint8_t ) 16 )
#define JOINTS_PER_LEG			( ( uint8_t ) 3 )
#elif defined HEXAPOD_ROBOT
#define SERVO_NUM				( ( uint8_t ) 18 )
#define JOINTS_PER_LEG			( ( uint8_t ) 3 )
#endif

/* 机器人运动控制全局变量声明 ------------------------------------------------*/


/* 机器人运动控制函数定义 ----------------------------------------------------*/
uint8_t CTRL_WriteTipPosToBuffer( TipPosType* xTipPosBuf, uint8_t ucTipCount);
uint8_t CTRL_Action(void);
uint8_t CTRL_ActionAfter_ms(uint16_t usNms);


#endif
