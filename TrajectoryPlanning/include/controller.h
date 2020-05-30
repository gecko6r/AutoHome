/**
  ***********************************UTF-8**************************************
  * @file    dataAcquisition.c
  * @author  Xiong
  * @version V0.3
  * @date    16-July-2020
  * @brief   此文件用于定义机器人运动控制的相关类型和结构，所有函数的参数和返回
  *			 中的角度单位都是°，距离或长度单位都是mm
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
}ControlMode_t;

/* 机器人类型定义*/
typedef enum
{
	GeckoRobot,
	HexapodRobot
}Robot_t;

/* 机器人运动控制宏定义 ------------------------------------------------*/
#define GECKO_ROBOT
/*
#define HEXAPOD_ROBOT
*/

#ifdef  GECKO_ROBOT
#define ctrlSERVO_NUM				( ( uint8_t ) 16 )
#define ctrlJOINTS_PER_LEG			( ( uint8_t ) 4 )
#define ctrlLEG_COUNT				( ( uint8_t ) 4 )
#define ctrlJOINT1_UP_LIMIT			( ( double ) 95 )
#define ctrlJOINT1_LOW_LIMIT		( ( double ) -95 )
#elif defined HEXAPOD_ROBOT
#define ctrlSERVO_NUM				( ( uint8_t ) 18 )
#define ctrlJOINTS_PER_LEG			( ( uint8_t ) 3 )
#define ctrlLEG_COUNT				( ( uint8_t ) 6 )
#endif


/************************** 机器人运动控制全局变量声明 ************************/
extern uint32_t uxServoPosBuf[ctrlSERVO_NUM];

/************************** 机器人运动控制全局函数定义 ************************/
uint8_t CTRL_WriteTipPosToBuf( TipPos_t* xTipPosBuf);
uint8_t CTRL_Action(void);
uint8_t CTRL_ActionAfter_ms(uint16_t usNms);

void CTRL_SetTipsPos( TipPos_t xTipPosBuf[ ctrlLEG_COUNT ] );

/****************************** 运动学逆解函数定义 ****************************/
void CTRL_InverseKinemix( TipPos_t xTipPosBuf[ctrlLEG_COUNT], 
						  LegAngle_t* xDstBuf);
LegAngle_t CTRL_SingleLegIK( TipPos_t xPoint );
void CTRL_LegAngTypeToDouble( LegAngle_t* xBuf, uint8_t ucCount, 
							  double* dDstBuf );
void CTRL_DoubleToPos( double* pbSrc, uint8_t ucCount, uint32_t* pbDst );

void CTRL_PoseBasedIK( BodyPose_t xBodyPose, Point3d xBodyCenter, 
					   Point3d* xTipGroundPos , LegAngle_t* xLegAngleBuf);

/************************** 机器人运动控制局部函数定义 ************************/


#endif
