/**
  ***********************************UTF-8**************************************
  * @file    dataAcquisition.c
  * @author  Xiong
  * @version V1.0
  * @date    16-July-2020
  * @brief   此文件用于定义机器人运动控制函数
  ******************************************************************************  
  */ 
  
#include "controller.h"
#include "usart.h"

/* 机器人运动控制全局变量定义 ------------------------------------------------*/
/* 机器人类型定义, 如果未定义，则定义为壁虎机器人*/
#if defined  GECKO_ROBOT
#undef HEXAPOD_ROBOT
RobotType_t RobotType = GeckoRobot;
#define TipsCount			( ( uint8_t ) 4 )



#elif defined HEXAPOD_ROBOT
RobotType_t RobotType = HexapodRobot;
#define TipsCount			( ( uint8_t ) 6 )



#else
#error "No available robot type defined!"
#endif


/* 机器人运动控制局部变量定义 ------------------------------------------------*/
static TipPosType* xTipPosBuf[TipsCount];















