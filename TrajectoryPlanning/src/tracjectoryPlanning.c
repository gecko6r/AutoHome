/**
  ***********************************UTF-8**************************************
  * @file    trajectoryPlanning.c
  * @author  Xiong
  * @version V1.0
  * @date    10-Aug-2020
  * @brief   此文件用于定义机器人步态轨迹生成函数
  ******************************************************************************  
  */ 

#include "trajectoryPlanning.h"

static uint16_t Period = 300;
static double StepHeight = 40.0;
static double StepLen = 60.0;

static double StepHeightLimit[ 2 ] = { 5.0, 50.0 };
static double StepLenLimit[ 2 ] = { 0.0, 60.0 };

static int speed;




int TP_TripodGait( double time )
{
	
	
	return 0;
}







