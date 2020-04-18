/**
  ***********************************UTF-8**************************************
  * @file    dataBuffer.h
  * @author  Xiong
  * @version V1.0
  * @date    18-July-2020
  * @brief   此文件用于定义机器人通讯的数据缓冲区和读写函数
  ******************************************************************************  
  */ 

#ifndef _DATA_MANAGER_H
#define _DATA_MANAGER_H


#include "FreeRTOS.h"
#include "semphr.h"

#include "sys.h"
#include "controller.h"

#include "i2c.h"
#include "dma.h"
#include "usart.h"



typedef struct _bufferMange
{
	void* 				pBufAddr;
	uint16_t 			usDataLen;
	bool 				bBufSetted;
	SemaphoreHandle_t 	xSemph;
}DataManagerType;

extern uint8_t ucServoTxBuffer[MAX_BUF_SIZE];

extern uint8_t ucServoRxBuffer[MAX_BUF_SIZE];

extern float ucServoTorqueBuffer[SERVO_NUM];



uint8_t DM_Init(void);
uint8_t DM_SetBuffer(uint8_t* ucSrcBuf, uint8_t ucSize, uint8_t ucBlockMs);


void DM_Flush(uint8_t* pucBuf, uint8_t ucLen, TaskFunction_t periphFlush);






#endif
