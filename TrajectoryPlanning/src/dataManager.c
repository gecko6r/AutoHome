/**
  ***********************************UTF-8**************************************
  * @file    dataBuffer.c
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于定义机器人通讯的数据缓冲区和读写函数
  ******************************************************************************  
  */ 

#include "FreeRTOS.h"
#include "queue.h"

#include "dataManager.h"

uint8_t ucServoTxBuffer[MAX_BUF_SIZE];
uint8_t* pServoTx = ucServoTxBuffer;

uint8_t ucServoRxBuffer[MAX_BUF_SIZE];
uint8_t* pServoRx = ucServoRxBuffer;

float ucServoTorqueBuffer[SERVO_NUM];
float* pServoTorque = ucServoTorqueBuffer;


/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化DataManager
	* @param  	None
	* @retval 	成功则返回0
				否则返回1
	*/
uint8_t DM_Init(void)
{
	
	
	return 0;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	写入指定数组
	* @param  	None
	* @retval 	None
	*/	
uint8_t DM_SetBuffer(uint8_t* ucSrcBuf, uint8_t ucSize,	uint8_t ucBlockMs)
{
	
	return 0;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	flush指定数组
	* @param  	None
	* @retval 	None
	*/	
void DM_Flush(uint8_t* pucBuf, uint8_t ucLen, TaskFunction_t periphFlush)
{
	
}
