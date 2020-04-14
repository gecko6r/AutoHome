/**
  ***********************************UTF-8***************************************
  * @file    i2c.h
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于定义STM32的IIC操作函数
  ******************************************************************************  
  */ 

#ifndef _PERIPHERAL_I2C_H
#define _PERIPHERAL_I2C_H

#include "sys.h"

#define I2C_TIME_OUT		((uint16_t) 1000)	//I2C通讯超时阈值	

typedef enum I2C_Err_Type{
	I2C_ERR_NoError 				= 0,
	I2C_ERR_Busy					= 1,
	I2C_ERR_StartTimeOut			= 2,
	I2C_ERR_SendSlaveAddrTimeOut	= 4,
	I2C_ERR_SendRegAddrTimeOut		= 8,
	I2C_ERR_SendDataTimeOut			= 16,
	I2C_ERR_ReadTimeOut				= 32,
	
	
}I2cErrType_t;


extern I2cErrType_t i2cError;


void IIC_Init(void);

uint8_t I2C_ByteWrite(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,\
					uint8_t ucData, I2cErrType_t* err);
uint8_t I2C_ByteRead(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr,\
							uint8_t ucRegAddr, I2cErrType_t* err);
uint8_t I2C_MultiWrite(I2C_TypeDef * I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,\
						uint8_t ucNumToWrite, uint8_t* pucBuffer, I2cErrType_t* err);
uint8_t I2C_MultiRead(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,\
					uint8_t ucNumToRead, uint8_t* pucBuffer, I2cErrType_t* err);
#endif
