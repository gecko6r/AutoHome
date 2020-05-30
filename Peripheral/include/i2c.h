/**
  ***********************************UTF-8**************************************
  * @file    i2c.h
  * @author  Xiong
  * @version V1.0
  * @date    16-July-2020
  * @brief   此文件用于定义STM32的IIC操作函数
  ******************************************************************************  
  */ 

#ifndef _PERIPHERAL_I2C_H
#define _PERIPHERAL_I2C_H

#include "sys.h"

#define I2C_TIME_OUT		( ( uint32_t ) 2000 ) 	//I2C通讯超时阈值	

typedef enum I2C_Err_Type{
	I2C_ERR_NoError 				= 0,
	I2C_ERR_Busy					= 1,
	I2C_ERR_StartTimeOut			= 2,
	I2C_ERR_SendSlaveAddrTimeOut	= 3,
	I2C_ERR_SendRegAddrTimeOut		= 4,
	I2C_ERR_SendDataTimeOut			= 5,
	I2C_ERR_ReadTimeOut				= 6,
	
	
}I2cErr_t;




void IIC_Init(void);

uint8_t I2C_ByteWrite(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,
					uint8_t ucData );
uint8_t I2C_ByteRead(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr,
							uint8_t ucRegAddr, uint8_t *pcDst );
uint8_t I2C_MultiWrite(I2C_TypeDef * I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,
					uint8_t ucNumToWrite, uint8_t* pucBuffer );
uint8_t I2C_MultiRead(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,
					uint8_t ucNumToRead, uint8_t* pucBuffer );
#endif
