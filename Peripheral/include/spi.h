/**
  ***********************************UTF-8***************************************
  * @file    spi.h
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于定义STM32的SPI操作函数
  ******************************************************************************  
  */ 
#ifndef _PERIPHERAL_SPI_H
#define _PERIPHERAL_SPI_H

#include "sys.h"


/* SPI通讯超时阈值 -----------------------------------------------------------*/
#define SPI_TIME_OUT_THRESHOLD		( ( uint16_t ) 500)	
#define SpiErr_t					SpiErrType



/* SPI错误类型定义------------------------------------------------------------*/
typedef enum SPI_Err_Type{
	SPI_ERR_NoError 				= 0x00,
	SPI_ERR_SendTimeOut				= 0x01,
	SPI_ERR_ReceiveTimeOut			= 0x02,
	SPI_ERR_WaitingToSendTimeOut	= 0x04,
	SPI_ERR_SendRegAddrTimeOut		= 0x08,
	SPI_ERR_SendDataTimeOut			= 0x10,
	SPI_ERR_ReadTimeOut				= 0x20,
}SpiErrType;

/* SPI收发函数定义------------------------------------------------------------*/
void SPI1_Init(void);
uint8_t SPI_RW_1Byte(SPI_TypeDef* SPIx, uint8_t ucSrc, SpiErr_t* pSpiErr);


#endif
