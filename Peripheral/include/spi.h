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
#define SPI_TIME_OUT_THRESHOLD	((uint16_t) 100)	
#define SpiErrType_t			SpiErrType



/* SPI错误类型定义------------------------------------------------------------*/
typedef enum SPI_Err_Type{
	SPI_ERR_NoError 				= 0,
	SPI_ERR_SendTimeOut				= 1,
	SPI_ERR_ReceiveTimeOut			= 2,
	SPI_ERR_WaitingToSendTimeOut	= 4,
	SPI_ERR_SendRegAddrTimeOut		= 8,
	SPI_ERR_SendDataTimeOut			= 16,
	SPI_ERR_ReadTimeOut				= 32,
}SpiErrType;

/* SPI收发函数定义------------------------------------------------------------*/
void SPI1_Init(void);
void SPI_Write(SPI_TypeDef* SPIx, uint8_t ucSrc, SpiErrType_t* pSpiErr);
uint8_t SPI_Read(SPI_TypeDef* SPIx, SpiErrType_t* pSpiErr);

u8 SPI_RW(u8 data);
uint8_t NRF_Read_Reg(uint8_t reg);
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value);

#endif
