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


///* SPI通讯超时阈值 -----------------------------------------------------------*/
//#define SPI_TIME_OUT_THRESHOLD		( ( uint16_t ) 5000)	
//typedef uint8_t						SpiErr_t;



///* SPI错误类型定义------------------------------------------------------------*/
//typedef enum SPI_Err_Type{
//	SPI_ERR_NoError 				= 0x00,
//	SPI_ERR_SendTimeOut				= 0x01,
//	SPI_ERR_ReceiveTimeOut			= 0x02,
//	SPI_ERR_WaitingToSendTimeOut	= 0x03,
//	SPI_ERR_SendRegAddrTimeOut		= 0x04,
//	SPI_ERR_SendDataTimeOut			= 0x05,
//	SPI_ERR_ReadTimeOut				= 0x06,
//}SpiErrType;

///* SPI收发函数定义------------------------------------------------------------*/
//void SPI1_Init(void);
//uint8_t SPI_RW_1Byte(SPI_TypeDef* SPIx, uint8_t ucSrc, SpiErr_t* pSpiErr);
//uint8_t SPI_Send( SPI_TypeDef* SPIx, uint8_t *pbSrc, uint8_t size );
//uint8_t SPI_Receive( SPI_TypeDef* SPIx, uint8_t *pbDst, uint8_t size );


#define SPI_CLK_GPIO_PORT			GPIOA
#define SPI_CLK_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define SPI_CLK_GPIO_PIN			GPIO_Pin_5
#define SPI_CLK_AF_PIN				GPIO_PinSource5

#define SPI_SDI_GPIO_PORT			GPIOA
#define SPI_SDI_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define SPI_SDI_GPIO_PIN			GPIO_Pin_6
#define SPI_SDI_AF_PIN				GPIO_PinSource6

#define SPI_SDO_GPIO_PORT			GPIOA
#define SPI_SDO_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define SPI_SDO_GPIO_PIN			GPIO_Pin_7
#define SPI_SDO_AF_PIN				GPIO_PinSource7




//SPI接口定义
#define SPI_PORT					SPI1						//SPI接口
#define GPIO_AF_SPI					GPIO_AF_SPI1
#define SPI_PORT_CLK				RCC_APB2Periph_SPI1			//SPI时钟

void SYS_SPI_Init( void );
uint8_t SPI_ByteIO( uint8_t TxByte );
void SPI_BufIO( uint8_t* ReadBuffer, uint8_t* WriteBuffer, uint16_t Length );


#endif
