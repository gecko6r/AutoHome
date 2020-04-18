/**
  ***********************************UTF-8***************************************
  * @file    spi.c
  * @author  Xiong
  * @version V1.0
  * @date    16-July-2020
  * @brief   此文件用于定义STM32的SPI操作函数
  * @note	 文件仅用于NRF24L01+的数据收发，因此SPI_Write(  )、SPI_Read(  )函数
			只允收发uint8_t类型的数据
  ******************************************************************************  
  */ 
#include "spi.h"
#include "usart.h"
#include "led.h"

static uint16_t SPI_TIME_COUNT = 0;

/****
	* @brief	初始化SPI1，
    * @param  	None
    * @retval 	None
    */
void SPI1_Init( void )
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure ); 
	

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	
	
	/*SPI设置为双线双向全双工*/
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex;
	/*SPI设置为主SPI*/
	SPI_InitStructure.SPI_Mode 				= SPI_Mode_Master;
	/*SPI数据宽度为8位*/
	SPI_InitStructure.SPI_DataSize 			= SPI_DataSize_8b;
	/*SPI同步时钟的空闲状态为低电平*/
	SPI_InitStructure.SPI_CPOL 				= SPI_CPOL_Low;
	/*SPI同步时钟的上升沿采集数据*/
	SPI_InitStructure.SPI_CPHA 				= SPI_CPHA_1Edge;
	/*SPI NSS信号由软件管理*/
	SPI_InitStructure.SPI_NSS 				= SPI_NSS_Soft;
	/*SPI波特率预分频值为64*/
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	/*SPI数据传输从MSB开始*/
	SPI_InitStructure.SPI_FirstBit 			= SPI_FirstBit_MSB;
	/*SPI CRC计算多项式*/
	SPI_InitStructure.SPI_CRCPolynomial 	= 7;
	SPI_Init( SPI1, &SPI_InitStructure );
	
	SPI_Cmd( SPI1, ENABLE );


}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	SPI发送并接收一个字节
    * @param  	ucSrc：待发送的字节数据
	* @param	pSpiErr：SPI错误类型指针
    * @retval 	SPI收到的字节
    */
uint8_t SPI_RW_1Byte(SPI_TypeDef* SPIx, uint8_t ucSrc, SpiErr_t* pSpiErr)
{	
	/*防止等待数据发送超时*/
	SPI_TIME_COUNT = 0;
	while( SPI_GetFlagStatus( SPIx, SPI_FLAG_TXE ) == RESET )
	{
		SPI_TIME_COUNT++;
		
		if( SPI_TIME_COUNT > SPI_TIME_OUT_THRESHOLD )
		{
			*pSpiErr |= SPI_ERR_WaitingToSendTimeOut;
			return 0;
		}
	}
	
	SPI_SendData( SPIx, ucSrc );
	
	SPI_TIME_COUNT = 0;
	while( SPI_GetFlagStatus( SPIx, SPI_FLAG_RXNE ) == RESET )
	{
		SPI_TIME_COUNT++;
		
		if( SPI_TIME_COUNT > SPI_TIME_OUT_THRESHOLD )
		{
			*pSpiErr |= SPI_ERR_ReadTimeOut;
			return 0;
		}
	}
	
	return ( uint8_t ) SPI_ReceiveData( SPIx );
	
}
