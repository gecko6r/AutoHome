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

//static uint16_t SPI_TIME_COUNT = 0;

#define SPI_WAIT_TIMEOUT			((uint16_t)0xFFFF)
void SYS_SPI_Init( void )
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( SPI_CLK_GPIO_CLK 	|\
							SPI_SDI_GPIO_CLK 	|\
							SPI_SDO_GPIO_CLK, ENABLE );
	
	//SPI RCC 初始化
	if( SPI_PORT == SPI1 )	
		RCC_APB2PeriphClockCmd( SPI_PORT_CLK, ENABLE );
	else
		RCC_APB1PeriphClockCmd( SPI_PORT_CLK, ENABLE );
	
	//SPI SCK 引脚初始化
	GPIO_InitStructure.GPIO_Pin 	= SPI_CLK_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_Init( SPI_CLK_GPIO_PORT, &GPIO_InitStructure ); 
	
	//SPI SDI(SDI) 引脚初始化
	GPIO_InitStructure.GPIO_Pin 	= SPI_SDI_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_Init( SPI_SDI_GPIO_PORT, &GPIO_InitStructure ); 
	
	//SPI SDO(SDO) 引脚初始化
	GPIO_InitStructure.GPIO_Pin 	= SPI_SDO_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_100MHz;
	GPIO_Init( SPI_SDO_GPIO_PORT, &GPIO_InitStructure ); 
	

	GPIO_PinAFConfig( SPI_CLK_GPIO_PORT, SPI_CLK_AF_PIN, GPIO_AF_SPI); 
	GPIO_PinAFConfig( SPI_SDI_GPIO_PORT, SPI_SDI_AF_PIN, GPIO_AF_SPI); 
	GPIO_PinAFConfig( SPI_SDO_GPIO_PORT, SPI_SDO_AF_PIN, GPIO_AF_SPI); 
	
	
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
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	/*SPI数据传输从MSB开始*/
	SPI_InitStructure.SPI_FirstBit 			= SPI_FirstBit_MSB;
	/*SPI CRC计算多项式*/
	SPI_InitStructure.SPI_CRCPolynomial 	= 7;
	SPI_Init( SPI_PORT, &SPI_InitStructure );
	
	SPI_Cmd( SPI_PORT, ENABLE );

}

uint8_t SPI_ByteIO( uint8_t TxByte )
{
	uint8_t data = 0;
	uint16_t usWaitTime = 0;
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI_PORT, SPI_FLAG_TXE ) )		//等待发送缓冲区为空
	{
		if( ++usWaitTime == SPI_WAIT_TIMEOUT )
		{
			break;			//如果等待超时则退出
		}
	}
	usWaitTime = SPI_WAIT_TIMEOUT / 2;		//重新设置接收等待时间(因为SPI的速度很快，正常情况下在发送完成之后会立即收到数据，等待时间不需要过长)
	SPI_PORT->DR = TxByte;	//发送数据
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI_PORT, SPI_FLAG_RXNE ) )		//等待接收缓冲区非空
	{
		if( ++usWaitTime == SPI_WAIT_TIMEOUT )
		{
			break;			//如果等待超时则退出
		}
	}
	
	data = (uint8_t)SPI_PORT->DR;		//读取接收数据
	
	return data;		//返回
	
}

void SPI_BufIO( uint8_t* ReadBuffer, uint8_t* WriteBuffer, uint16_t Length )
{
	while( Length-- )
		*ReadBuffer++ = SPI_ByteIO( *WriteBuffer++ );				//收发数据
}





















///****
//	* @brief	初始化SPI1，
//    * @param  	None
//    * @retval 	None
//    */
//void SPI1_Init( void )
//{
//	SPI_InitTypeDef  SPI_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init( GPIOA, &GPIO_InitStructure ); 
//	

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); 
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
//	
//	
//	/*SPI设置为双线双向全双工*/
//	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex;
//	/*SPI设置为主SPI*/
//	SPI_InitStructure.SPI_Mode 				= SPI_Mode_Master;
//	/*SPI数据宽度为8位*/
//	SPI_InitStructure.SPI_DataSize 			= SPI_DataSize_8b;
//	/*SPI同步时钟的空闲状态为低电平*/
//	SPI_InitStructure.SPI_CPOL 				= SPI_CPOL_Low;
//	/*SPI同步时钟的上升沿采集数据*/
//	SPI_InitStructure.SPI_CPHA 				= SPI_CPHA_1Edge;
//	/*SPI NSS信号由软件管理*/
//	SPI_InitStructure.SPI_NSS 				= SPI_NSS_Soft;
//	/*SPI波特率预分频值为64*/
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
//	/*SPI数据传输从MSB开始*/
//	SPI_InitStructure.SPI_FirstBit 			= SPI_FirstBit_MSB;
//	/*SPI CRC计算多项式*/
//	SPI_InitStructure.SPI_CRCPolynomial 	= 7;
//	SPI_Init( SPI1, &SPI_InitStructure );
//	
//	SPI_Cmd( SPI1, ENABLE );


//}
///* ---------------------------------------------------------------------------*/

///****
//	* @brief	SPI发送并接收一个字节
//    * @param  	ucSrc：待发送的字节数据
//	* @param	pSpiErr：SPI错误类型指针
//    * @retval 	SPI收到的字节
//    */
//uint8_t SPI_RW_1Byte(SPI_TypeDef* SPIx, uint8_t ucSrc, SpiErr_t* pSpiErr)
//{	
//	/*防止等待数据发送超时*/
//	SPI_TIME_COUNT = 0;
//	*pSpiErr = 0;
//	while( SPI_GetFlagStatus( SPIx, SPI_FLAG_TXE ) == RESET )
//	{
//		SPI_TIME_COUNT++;
//		
//		if( SPI_TIME_COUNT > SPI_TIME_OUT_THRESHOLD )
//		{
//			*pSpiErr = SPI_ERR_WaitingToSendTimeOut;
//			return 0;
//		}
//	}
//	
//	SPI_SendData( SPIx, ucSrc );
//	
//	SPI_TIME_COUNT = 0;
//	while( SPI_GetFlagStatus( SPIx, SPI_FLAG_RXNE ) == RESET )
//	{
//		SPI_TIME_COUNT++;
//		
//		if( SPI_TIME_COUNT > SPI_TIME_OUT_THRESHOLD )
//		{
//			*pSpiErr = SPI_ERR_ReadTimeOut;
//			return 0;
//		}
//	}
//	
//	return ( uint8_t ) SPI_ReceiveData( SPIx );
//	
//}
///* ---------------------------------------------------------------------------*/

///****
//	* @brief	SPI发送数据
//    * @param  	pbSrc：数据数组
//	* @param	size：数组大小（byte）
//    * @retval 	0：无误；非0：有误
//    */
//uint8_t SPI_Send( SPI_TypeDef* SPIx, uint8_t *pbSrc, uint8_t size )
//{
//	
//	SPI_TIME_COUNT = 0;
//	while( SPI_GetFlagStatus( SPIx, SPI_FLAG_BSY ) )
//	{
//		SPI_TIME_COUNT++;
//		if( SPI_TIME_COUNT > SPI_TIME_OUT_THRESHOLD )
//		{
//			return SPI_ERR_WaitingToSendTimeOut;
//		}
//	}
//	
//	while( size-- )
//	{
//		SPI_TIME_COUNT = 0;
//		while( SPI_GetFlagStatus( SPIx, SPI_FLAG_TXE ) == RESET )
//		{
//			SPI_TIME_COUNT++;
//			if( SPI_TIME_COUNT > SPI_TIME_OUT_THRESHOLD )
//			{
//				return SPI_ERR_SendDataTimeOut;
//			}
//		}
//		SPIx->DR = *pbSrc++;
//	}
//	
//	while( SPI_GetFlagStatus( SPIx, SPI_FLAG_BSY ) );
//	size = SPIx->DR;
//	
//	return 0;
//}
///* ---------------------------------------------------------------------------*/

///****
//	* @brief	SPI接收数据
//    * @param  	pbDst：数据数组
//	* @param	size：数组大小（byte）
//    * @retval 	0：无误；非0：有误
//    */
//uint8_t SPI_Receive( SPI_TypeDef* SPIx, uint8_t *pbDst, uint8_t size )
//{
//	while( size-- )
//	{
//		SPI_TIME_COUNT = 0;
//		while( SPI_GetFlagStatus( SPIx, SPI_FLAG_TXE ) == RESET )
//		{
//			SPI_TIME_COUNT++;
//			if( SPI_TIME_COUNT > SPI_TIME_OUT_THRESHOLD )
//			{
//				return SPI_ERR_SendDataTimeOut;
//			}
//		}
//		SPIx->DR = 0xFF;
//		
//		SPI_TIME_COUNT = 0;
//		while( SPI_GetFlagStatus( SPIx, SPI_FLAG_RXNE ) == RESET )
//		{
//			SPI_TIME_COUNT++;
//			if( SPI_TIME_COUNT > SPI_TIME_OUT_THRESHOLD )
//			{
//				return SPI_ERR_ReceiveTimeOut;
//			}
//		}
//		*pbDst++ = SPIx->DR;
//	}
//	
//	return 0;
//}
