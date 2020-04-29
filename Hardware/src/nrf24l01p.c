/**
  ***********************************UTF-8**************************************
  * @file    nrf24l01p.c
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于实现Nrf24L01+的收发，模块型号为EBYTE-ML01DP5
  ******************************************************************************  
  */ 
  
  /*-------------STM32F407VET6 与 EBYTE-ML01DP5的接线---------------------------
				   _______________________________
				  |               |               |
				  | STM32F407VET6 | EBYTE-ML01DP5 |
				  |_______________|_______________|
				  |      PA4      |      CSN      |
				  |---------------|---------------|
				  |      PA5      |      SCK      |
				  |---------------|---------------|
				  |      PA6      |      MISO     |
				  |---------------|---------------|
				  |      PA7      |      MOSI     |
				  |---------------|---------------|
				  |      PC4      |      CE       |
				  |---------------|---------------|
				  |      PC5      |      IRQ      |
				  |_______________|_______________|
  ----------------------------------------------------------------------------*/
  
#include "nrf24l01p.h"
#include "usart.h"

/*设定nrf24l01+模块的发送目标地址*/
const uint8_t NRF_TX_Addr[5] = {0x00, 0x20, 0x16, 0x09, 0x14};
/*设定nrf24l01+模块的本机接收地址*/
const uint8_t NRF_RX_Addr[5] = {0x00, 0x19, 0x95, 0x08, 0x18};
/*设定nrf24l01+模块的收/发频道的中心频率为2484MHz*/
const uint8_t RF_Channel = 44;

static uint8_t nrfIT_Triggered = RESET;

/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化nrf24l01+ 相连引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_GPIO_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd( nrfRCC, ENABLE );
	
	/* 配置nrf24l01+模块CSN引脚 */
	GPIO_InitStructure.GPIO_Pin 	= nrfIO_CSN_Pin;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init( nrfIO_CSN_GPIO, &GPIO_InitStructure );
	
	/* 配置nrf24l01+模块CE引脚 */
	GPIO_InitStructure.GPIO_Pin 	= nrfIO_CE_Pin;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init( nrfIO_CE_GPIO, &GPIO_InitStructure );
	
	/* 配置nrf24l01+模块IRQ引脚 */
	GPIO_InitStructure.GPIO_Pin 	= nrfIO_IRQ_Pin;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_Init( nrfIO_IRQ_GPIO, &GPIO_InitStructure );
	
	GPIO_SetBits( nrfIO_IRQ_GPIO, nrfIO_IRQ_Pin );
	GPIO_SetBits( GPIOC, GPIO_Pin_5 );
	GPIO_SetBits( nrfIO_CSN_GPIO, nrfIO_CSN_Pin );
	GPIO_ResetBits( nrfIO_CE_GPIO, nrfIO_CE_Pin );
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化nrf24l01+ 外部中断
	* @param  	None
	* @retval 	None
	*/
static void Nrf_IRQ_Init( void )
{
	EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = nrfEXTI_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    /* Enable and set EXTI Interrupt to the highest priority */
    NVIC_InitStructure.NVIC_IRQChannel = nrfEXTI_Line;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );
    EXTI_ClearITPendingBit( nrfEXTI_Line );

}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 外部中断服务函数
	* @param  	None
	* @retval 	None
	*/
void EXTI0_IRQHandler( void )
{
	if( EXTI_GetITStatus( EXTI_Line0 ) != RESET )
	{
		printf( "reveived\r\n" );
		
		EXTI_ClearITPendingBit( EXTI_Line0 );
	}
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	使能nrf24l01+ CE引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_SetCE_High( void )
{
	GPIO_SetBits( nrfIO_CE_GPIO, nrfIO_CE_Pin );
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	失能nrf24l01+ CE引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_SetCE_Low( void )
{
	GPIO_ResetBits( nrfIO_CE_GPIO, nrfIO_CE_Pin );
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	使能nrf24l01+ CSN引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_SetCSN_High( void )
{
	GPIO_SetBits( nrfIO_CSN_GPIO, nrfIO_CSN_Pin );
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	失能nrf24l01+ CSN引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_SetCSN_Low( void )
{
	GPIO_ResetBits( nrfIO_CSN_GPIO, nrfIO_CSN_Pin );
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 单个寄存器写入,如果写入过程无误，返回值最高为应为0，
				否则为1
	* @param  	ucRegAddr：寄存器地址
	* @param  	ucSrc：待写入的值
	* @param  	pErr：存储错误信息的指针
	* @retval 	nrf24l01+ 状态寄存器值
	*/
NrfStatus_t Nrf_RegSingleWrite( uint8_t ucRegAddr, uint8_t ucSrc, 
									SpiErr_t* pErr )
{
	NrfStatus_t xNrfStatus = 0;
	
	Nrf_SetCE_Low(  );
	Nrf_SetCSN_Low(  );
	
	/*发送写寄存器操作和寄存器地址,nrf24l01+ status值*/
	xNrfStatus = SPI_RW_1Byte( nrfSPI, nrfCMD_W_REG|ucRegAddr, pErr );	
	
	/*发送新的寄存器值*/
	SPI_RW_1Byte( nrfSPI, ucSrc, pErr );	
	if( *pErr != SPI_ERR_NoError ) return xNrfStatus|( 1<<7 );

	
	Nrf_SetCSN_High(  );
	Nrf_SetCE_High(  );
	
	return xNrfStatus;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 单个寄存器读取,如果读取过程无误，返回值最高为应为0，
				否则为1
	* @param  	ucRegAddr：寄存器地址
	* @param  	ucSrc：待写入的值
	* @param  	pErr：存储错误信息的指针
	* @retval 	nrf24l01+ 状态寄存器值
	*/
NrfStatus_t Nrf_RegSingleRead( uint8_t ucRegAddr, uint8_t* pucDst, 
									SpiErr_t* pErr )
{
	NrfStatus_t xNrfStatus = 0;
	
	Nrf_SetCSN_Low(  );
	
	/*发送读寄存器操作和寄存器地址，并读status*/
	xNrfStatus = SPI_RW_1Byte( nrfSPI, nrfCMD_R_REG|ucRegAddr, pErr );	
	/*读取寄存器值*/
	*pucDst = SPI_RW_1Byte( nrfSPI, 0x00, pErr );

	if( *pErr != SPI_ERR_NoError ) return xNrfStatus|( 1<<7 );

	Nrf_SetCSN_High(  );
	
	return xNrfStatus;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 多个寄存器写入，如果写入过程无误，返回值最高为应为0，
				否则为1
	* @param  	ucRegAddr：起始寄存器地址
	* @param  	ucSrcBuf：待写入的值数组
	* @param  	ucLen：数组长度
	* @param  	pErr：存储错误信息的指针
	* @retval 	nrf24l01+ 状态寄存器值|( errBit<<7 )，若有错误errBit为1， 
				无错误为0
	*/
NrfStatus_t Nrf_RegMultiWrite( uint8_t ucRegAddr, uint8_t* ucSrcBuf, 
									uint8_t ucLen, SpiErr_t* pErr )	
{
	NrfStatus_t xNrfStatus = 0;
	
	Nrf_SetCE_Low(  );
	Nrf_SetCSN_Low(  );
	
	/*发送寄存器写操作和起始寄存器地址*/
	SPI_RW_1Byte( nrfSPI, nrfCMD_W_REG|ucRegAddr, pErr );
	if( *pErr != SPI_ERR_NoError ) return xNrfStatus|( 1<<7 );
	
	/*读状态寄存器值*/
	xNrfStatus = SPI_RW_1Byte( nrfSPI, 0x00, pErr );
	if( *pErr != SPI_ERR_NoError ) return xNrfStatus|( 1<<7 );
	
	/*发送寄存器值*/
	while( ucLen-- )
	{
		SPI_RW_1Byte( nrfSPI, *( ucSrcBuf++ ), pErr );
		if( *pErr != SPI_ERR_NoError ) return xNrfStatus|( 1<<7 );
	}
	
	Nrf_SetCSN_High(  );
	Nrf_SetCE_High(  );
	
	return xNrfStatus;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 多个寄存器读取，如果读取过程无误，返回值最高为应为0，
				否则为1
	* @param  	ucRegAddr：起始寄存器地址
	* @param  	ucDstBuf：寄存器值存放数组
	* @param  	ucLen：数组长度
	* @param  	pErr：存储错误信息的指针
	* @retval 	nrf24l01+ 状态寄存器值|( errBit<<7 )，若有错误errBit为1， 
				无错误为0
	*/
NrfStatus_t Nrf_RegMultiRead( uint8_t ucRegAddr, uint8_t* ucDstBuf, 
									uint8_t ucLen, SpiErr_t* pErr )
{
	NrfStatus_t xNrfStatus = 0;
	
	Nrf_SetCSN_Low(  );
	
	/*发送寄存器读操作和起始寄存器地址，并读状态寄存器值*/
	xNrfStatus = SPI_RW_1Byte( nrfSPI, nrfCMD_R_REG|ucRegAddr, pErr );
	if( *pErr != SPI_ERR_NoError ) return xNrfStatus|( 1<<7 );
	
	/*接收寄存器值*/
	while( ucLen-- )
	{
		*( ucDstBuf++ ) = SPI_RW_1Byte( nrfSPI, 0x00, pErr );
		if( *pErr != SPI_ERR_NoError ) return xNrfStatus|( 1<<7 );
	}
	
	Nrf_SetCSN_High(  );
	
	return xNrfStatus;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化nrf24l01+ 的相关引脚和外部中断，以及SPI1
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_Init( void )
{
	uint8_t ucConfig = 0;
	SpiErr_t xErr = SPI_ERR_NoError;
	NrfStatus_t xStatus = 0;
	/*初始化nrf24l01+相连引脚*/
	Nrf_GPIO_Init();
	/*初始化nrf24l01+ 外部中断*/
	Nrf_IRQ_Init();
	/*初始化nrf24l01+ SPI*/
	SPI1_Init();
	
//	ucConfig =   NrfInitStruct->NRF_Mask_RX_DR			|\
//				 NrfInitStruct->NRF_Mask_TX_DS			|\
//				 NrfInitStruct->NRF_Mask_MAX_RT			|\
//				 NrfInitStruct->NRF_EN_CRC				|\
//				 NrfInitStruct->NRF_CRC_Coding_Bytes	|\
//				 NrfInitStruct->NRF_PWR_Manage			|\
//				 NrfInitStruct->NRF_PRIM_RX;
	
	xStatus = Nrf_RegSingleWrite( nrfREG_CONFIG, ucConfig, &xErr );
	
	return xStatus;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_TxData( uint8_t* ucSrcBuf, uint8_t ucLen )
{
	NrfStatus_t xStatus;
	SpiErr_t xErr;
	uint16_t count = 0;
	
	Nrf_SetCSN_Low();
	SPI_RW_1Byte(nrfSPI, nrfCMD_FLUSH_TX, &xErr);
	Nrf_SetCSN_High();
	
	Nrf_SetCE_Low();
	Nrf_RegMultiWrite( nrfCMD_W_TX_PAYLOAD, ucSrcBuf, ucLen, &xErr);
	Nrf_SetCE_High();
	
	while( Nrf_IT_Triggered() == RESET )
	{
		if( count++ > SPI_TIME_OUT_THRESHOLD )
		{
			
			return 0xff;
		}
	}
	
	xStatus = SPI_RW_1Byte( nrfSPI, 0x00, &xErr );
	Nrf_RegSingleWrite( nrfREG_STATUS, xStatus, &xErr);
	
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_RxData( uint8_t* usDstBuf, uint8_t ucLen )
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetAutoAckState( NRF_Data_Pipe_t xPipe, FunctionalState fState )
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetRxAddrState( NRF_Data_Pipe_t xPipe, FunctionalState fState )
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetPowerState( FunctionalState fState )
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetMode( NRF_Mode_t xTxRxMode )
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetAutoRetr( uint8_t RETR_Status )
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetRF_Chn( uint8_t RF_Channel, uint8_t RF_param )
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetADDR_Width( NRF_ADDR_Width_t xWidth )
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetRxADDR( const uint8_t* rxAddrBuf, uint8_t addrWidth)
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatus_t Nrf_SetTxADDR( const uint8_t* txAddrBuf, uint8_t addrWidth)
{
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	发送数据
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
uint8_t Nrf_IT_Triggered(void)
{
	
}
