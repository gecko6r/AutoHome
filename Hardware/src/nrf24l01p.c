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
				  |      PB0      |      IRQ      |
				  |_______________|_______________|
  ----------------------------------------------------------------------------*/
  
#include "stdlib.h"
#include "string.h"


#include "nrf24l01p.h"
#include "usart.h"
#include "led.h"
#include "delay.h"
#include "spi.h"

const uint16_t FIXED_PACK_LEN = 32;
static const uint8_t TxAddr[ 5 ] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
static const uint8_t RxAddr[ 5 ] = { 0xd7, 0xd7, 0xd7, 0xd7, 0xd7 };
static const uint8_t RF_Channel = 100;	//2400 + 100 MHz

static volatile bool nrfIrqTriggered = false;

const char * const g_ErrorString = "NRF is not find !...";

static void NRF_SET_CS_LOW( void );
static void NRF_SET_CS_HIGH( void );
static void NRF_SET_CE_LOW( void );
static void NRF_SET_CE_HIGH( void );

/* ---------------------------------------------------------------------------*/	
static void NRF_SET_CS_LOW( void )
{
	GPIO_ResetBits( nrfIO_CSN_GPIO, nrfIO_CSN_Pin );
}
/* ---------------------------------------------------------------------------*/	
static void NRF_SET_CS_HIGH( void )
{
	GPIO_SetBits( nrfIO_CSN_GPIO, nrfIO_CSN_Pin );
}
/* ---------------------------------------------------------------------------*/	
static void NRF_SET_CE_LOW( void )
{
	GPIO_ResetBits( nrfIO_CE_GPIO, nrfIO_CE_Pin );
}
/* ---------------------------------------------------------------------------*/	
static void NRF_SET_CE_HIGH( void )
{
	GPIO_SetBits( nrfIO_CE_GPIO, nrfIO_CE_Pin );
}

	

/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化nrf24l01+ 相连引脚
	* @param  	None
	* @retval 	None
	*/
void NRF_GPIO_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd( nrfRCC, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
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

	SYSCFG_EXTILineConfig( EXTI_PortSourceGPIOB, EXTI_PinSource0);
    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = nrfEXTI_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

	
	
    /* Enable and set EXTI Interrupt to the highest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

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
		LED1 = ~LED1;
	
		EXTI_ClearITPendingBit( EXTI_Line0 );
	}

}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF读寄存器
  * @param :
           @Addr:寄存器地址
  * @note  :地址在设备中有效
  * @retval:读取的数据
  */
uint8_t NRF_Read_Reg( uint8_t RegAddr )
{
    uint8_t btmp;
	
    NRF_SET_CS_LOW( );			//片选
	
   SPI_ByteIO( nrfCMD_READ_REG | RegAddr );	//读命令 地址
    btmp =SPI_ByteIO( 0xFF );				//读数据
	
    NRF_SET_CS_HIGH( );			//取消片选
	
    return btmp;
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF读指定长度的数据
  * @param :
  *			@reg:地址
  *			@pBuf:数据存放地址
  *			@len:数据长度
  * @note  :数据长度不超过255，地址在设备中有效
  * @retval:读取状态
  */
void NRF_Read_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len )
{
    uint8_t i;
	
    NRF_SET_CS_LOW( );			//片选
	
   SPI_ByteIO( nrfCMD_READ_REG | RegAddr );	//读命令 地址
    for( i = 0; i < len; i ++ )
    {
        pBuf[ i ]  =SPI_ByteIO( 0xFF );	//读数据
    }
    NRF_SET_CS_HIGH( );		//取消片选
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF写寄存器
  * @param :无
  * @note  :地址在设备中有效
  * @retval:读写状态
  */
void NRF_Write_Reg( uint8_t RegAddr, uint8_t Value )
{
    NRF_SET_CS_LOW( );		//片选
	
   SPI_ByteIO( nrfCMD_WRITE_REG | RegAddr );	//写命令 地址
   SPI_ByteIO( Value );			//写数据
	
    NRF_SET_CS_HIGH( );		//取消片选
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF写指定长度的数据
  * @param :
  *			@reg:地址
  *			@pBuf:写入的数据地址
  *			@len:数据长度
  * @note  :数据长度不超过255，地址在设备中有效
  * @retval:写状态
  */
void NRF_Write_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len )
{
    uint8_t i;
	
    NRF_SET_CS_LOW( );		//片选
	

   SPI_ByteIO( nrfCMD_WRITE_REG | RegAddr );	//写命令 地址
    for( i = 0; i < len; i ++ )
    {
       SPI_ByteIO( pBuf[ i ] );		//写数据
    }
	
    NRF_SET_CS_HIGH( );		//取消片选
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :清空TX缓冲区
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Flush_Tx_Fifo ( void )
{
    NRF_SET_CS_LOW( );		//片选
	
   SPI_ByteIO( nrfCMD_FLUSH_TX );	//清TX FIFO命令
	
    NRF_SET_CS_HIGH( );		//取消片选
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :清空RX缓冲区
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Flush_Rx_Fifo( void )
{
    NRF_SET_CS_LOW( );		//片选
	
   SPI_ByteIO( nrfCMD_FLUSH_RX );	//清RX FIFO命令
	
    NRF_SET_CS_HIGH( );		//取消片选
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :重新使用上一包数据
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Reuse_Tx_Payload( void )
{
    NRF_SET_CS_LOW( );		//片选
	
   SPI_ByteIO( nrfCMD_REUSE_TX_PL );		//重新使用上一包命令
	
    NRF_SET_CS_HIGH( );		//取消片选
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF空操作
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Nop( void )
{
    NRF_SET_CS_LOW( );		//片选
	
   SPI_ByteIO( 0xFF );		//空操作命令
	
    NRF_SET_CS_HIGH( );		//取消片选
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF读状态寄存器
  * @param :无
  * @note  :无
  * @retval:NRF状态
  */
uint8_t NRF_Read_Status_Register( void )
{
    uint8_t Status;
	
    NRF_SET_CS_LOW( );		//片选
	
    Status =SPI_ByteIO( nrfCMD_READ_REG | nrfREG_STATUS );	//读状态寄存器
	
    NRF_SET_CS_HIGH( );		//取消片选
	
    return Status;
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF清中断
  * @param :
           @IRQ_Source:中断源
  * @note  :无
  * @retval:清除后状态寄存器的值
  */
uint8_t NRF_Clear_IRQ_Flag( uint8_t IRQ_Source )
{
    uint8_t btmp = 0;

    IRQ_Source &= ( 1 << nrfRX_DR ) | ( 1 << nrfTX_DS ) | ( 1 << nrfMAX_RT );	//中断标志处理
    btmp = NRF_Read_Status_Register( );			//读状态寄存器
			
    NRF_SET_CS_LOW( );			//片选
   SPI_ByteIO( nrfCMD_WRITE_REG | nrfREG_STATUS );	//写状态寄存器命令
   SPI_ByteIO( IRQ_Source | btmp );		//清相应中断标志
    NRF_SET_CS_HIGH( );			//取消片选
	
    return ( NRF_Read_Status_Register( ));			//返回状态寄存器状态
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :读NRF中断状态
  * @param :无
  * @note  :无
  * @retval:中断状态
  */
uint8_t NRF_Read_IRQ_Status( void )
{
    return ( NRF_Read_Status_Register( ) & (( 1 << nrfRX_DR ) | ( 1 << nrfTX_DS ) | ( 1 << nrfMAX_RT )));	//返回中断状态
}
 /* ---------------------------------------------------------------------------*/	

 /**
  * @brief :读FIFO中数据宽度
  * @param :无
  * @note  :无
  * @retval:数据宽度
  */
uint8_t NRF_Read_Top_Fifo_Width( void )
{
    uint8_t btmp;
	
    NRF_SET_CS_LOW( );		//片选
	
   SPI_ByteIO( nrfCMD_R_RX_PLD_WID );	//读FIFO中数据宽度命令
    btmp =SPI_ByteIO( 0xFF );	//读数据
	
    NRF_SET_CS_HIGH( );		//取消片选
	
    return btmp;
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :读接收到的数据
  * @param :无
  * @note  :无
  * @retval:
           @pRxBuf:数据存放地址首地址
  */
uint8_t NRF_Read_Rx_Payload( uint8_t *pRxBuf )
{
    uint8_t Width, PipeNum;
	
    PipeNum = ( NRF_Read_Reg( nrfREG_STATUS ) >> 1 ) & 0x07;	//读接收状态
    Width = NRF_Read_Top_Fifo_Width( );		//读接收数据个数

    NRF_SET_CS_LOW( );		//片选
	SPI_ByteIO( nrfCMD_R_RX_PLD );			//读有效数据命令
	
    for( PipeNum = 0; PipeNum < Width; PipeNum ++ )
    {
        *( pRxBuf + PipeNum ) =SPI_ByteIO( 0xFF );		//读数据
    }
    NRF_SET_CS_HIGH( );		//取消片选
    NRF_Flush_Rx_Fifo( );	//清空RX FIFO
	
    return Width;
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :发送数据（带应答）
  * @param :
  *			@pTxBuf:发送数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF_Write_Tx_Payload_Ack( uint8_t *pTxBuf, uint8_t len )
{
    uint8_t btmp;
    uint8_t length = ( len > 32 ) ? 32 : len;		//数据长达大约32 则只发送32个

    NRF_Flush_Tx_Fifo( );		//清TX FIFO
	
    NRF_SET_CS_LOW( );			//片选
   SPI_ByteIO( nrfCMD_W_TX_PLD );	//发送命令
	
    for( btmp = 0; btmp < length; btmp ++ )
    {
       SPI_ByteIO( *( pTxBuf + btmp ) );	//发送数据
    }
    NRF_SET_CS_HIGH( );			//取消片选
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :发送数据（不带应答）
  * @param :
  *			@pTxBuf:发送数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF_Write_Tx_Payload_NoAck( uint8_t *pTxBuf, uint8_t len )
{
	if( len > 32 || len == 0 )
	{
		return ;		//数据长度大于32 或者等于0 不执行
	}



	NRF_SET_CS_LOW( );	//片选


	SPI_ByteIO( nrfCMD_W_TX_PLD_NACK );	//发送命令
	while( len-- )
	{
		SPI_ByteIO( *pTxBuf++ );			//发送数据
	}
	NRF_SET_CS_HIGH( );		//取消片选
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :在接收模式下向TX FIFO写数据(带ACK)
  * @param :
  *			@pData:数据地址
  *			@len:长度
  * @note  :一次不超过32个字节
  * @retval:无
  */
void NRF_Write_Tx_Payload_InAck( uint8_t *pData, uint8_t len )
{
    uint8_t btmp;
	
	len = ( len > 32 ) ? 32 : len;		//数据长度大于32个则只写32个字节

    NRF_SET_CS_LOW( );			//片选
   SPI_ByteIO( nrfCMD_W_ACK_PLD );		//命令
    for( btmp = 0; btmp < len; btmp ++ )
    {
       SPI_ByteIO( *( pData + btmp ) );	//写数据
    }
    NRF_SET_CS_HIGH( );			//取消片选
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :设置发送地址
  * @param :
  *			@pAddr:地址存放地址
  *			@len:长度
  * @note  :无
  * @retval:无
  */
void NRF_Set_TxAddr( const uint8_t *pAddr, uint8_t len )
{
	len = ( len > 5 ) ? 5 : len;					//地址不能大于5个字节
    NRF_Write_Buf( nrfREG_TX_ADDR, ( uint8_t * )pAddr, len );	//写地址
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :设置接收通道地址
  * @param :
  *			@PipeNum:通道
  *			@pAddr:地址存肥着地址
  *			@Len:长度
  * @note  :通道不大于5 地址长度不大于5个字节
  * @retval:无
  */
void NRF_Set_RxAddr( uint8_t PipeNum, const uint8_t *pAddr, uint8_t Len )
{
    Len = ( Len > 5 ) ? 5 : Len;
    PipeNum = ( PipeNum > 5 ) ? 5 : PipeNum;		//通道不大于5 地址长度不大于5个字节

    NRF_Write_Buf( nrfREG_RX_ADDR_P0 + PipeNum, ( uint8_t *) pAddr, Len );	//写入地址
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :设置通信速度
  * @param :
  *			@Speed:速度
  * @note  :无
  * @retval:无
  */
void NRF_Set_Speed( nRf24l01SpeedType Speed )
{
	uint8_t btmp = 0;
	
	btmp = NRF_Read_Reg( nrfREG_RF_SETUP );
	btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	
	if( Speed == SPEED_250K )		//250K
	{
		btmp |= ( 1<<5 );
	}
	else if( Speed == SPEED_1M )   //1M
	{
   		btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	}
	else if( Speed == SPEED_2M )   //2M
	{
		btmp |= ( 1<<3 );
	}

	NRF_Write_Reg( nrfREG_RF_SETUP, btmp );
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :设置功率
  * @param :
  *			@Speed:速度
  * @note  :无
  * @retval:无
  */
void NRF_Set_Power( nRf24l01PowerType Power )
{
    uint8_t btmp;
	
	btmp = NRF_Read_Reg( nrfREG_RF_SETUP ) & ~0x07;
    switch( Power )
    {
        case POWER_F18DBM:
            btmp |= nrfPWR_18DB;
            break;
        case POWER_F12DBM:
            btmp |= nrfPWR_12DB;
            break;
        case POWER_F6DBM:
            btmp |= nrfPWR_6DB;
            break;
        case POWER_0DBM:
            btmp |= nrfPWR_0DB;
            break;
        default:
            break;
    }
    NRF_Write_Reg( nrfREG_RF_SETUP, btmp );
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :设置频率
  * @param :
  *			@FreqPoint:频率设置参数
  * @note  :值不大于127
  * @retval:无
  */
void RF24LL01_Write_Hopping_Point( uint8_t FreqPoint )
{
    NRF_Write_Reg(  nrfREG_RF_CH, FreqPoint & 0x7F );
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF检测
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void NRF_check( void )
{
	uint8_t i;
	uint8_t buf[5]={ 0XA5, 0XA5, 0XA5, 0XA5, 0XA5 };
	uint8_t read_buf[ 5 ] = { 0 };
	 
	while( 1 )
	{
		NRF_Write_Buf( nrfREG_TX_ADDR, buf, 5 );			//写入5个字节的地址
		NRF_Read_Buf( nrfREG_TX_ADDR, read_buf, 5 );		//读出写入的地址  
		for( i = 0; i < 5; i++ )
		{
			if( buf[ i ] != read_buf[ i ] )
			{
				break;
			}	
		} 
		
		if( 5 == i )
		{
			printf("Nrf24L01 online\r\n");
			break;
		}
		else
		{
			printf(" ERROR: No Nrf24l01p connected\r\n");
		}
		delay_ms(1500);
	}
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :设置模式
  * @param :
  *			@Mode:模式发送模式或接收模式
  * @note  :无
  * @retval:无
  */
void NRF_Set_Mode( nRf24l01ModeType Mode )
{
    uint8_t controlreg = 0;
	controlreg = NRF_Read_Reg( nrfREG_CONFIG );
	
    if( Mode == MODE_TX )       
	{
		controlreg &= ~( 1<< nrfPRIM_RX );
	}
    else 
	{
		if( Mode == MODE_RX )  
		{ 
			controlreg |= ( 1<< nrfPRIM_RX ); 
		}
	}

    NRF_Write_Reg( nrfREG_CONFIG, controlreg );
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF发送一次数据
  * @param :
  *			@txbuf:待发送数据首地址
  *			@Length:发送数据长度
  * @note  :无
  * @retval:
  *			MAX_TX：达到最大重发次数
  *			TX_OK：发送完成
  *			0xFF:其他原因
  */ 
uint8_t NRF_TxPacket( uint8_t *txbuf, uint8_t Length )
{
	uint8_t l_Status = 0;
	uint16_t l_MsTimes = 0;
	
	NRF_SET_CS_LOW( );		//片选
	SPI_ByteIO( nrfCMD_FLUSH_TX );
	NRF_SET_CS_HIGH( );
	
	NRF_SET_CE_LOW( );		
	NRF_Write_Buf( nrfCMD_W_TX_PLD_NACK, txbuf, Length );	//写数据到TX BUF 32字节  TX_PLOAD_WIDTH
	NRF_SET_CE_HIGH( );			//启动发送
	while( 0 != NRF_GET_IRQ_STATUS( ))
	{
		delay_ms( 1 );
		if( 500 == l_MsTimes++ )						//500ms还没有发送成功，重新初始化设备
		{
			NRF_GPIO_Init( );
			NRF_Init( );
			NRF_Set_Mode( MODE_TX );
			break;
		}
	}
	l_Status = NRF_Read_Reg( nrfREG_STATUS );						//读状态寄存器
	NRF_Write_Reg( nrfREG_STATUS, l_Status );						//清除TX_DS或MAX_RT中断标志
	
	if( l_Status & nrfMAX_TX )	//达到最大重发次数
	{
		NRF_Write_Reg( nrfCMD_FLUSH_TX, 0xff );	//清除TX FIFO寄存器
		return nrfMAX_TX; 
	}
	if( l_Status & nrfTX_OK )	//发送完成
	{
		return nrfTX_OK;
	}
	
	return 0xFF;	//其他原因发送失败
}
/* ---------------------------------------------------------------------------*/	

/**
  * @brief :NRF接收数据
  * @param :
  *			@rxbuf:接收数据存放地址
  * @note  :无
  * @retval:接收的数据个数
  */ 
uint8_t NRF_RxPacket( uint8_t *rxbuf )
{
	uint8_t l_Status = 0, l_RxLength = 0, l_100MsTimes = 0;
	
	NRF_SET_CS_LOW( );		//片选
	SPI_ByteIO( nrfCMD_FLUSH_RX );
	NRF_SET_CS_HIGH( );
	
	while( 0 != NRF_GET_IRQ_STATUS( ))
	{
		delay_ms( 100 );
		
		if( 30 == l_100MsTimes++ )		//3s没接收过数据，重新初始化模块
		{
			NRF_GPIO_Init( );
			NRF_Init( );
			NRF_Set_Mode( MODE_RX );
			break;
		}
	}
	
	l_Status = NRF_Read_Reg( nrfREG_STATUS );		//读状态寄存器
	NRF_Write_Reg( nrfREG_STATUS,l_Status );		//清中断标志
	if( l_Status & nrfRX_OK)	//接收到数据
	{
		l_RxLength = NRF_Read_Reg( nrfCMD_R_RX_PLD_WID );		//读取接收到的数据个数
		NRF_Read_Buf( nrfCMD_R_RX_PLD,rxbuf,l_RxLength );	//接收到数据 
		NRF_Write_Reg( nrfCMD_FLUSH_RX, 0xff );				//清除RX FIFO
		return l_RxLength;
	}	
	
	return 0;				//没有收到数据	
}
/* ---------------------------------------------------------------------------*/	

 /**
  * @brief :NRF模块初始化
  * @param :
			@pbSrc: 数据数组
			@len：	数组长度
  * @retval:
			0:数据无误
			1：数据长度无效
			2：申请栈空间失败
			3：发送失败
  */
uint8_t NRF_SendData( uint8_t *pbSrc, uint16_t len )
{
	uint8_t *pbuf = NULL;
	uint8_t size = 0;
	uint8_t *p = pbSrc;
	uint8_t res = 0;
	
	if( !len ) return 1;
	
	pbuf = malloc( nrfFIXED_PACK_LEN );
	
	if( pbuf == NULL )
	{
		return 2 ;
	}
	
	while( len )
	{
		//第一个字节作为包头，表示此次发送的数据量，实际可用数据为最大数据量-1
		size = ( len > ( FIXED_PACK_LEN - 1 ) ) ? ( FIXED_PACK_LEN - 1 ) : len;
		
		memcpy( pbuf+1, p, size );
		
		pbuf[ 0 ] = size;		
		res = NRF_TxPacket( pbuf, FIXED_PACK_LEN );
		if( res != nrfTX_OK )
		{
			free( pbuf );
			return 3;
		}
		
		p += size;
		memset( pbuf, 0, 32 );
		len -= size;
	}
		
	free( pbuf );
	
	
	return 0;
	
}
/* ---------------------------------------------------------------------------*/	
 /**
  * @brief :NRF模块初始化
  * @param :无
  * @note  :无
  * @retval:无
  */
void NRF_Init( void )
{
	Nrf_IRQ_Init();
    NRF_SET_CE_HIGH( );

    NRF_Write_Reg( nrfREG_EN_AA, ( 1 << nrfENAA_P0 ) );   		//通道0自动应答
    NRF_Write_Reg( nrfREG_EN_RXADDR, ( 1 << nrfERX_P0 ) );		//通道0接收
    NRF_Write_Reg( nrfREG_SETUP_AW, nrfAW_5BYTES );     		//地址宽度 5个字节
    NRF_Write_Reg( nrfREG_RF_CH, RF_Channel );             			//初始化通道

	NRF_Write_Reg( nrfREG_CONFIG, 0x0e );
	NRF_Write_Reg( nrfREG_SETUP_RETR, 0x00 );
	NRF_Write_Reg( nrfREG_DYNPD, 0x01 );
	NRF_Write_Reg( nrfREG_FEATRUE, 0x07 );

    NRF_Set_TxAddr( (uint8_t *)TxAddr, 5 );                      //设置TX地址
    NRF_Set_RxAddr( 0, (uint8_t *)RxAddr, 5 );                   //设置RX地址
}



