/**
  ***********************************UTF-8**************************************
  * @file    nrf24l01p.h
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于实现NNRF+的收发，模块型号为EBYTE-ML01DP5
  ******************************************************************************  
  */ 
#ifndef _NNRFP_H
#define _NNRFP_H

#include "spi.h"

/** 配置和选项定义 */
#define nrfFIXED_PACK_LEN			32
#define nrfREPEAT_CNT				5


#define nrfRCC						( nrfIO_CSN_RCC|\
									  nrfIO_CE_RCC|\
									  nrfIO_IRQ_RCC )

#define nrfIO_CSN_RCC				RCC_AHB1Periph_GPIOA
#define nrfIO_CSN_GPIO				( ( GPIO_TypeDef* ) GPIOA )
#define nrfIO_CSN_Pin				( ( uint16_t ) GPIO_Pin_4 )

#define nrfIO_CE_RCC				RCC_AHB1Periph_GPIOC
#define nrfIO_CE_GPIO				( ( GPIO_TypeDef* ) GPIOC )
#define nrfIO_CE_Pin				( ( uint16_t ) GPIO_Pin_4 )

#define nrfIO_IRQ_RCC				RCC_AHB1Periph_GPIOB
#define nrfIO_IRQ_GPIO				( ( GPIO_TypeDef* ) GPIOB )
#define nrfIO_IRQ_Pin				( ( uint16_t ) GPIO_Pin_0 )
#define nrfEXTI_Line				( ( uint32_t ) EXTI_Line0 )


/** 口线操作函数定义 */

#define NRF_GET_IRQ_STATUS( )		PBin(0) ? 1 : 0	//IRQ状态

typedef enum ModeType
{
	MODE_TX = 0,
	MODE_RX
}nRf24l01ModeType;

typedef enum SpeedType
{
	SPEED_250K = 0,
	SPEED_1M,
	SPEED_2M
}nRf24l01SpeedType;

typedef enum PowerType
{
	POWER_F18DBM = 0,
	POWER_F12DBM,
	POWER_F6DBM,
	POWER_0DBM
}nRf24l01PowerType;


/** NNRF定义 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//寄存器操作命令
#define nrfCMD_READ_REG   		0x00	//读配置寄存器，低5位为寄存器地址
#define nrfCMD_WRITE_REG  		0x20	//写配置寄存器，低5位为寄存器地址
#define nrfCMD_R_RX_PLD    		0x61	//读RX有效数据，1~32字节
#define nrfCMD_W_TX_PLD    		0xA0	//写TX有效数据，1~32字节
#define nrfCMD_FLUSH_TX        	0xE1	//清除TX FIFO寄存器，发射模式下使用
#define nrfCMD_FLUSH_RX        	0xE2	//清除RX FIFO寄存器，接收模式下使用
#define nrfCMD_REUSE_TX_PL     	0xE3	//重新使用上一包数据，CE为高，数据包被不断发送
#define nrfCMD_R_RX_PLD_WID     0x60
#define nrfCMD_W_ACK_PLD		0xA8
#define nrfCMD_W_TX_PLD_NACK 	0xB0
//SPI(NNRF)寄存器地址
#define nrfREG_CONFIG			0x00	//配置寄存器地址，bit0:1接收模式,0发射模式
										//bit1:电选择;bit2:CRC模式;bit3:CRC使能
										//bit4:中断MAX_RT(达到最大重发次数中断)使能
										//bit5:中断TX_DS使能;bit6:中断RX_DR使能	
#define nrfREG_EN_AA			0x01	//使能自动应答功能 bit0~5 对应通道0~5
#define nrfREG_EN_RXADDR		0x02	//接收地址允许 bit0~5 对应通道0~5
#define nrfREG_SETUP_AW			0x03	//设置地址宽度(所有数据通道) 
										//bit0~1: 00,3字节 01,4字节, 02,5字节
#define nrfREG_SETUP_RETR		0x04	//建立自动重发;bit0~3:自动重发计数器
										//bit4~7:自动重发延时 250*x+86us
#define nrfREG_RF_CH			0x05	//RF通道,bit0~6工作通道频率
#define nrfREG_RF_SETUP			0x06	//RF寄存器，bit5|bit3:00,1Mbps 012Mbps, 10:250kbps
										//bit1~2:发射功率;bit0:噪声放大器增益
#define nrfREG_STATUS			0x07	//状态寄存器;bit0:TX FIFO满标志;bit1~3:接收数据通道号(最大:6);bit4:达到最多次重发次数
										//bit5:数据发送完成中断;bit6:接收数据中断
#define nrfMAX_TX  				0x10	//达到最大发送次数中断
#define nrfTX_OK   				0x20	//TX发送完成中断
#define nrfRX_OK   				0x40	//接收到数据中断

#define nrfREG_OBSERVE_TX		0x08	//发送检测寄存器,bit7~4:数据包丢失计数器
										//bit3~0:重发计数器
#define nrfREG_CD				0x09	//载波检测寄存器,bit0:载波检测
#define nrfREG_RX_ADDR_P0		0x0A	//数据通道0接收地址，最大长度5个字节，低字节在前
#define nrfREG_RX_ADDR_P1		0x0B	//数据通道1接收地址，最大长度5个字节，低字节在前
#define nrfREG_RX_ADDR_P2		0x0C	//数据通道2接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define nrfREG_RX_ADDR_P3		0x0D	//数据通道3接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define nrfREG_RX_ADDR_P4		0x0E	//数据通道4接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define nrfREG_RX_ADDR_P5		0x0F	//数据通道5接收地址，最低字节可设置，高字节，必须同RX_ADDR_P1[39:8]相等
#define nrfREG_TX_ADDR			0x10	//发送地址(低字节在前),ShockBurstTM模式下，RX_ADDR_P0与地址相等
#define nrfREG_RX_PW_P0			0x11	//接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define nrfREG_RX_PW_P1			0x12	//接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define nrfREG_RX_PW_P2			0x13	//接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define nrfREG_RX_PW_P3			0x14	//接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define nrfREG_RX_PW_P4			0x15	//接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define nrfREG_RX_PW_P5			0x16	//接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define nrfREG_FIFO_STATUS 		0x17	//FIFO状态寄存器;bit0:RX FIFO寄存器空标志;bit1:RX FIFO满标志;bit2~3保留
										//bit4:TX FIFO 空标志;bit5:TX FIFO满标志;bit6:1,循环发送上一数据包.0,不循环								
#define nrfREG_DYNPD			0x1C
#define nrfREG_FEATRUE			0x1D
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//位定义
#define nrfMASK_RX_DR   	6 
#define nrfMASK_TX_DS   	5 
#define nrfMASK_MAX_RT  	4 
#define nrfEN_CRC       	3 
#define nrfCRCO         	2 
#define nrfPWR_UP       	1 
#define nrfPRIM_RX      	0 

#define nrfENAA_P5      	5 
#define nrfENAA_P4      	4 
#define nrfENAA_P3      	3 
#define nrfENAA_P2      	2 
#define nrfENAA_P1      	1 
#define nrfENAA_P0      	0 

#define nrfnrfERX_P5       	5 
#define nrfERX_P4       	4 
#define nrfERX_P3       	3 
#define nrfERX_P2      		2 
#define nrfERX_P1       	1 
#define nrfERX_P0       	0 

#define nrfAW_RERSERVED 	0x0 
#define nrfAW_3BYTES    	0x1
#define nrfAW_4BYTES    	0x2
#define nrfAW_5BYTES    	0x3

#define nrfARD_250US    	(0x00<<4)
#define nrfARD_500US    	(0x01<<4)
#define nrfARD_750US    	(0x02<<4)
#define nrfARD_1000US   	(0x03<<4)
#define nrfARD_2000US   	(0x07<<4)
#define nrfARD_4000US   	(0x0F<<4)
#define nrfARC_DISABLE   	0x00
#define nrfARC_15        	0x0F

#define nrfCONT_WAVE     	7 
#define nrfRF_DR_LOW     	5 
#define nrfPLL_LOCK      	4 
#define nrfRF_DR_HIGH    	3 
//bit2-bit1:
#define nrfPWR_18DB  		(0x00<<1)
#define nrfPWR_12DB  		(0x01<<1)
#define nrfPWR_6DB   		(0x02<<1)
#define nrfPWR_0DB   		(0x03<<1)

#define nrfRX_DR         	6 
#define nrfTX_DS         	5 
#define nrfMAX_RT        	4 
//for bit3-bit1, 
#define nrfTX_FULL_0     	0 

#define nrfRPD           	0 

#define nrfTX_REUSE      	6 
#define nrfTX_FULL_1     	5 
#define nrfTX_EMPTY      	4 
//bit3-bit2, reserved, only '00'
#define nrfRX_FULL       	1 
#define nrfRX_EMPTY      	0 

#define nrfDPL_P5        	5 
#define nrfDPL_P4        	4 
#define nrfnrfDPL_P3        	3 
#define nrfDPL_P2        	2 
#define nrfDPL_P1        	1 
#define nrfDPL_P0        	0 

#define nrfEN_DPL        	2 
#define nrfEN_ACK_PAY    	1 
#define nrfEN_DYN_ACK    	0 
#define nrfIRQ_ALL  ( (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT) )



uint8_t NRF_Read_Reg( uint8_t RegAddr );
void NRF_Write_Reg( uint8_t RegAddr, uint8_t Value );
void NRF_Read_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len );
void NRF_Write_Buf( uint8_t RegAddr, uint8_t *pBuf, uint8_t len );

void NRF_Flush_Tx_Fifo ( void );
void NRF_Flush_Rx_Fifo( void );
void NRF_Reuse_Tx_Payload( void );
void NRF_Nop( void );
uint8_t NRF_Read_Status_Register( void );
uint8_t NRF_Clear_IRQ_Flag( uint8_t IRQ_Source );
uint8_t NRF_Read_IRQ_Status( void );
uint8_t NRF_Read_Top_Fifo_Width( void );
uint8_t NRF_Read_Rx_Payload( uint8_t *pRxBuf );
void NRF_Write_Tx_Payload_Ack( uint8_t *pTxBuf, uint8_t len );
void NRF_Write_Tx_Payload_NoAck( uint8_t *pTxBuf, uint8_t len );
void NRF_Write_Tx_Payload_InAck( uint8_t *pData, uint8_t len );
void NRF_Set_TxAddr( const uint8_t *pAddr, uint8_t len );
void NRF_Set_RxAddr( uint8_t PipeNum, const uint8_t *pAddr, uint8_t Len );
void NRF_Set_Speed( nRf24l01SpeedType Speed );
void NRF_Set_Power( nRf24l01PowerType Power );
void RF24LL01_Write_Hopping_Point( uint8_t FreqPoint );
void NRF_Set_Mode( nRf24l01ModeType Mode );
void  NRF_check( void );


uint8_t NRF_TxPacket( uint8_t *txbuf, uint8_t Length );
uint8_t NRF_RxPacket( uint8_t *rxbuf );
uint8_t NRF_SendData( uint8_t *pbSrc, uint16_t len );
void NRF_GPIO_Init( void );
void NRF_Init( void );	

#endif
