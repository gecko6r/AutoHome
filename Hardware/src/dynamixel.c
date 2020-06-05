/**
  ***********************************UTF-8**************************************
  * @file    dunamixel.c
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   This file contains all the functions prototypes for the Dynamixel 
  *          servo control
  ******************************************************************************  
  */ 
  /*-----------------------Dynamixel 舵机通信协议2.0，发送数据包格式-------------------------------------
   ______________________________________________________________________________________________________
  |                                                                     |                   |           |
  |                                  8                                  |     para_len      |     2     |
  |_____________________________________________________________________|___________________|___________|
  |                                                                     |                   |           |  
  |Header1|Header2|Header3|Reserved|PacketID|Length1|Length2|Instruction|Param |Param|Param |CRC1 |CRC2 |
  |_____________________________________________________________________|___________________|___________|
  |                                                                     |                   |           |
  | 0xFF  | 0xFF  | 0xFD  |  0x00  |   ID   | Len_L | Len_H |Instrucion |Param1|...  |ParamN|CRC_L|CRC_H|
  |_____________________________________________________________________|___________________|___________|
  -----------------------------------------------------------------------------------------------------*/
  

 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "controller.h"
#include "dynamixel.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "dma.h"



/*--------------------------------全局变量----------------------------------- */
//舵机发送缓冲区
uint8_t g_ucaServoTxBuffer[HALF_BUF_SIZE];

//舵机接收缓冲区
uint8_t g_ucaServoRxBuffer[HALF_BUF_SIZE];

//舵机状态数组
double dServoStatusBuf[ dxlStatusCount ][ ctrlSERVO_NUM ];

//舵机消息控制
ServoMsg xServoMsg;

uint8_t g_ucaDataRecved[ ctrlSERVO_NUM ];

/*--------------------------------局部变量----------------------------------- */
//存储舵机ID
static uint8_t ucDxlIdBuf[ctrlSERVO_NUM] = {0U};

//存储发送给舵机的参数
static uint32_t uxDxlParamBuf[ctrlSERVO_NUM] = {0U};

//存储舵机位置
static uint32_t s_ulaServoPosBuf[ ctrlSERVO_NUM ];

//存储舵机电流（力矩）
static uint16_t s_usaServoCurrentBuf[ ctrlSERVO_NUM ];

//存储舵机速度
static uint32_t s_ulaServoVelBuf[ ctrlSERVO_NUM ];

//存储舵机电压
static uint16_t s_usServoVolt;

//在线舵机数量
static uint8_t s_ucOnlineServoCount = ctrlSERVO_NUM;

//舵机初始化完毕？
static bool s_bServoInitFinished = false;



/*-----------------------------------常量-------------------------------------*/
//DXL servo supported baudrates
static const uint32_t ulDxlBaudrateBuf[8] = { 9600, 57600, 115200,
											  1000000, 2000000, 3000000,
											  4000000, 4500000};			

static const char* cServoStatusMsgBuf[ dxlStatusCount ] = {		
								"Present position: ",
								"Present voltage:  ",
								"Present current:  ",
								"Present velocity: ",
								"Present tick:     ", };
/*********************************变量定义结束*********************************/
									   
/* Dynamixel 相关函数定义-----------------------------------------------------*/		
											   
/****
	* @brief	初始化舵机通讯数组、串口、DMA，并使能舵机力矩
	* @param  	ulBaudrate: 对应串口的波特率
	* @param  	bTorqueState: 舵机力矩使能
	* @retval 	返回在线舵机数目，或-1系统错误
	*/
int DXL_ServoInit(EDxlBaudrate eBaudrate, FunctionalState fTorqueState)
{
	uint8_t i = 0;
	int ret;
	
	//初始化舵机id数组
	for ( i = 0; i < ctrlSERVO_NUM; i++ )
		ucDxlIdBuf[ i ] = 0;
	
	xServoMsg = xCreateMsg( g_ucaServoRxBuffer );
	
	//初始化舵机通讯数组包头
	g_ucaServoTxBuffer[ eBYTE_HEADER_1 ]	= 0xFF;
	g_ucaServoTxBuffer[ eBYTE_HEADER_2 ]	= 0xFF;
	g_ucaServoTxBuffer[ eBYTE_HRADER_3 ]	= 0xFD;
	g_ucaServoTxBuffer[ eBYTE_RESV ]		= 0x00;
	
	//初始化电平转换芯片方向控制引脚
	
	//初始化串口及dma
	USART1_Init( ulDxlBaudrateBuf[ eBaudrate ] );
	DMA_ServoTxInit( dmaServoTxPeriphAddr, ( uint32_t )g_ucaServoTxBuffer, MAX_BUF_SIZE );

	delay_ms( 200 );
	ret = DXL_Ping();
	if( ret > 0 )
	{
		printf("%d servos online\r\n", s_ucOnlineServoCount );
		DXL_SetAllTorque( fTorqueState );
	}
	else
	{
		printf("Error, failed to initiate servos, please check the power supply OR cable connection!\r\n" );
	}
		
	return ret;
		
	
	

}
/* ---------------------------------------------------------------------------*/

/****
    * @brief	写入单个舵机的寄存器(可以是多个连续的寄存器)
    * @param  	usRegAddr：寄存器地址
    * @param  	usRegSize：寄存器大小(Byte)
    * @param  	ulData：待写入的数据
    * @param  	ucId：舵机id
    * @retval 	None
    */
void DXL_RegWrite(uint16_t usRegAddr, uint16_t usRegSize, 
					uint32_t ulData, uint8_t ucId)
{
	u8 i;
	/* 参数长度：数据长度（Para1~ParaN） + 附加长度（16位寄存器地址）*/
	uint16_t usParamLen = PARAM_ADD_LEN + usRegSize;
	/* 包体长度：参数长度 + 附加长度（8位指令 + 16位校验码）*/
	uint16_t usPacketLen = PACKET_ADD_LEN + usParamLen;
	/* 数据包总长：包体长度 + 剩余长度*/
	uint16_t usPacketSize = usPacketLen + PACKET_LEFT_LEN;
	
	uint16_t usDxlCrcCheck;

	g_ucaServoTxBuffer[eBYTE_ID]			= ucId;
	g_ucaServoTxBuffer[eBYTE_LEN_L]			= LOW_BYTE(usPacketLen);
	g_ucaServoTxBuffer[eBYTE_LEN_H]			= HIGH_BYTE(usPacketLen);
	g_ucaServoTxBuffer[eBYTE_INST]			= dxlINST_Write;
	g_ucaServoTxBuffer[BYTE_ParamN(1)]		= LOW_BYTE(usRegAddr);
	g_ucaServoTxBuffer[BYTE_ParamN(2)]		= HIGH_BYTE(usRegAddr);

	
	for(i=0; i<usRegSize; i++) g_ucaServoTxBuffer[10+i] = nthByteOf(ulData, i);
	
	
	usDxlCrcCheck = CrcCheck(g_ucaServoTxBuffer, usPacketSize - 2);
	g_ucaServoTxBuffer[usPacketSize - 2] = LOW_BYTE(usDxlCrcCheck);
	g_ucaServoTxBuffer[usPacketSize - 1] = HIGH_BYTE(usDxlCrcCheck);

/*	
//////////////////////////////////////////////////////////////////////////////
#ifdef IN_DEBUG_MODE
	
	printf("in file: %s, line: %d\r\n", __FILE__, __LINE__);
	DXL_PrintBuf(g_ucaServoTxBuffer, usPacketSize, false);
	
#endif
//////////////////////////////////////////////////////////////////////////////
*/
	
	DMA_SendData( dmaServoTxStream, usPacketSize );
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	同步写入多个舵机的寄存器(可以是多个连续的寄存器)
    * @param  	usRegAddr：寄存器地址
    * @param  	usRegSize：寄存器大小(Byte)
    * @param  	ulData：待写入的数据数组
    * @param  	ucId：舵机id数组
    * @retval 	None
    */
void DXL_RegSyncWrite(uint16_t usRegAddr, uint16_t usRegSize, 
	uint8_t ucServoNum, uint32_t* ulDataBuf, uint8_t* ucIdBuf)
{
	
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t idCount = 0;
	
	/* 参数长度：数据长度（Para1~ParaN） + 附加长度（寄存器地址，寄存器长度）*/
	uint16_t usParamLen = 2+PARAM_ADD_LEN + ucServoNum * ( usRegSize + 1 );
	/* 包体长度：参数长度 + 附加长度（8位指令 + 16位校验码）*/
	uint16_t usPacketLen = PACKET_ADD_LEN + usParamLen;
	/* 数据包总长：包体长度 + 剩余长度*/
	uint16_t usPacketSize = usPacketLen + PACKET_LEFT_LEN;
	
	uint16_t usDxlCrcCheck = 0U;
	
	g_ucaServoTxBuffer[eBYTE_ID]			= ID_BROADCAST;
	g_ucaServoTxBuffer[eBYTE_LEN_L]			= LOW_BYTE(usPacketLen);
	g_ucaServoTxBuffer[eBYTE_LEN_H]			= HIGH_BYTE(usPacketLen);
	g_ucaServoTxBuffer[eBYTE_INST]			= dxlINST_Sync_Write;
	g_ucaServoTxBuffer[BYTE_ParamN(1)]		= LOW_BYTE(usRegAddr);
	g_ucaServoTxBuffer[BYTE_ParamN(2)]		= HIGH_BYTE(usRegAddr);
	g_ucaServoTxBuffer[BYTE_ParamN(3)]		= LOW_BYTE(usRegSize);
	g_ucaServoTxBuffer[BYTE_ParamN(4)]		= HIGH_BYTE(usRegSize);
	
	for(i=0; i< ctrlSERVO_NUM; i++) 
	{
		if( ucDxlIdBuf[ i ] )
		{
			g_ucaServoTxBuffer[SYNC_OPT_START_ADDR + idCount*(usRegSize+1)] = ucIdBuf[i];

			for(j=0; j<usRegSize; j++)
				g_ucaServoTxBuffer[SYNC_OPT_START_ADDR+idCount*(usRegSize+1)+1 + j] 
													= nthByteOf(ulDataBuf[idCount], j);
			idCount++;
		}
	}
	
	usDxlCrcCheck = CrcCheck(g_ucaServoTxBuffer, usPacketSize - 2);
	g_ucaServoTxBuffer[usPacketSize - 2] = LOW_BYTE(usDxlCrcCheck);
	g_ucaServoTxBuffer[usPacketSize - 1] = HIGH_BYTE(usDxlCrcCheck);
	
/*	
//////////////////////////////////////////////////////////////////////////////
#ifdef IN_DEBUG_MODE
	
	printf("in file: %s, line: %d\r\n", __FILE__, __LINE__);
	DXL_PrintBuf(g_ucaServoTxBuffer, usPacketSize, false);
	
#endif
//////////////////////////////////////////////////////////////////////////////
*/

	DMA_SendData( dmaServoTxStream, usPacketSize );
	
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	读取单个舵机的寄存器值(可以是多个连续的寄存器)
    * @param  	usRegAddr：寄存器地址
    * @param  	usRegSize：寄存器大小(Byte)
    * @param  	ucId：舵机id数组
	* @retval 	0：无误；非0：有误
    */
int DXL_RegRead(uint16_t usRegAddr, uint16_t usRegSize, uint8_t ucId, uint32_t *pDst)
{
	uint32_t ret  = 0;
	uint8_t i = 0;
	int start_byte;
	/* 参数长度：寄存器地址+寄存器长度*/
	uint16_t usParamLen = 4;
	/* 包体长度：参数长度 + 附加长度3（8位指令 + 16位校验码）*/
	uint16_t usPacketLen = PACKET_ADD_LEN + usParamLen;
	/* 数据包总长：包体长度 + 剩余长度7*/
	uint16_t usPacketSize = usPacketLen + PACKET_LEFT_LEN;
	
	uint16_t usDxlCrcCheck = 0U;
	
	g_ucaServoTxBuffer[eBYTE_ID]			= ucId;
	g_ucaServoTxBuffer[eBYTE_LEN_L]			= LOW_BYTE(usPacketLen);
	g_ucaServoTxBuffer[eBYTE_LEN_H]			= HIGH_BYTE(usPacketLen);
	g_ucaServoTxBuffer[eBYTE_INST]			= dxlINST_Read;
	g_ucaServoTxBuffer[BYTE_ParamN(1)]		= LOW_BYTE(usRegAddr);
	g_ucaServoTxBuffer[BYTE_ParamN(2)]		= HIGH_BYTE(usRegAddr);
	g_ucaServoTxBuffer[BYTE_ParamN(3)]		= LOW_BYTE(usRegSize);
	g_ucaServoTxBuffer[BYTE_ParamN(4)]		= HIGH_BYTE(usRegSize);
	
	usDxlCrcCheck = CrcCheck(g_ucaServoTxBuffer, usPacketSize - 2);
	g_ucaServoTxBuffer[usPacketSize - 2]	= LOW_BYTE(usDxlCrcCheck);
	g_ucaServoTxBuffer[usPacketSize - 1] 	= HIGH_BYTE(usDxlCrcCheck);
	
/*	
//////////////////////////////////////////////////////////////////////////////
#ifdef IN_DEBUG_MODE
	
	printf("in file: %s, line: %d\r\n", __FILE__, __LINE__);
	DXL_PrintBuf(g_ucaServoTxBuffer, usPacketSize, false);
	
#endif
//////////////////////////////////////////////////////////////////////////////
*/
	
	xServoMsg.usRegAddr = usRegAddr;
	xServoMsg.usRegSize = usRegSize;
	xServoMsg.byteToRecv = dxlMIN_STATUS_PACK_LEN + usRegSize;
	DXL_SetPacketReadEnable(&xServoMsg, ENABLE);
	DMA_SendData( dmaServoTxStream, usPacketSize );
	
	
	
	while( !xServoMsg.bDataReady )
		if( ret++ >= 9000 )
		{
			ret = -1;
			break;
		}
	if( ret == -1 )
		return 1;
	
	start_byte = DXL_FindFirstPackhead( xServoMsg.pDataBuf, xServoMsg.ucByteRecved );
	if( start_byte < 0 )
		return 2;
	else
	{
		*pDst = 0;
		for( i = 0; i < xServoMsg.usRegSize; i++ )
		{
			*pDst |= g_ucaServoRxBuffer[ start_byte + dxlStatusPackParamByte + i ] << ( i * 8 );
		}
	}
	
	xClearMsg( &xServoMsg );
	return 0;

	
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	读取多个舵机的寄存器值(可以是多个连续的寄存器)
    * @param  	usRegAddr：寄存器地址
    * @param  	usRegSize：寄存器大小(Byte)
    * @retval 	None
    */
void DXL_RegSyncRead(uint16_t usRegAddr, uint16_t usRegSize, uint8_t ucServoNum,
					uint8_t* ucIdBuf)
{
	uint8_t i = 0;
	uint8_t idCount = 0;
	/* 参数长度：寄存器地址+寄存器长度*/
	uint16_t usParamLen = 4 +  ucServoNum;
	/* 包体长度：参数长度 + 附加长度3（8位指令 + 16位校验码）*/
	uint16_t usPacketLen = PACKET_ADD_LEN + usParamLen;
	/* 数据包总长：包体长度 + 剩余长度7*/
	uint16_t usPacketSize = usPacketLen + PACKET_LEFT_LEN;
	
	uint16_t usDxlCrcCheck = 0U;
	
	while( usart2Busy );
	
	g_ucaServoTxBuffer[eBYTE_ID]		= ID_BROADCAST;
	g_ucaServoTxBuffer[eBYTE_LEN_L]		= LOW_BYTE(usPacketLen);
	g_ucaServoTxBuffer[eBYTE_LEN_H]		= HIGH_BYTE(usPacketLen);
	g_ucaServoTxBuffer[eBYTE_INST]		= dxlINST_Sync_Read;
	g_ucaServoTxBuffer[BYTE_ParamN(1)]	= LOW_BYTE(usRegAddr);
	g_ucaServoTxBuffer[BYTE_ParamN(2)]	= HIGH_BYTE(usRegAddr);
	g_ucaServoTxBuffer[BYTE_ParamN(3)]	= LOW_BYTE(usRegSize);
	g_ucaServoTxBuffer[BYTE_ParamN(4)]	= HIGH_BYTE(usRegSize);
	
	for(i=0; i<ctrlSERVO_NUM; i++)
	{
		if( ucDxlIdBuf[ i ] )
		{
			g_ucaServoTxBuffer[SYNC_OPT_START_ADDR + idCount ] = ucDxlIdBuf[i];
			idCount++;
		}
	}
	
	usDxlCrcCheck = CrcCheck(g_ucaServoTxBuffer, usPacketSize - 2);
	g_ucaServoTxBuffer[usPacketSize - 2] = LOW_BYTE(usDxlCrcCheck);
	g_ucaServoTxBuffer[usPacketSize - 1] = HIGH_BYTE(usDxlCrcCheck);

/*	
//////////////////////////////////////////////////////////////////////////////
#ifdef IN_DEBUG_MODE
	
	printf("in file: %s, line: %d\r\n", __FILE__, __LINE__);
	DXL_PrintBuf(g_ucaServoTxBuffer, usPacketSize, false);
	
#endif
//////////////////////////////////////////////////////////////////////////////
*/
	xClearMsg( &xServoMsg );
	xServoMsg.usRegAddr = usRegAddr;
	xServoMsg.usRegSize = usRegSize;
	xServoMsg.byteToRecv = ( s_ucOnlineServoCount ) * ( dxlMIN_STATUS_PACK_LEN + usRegSize );
	DXL_SetPacketReadEnable(&xServoMsg, ENABLE);
	DMA_SendData( dmaServoTxStream, usPacketSize );
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	设定消息结构体使能
	* @param  	usRegAddr：寄存器地址
	* @param  	usRegSize：寄存器大小
	* @param  	fPacketRead：接收舵机返回值使能
	* @retval 	None
	*/
void DXL_SetPacketReadEnable(ServoMsg* pServoMsg, FunctionalState fWriteEnable)
{
	if(fWriteEnable != DISABLE) 
	{
		pServoMsg->fWriteEnable = ENABLE;
		pServoMsg->fReadEnable = ENABLE;
	}
	
	else 
	{
		pServoMsg->fWriteEnable = DISABLE;
		pServoMsg->fReadEnable = DISABLE;
	}
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	解析舵机返回参数，放入舵机状态数组,
				此函数会调用下ClearMsg()清理结构体内容
	* @param  	pServoMsg：消息结构体指针
	* @retval 	返回有效数据包数目，或者-1：数据包长度不足，-2：数据中无包头，-3：数据有误
	*/
int DXL_GetPresentParam( ServoMsg* pServoMsg )
{
	uint16_t usBufLen = pServoMsg->ucByteRecved;
	uint16_t usRegAddr = pServoMsg->usRegAddr;
	uint16_t usRegSize = pServoMsg->usRegSize;
	uint8_t* ucSrcBuf = pServoMsg->pDataBuf;
	int packHeadIndex = 0;
	uint16_t packLen = 0;
	uint8_t statusPackLeftLen = usBufLen;
	uint8_t validPackCount = 0;
	uint16_t crc = 0;
	uint8_t statusBufCol = 0;
	double statusCoff = 0.0;
	uint8_t id = 0;
	uint32_t statusVal = 0;
	int i = 0;
	
	
	if( usBufLen < dxlMIN_STATUS_PACK_LEN ) { return -1; }

	//根据数据包内容确定数据存储位置及系数
	if(usRegAddr == dxlREG_Present_Voltage) 	
	{
		statusBufCol = dxlCOL_Volt;
		statusCoff = 0.10;						//电压，单位V
	}
	else if(usRegAddr == dxlREG_Present_Current) 	
	{
		statusBufCol = dxlCOL_Current;
		statusCoff = 2.29;						//电流，单位mA
	}
	else if(usRegAddr == dxlREG_Present_Velocity) 	
	{
		statusBufCol = dxlCOL_Velocity;
		statusCoff = 0.229*360.0/60.0;			//角速度
	}
	else if(usRegAddr == dxlREG_Present_Position) 	
	{
		statusBufCol = dxlCOL_Pos;
		statusCoff = 1;			//角速度
	}
	while( statusPackLeftLen >= dxlMIN_STATUS_PACK_LEN )
	{

		packHeadIndex = DXL_FindFirstPackhead( ucSrcBuf, statusPackLeftLen );
		//如果数据包中没有包头
		if( packHeadIndex < 0 ) 
		{
			xClearMsg( pServoMsg );
			statusPackLeftLen = 0;
			continue; 
		}
		//数组指针指向包头
		ucSrcBuf += packHeadIndex;
		
		packLen =  7 + ( ucSrcBuf[ dxlStatusPackLenLowByte ] | ucSrcBuf[ dxlStatusPackLenHighByte ] << 8 );
#ifdef IN_DEBUG_MODE
		for( i = 0; i < packLen; i++ )
			printf("%02X ", ucSrcBuf[ i ] );
		printf( "\r\n");
#endif
		//如果收到自己发送的数据包，则跳出此次循环
		if( ucSrcBuf[ dxlStatusPackInstByte ] != 0x55 )
		{
			ucSrcBuf += packLen;
			statusPackLeftLen -= packLen;
			continue;
		}
		
		//如果开启了数据校验
		if ( dxlEnableStatusCRC )
		{
			crc = CrcCheck( ucSrcBuf, packLen - 2 );
			if( LOW_BYTE( crc ) != ucSrcBuf[ packLen - 2 ] ||
				HIGH_BYTE( crc ) != ucSrcBuf[ packLen - 1 ] )
			{
				ucSrcBuf += packLen;
				statusPackLeftLen -= packLen;
				continue;
			}
		}
		
		//提取ID
		id = ucSrcBuf[ dxlStatusPackIdByte ];
		
		if( usRegAddr != dxlPing )
		{
			//提取数据
			for( i = 0; i < usRegSize; i++ ) 	
			{
				statusVal |= ( ( ucSrcBuf[ dxlStatusPackParamByte + i ] ) << ( 8 * i ) );
			}
			
			if( usRegAddr == dxlREG_Present_Current )
				s_usaServoCurrentBuf[ id - 1 ] = statusVal;
			if( usRegAddr == dxlREG_Present_Position )
				s_ulaServoPosBuf[ id - 1 ] = statusVal;
			if( usRegAddr == dxlREG_Present_Velocity )
				s_ulaServoVelBuf[ id - 1 ] = statusVal;
			if( usRegAddr == dxlREG_Present_Voltage )
				s_usServoVolt = statusVal;
			
		}
		
		else
			ucDxlIdBuf[ id - 1 ] = id;
		
		

		//写入舵机状态数组
		if( usRegSize == 2 )
			dServoStatusBuf [ statusBufCol ][ id - 1 ] = statusCoff * ( short ) ( statusVal & 0xffff );
		else if( usRegSize == 4 )
			dServoStatusBuf [ statusBufCol ][ id - 1 ] = ( int ) statusVal;

		
#ifdef IN_DEBUG_MODE
		printf("id: %d, type: %d, value: %f\r\n", id, statusBufCol, dServoStatusBuf [ statusBufCol ][ id - 1 ] );
#endif
		//有效数据包数量++，数组起始指针后移，数组剩余空间减少
		statusVal = 0;
		validPackCount++;
		ucSrcBuf += packLen;
		statusPackLeftLen -= packLen;
	}
	
	//重置数据包
	xClearMsg( pServoMsg );
	
	if( validPackCount == 0 ) 
	{
		return -3; 

	}
		
	return validPackCount;		
}	
/* ---------------------------------------------------------------------------*/

/****
	* @brief	寻找首个包头的位置
	* @param  	pucBuf：数据数组，len：数组长度
	* @retval 	非负：包头起始位置；否则，未找到包头
	*/
static int DXL_FindFirstPackhead( uint8_t * pucBuf, uint8_t len )
{
	uint8_t i = 0;
	
	if( len < dxlMIN_STATUS_PACK_LEN ) { return -1; }
	
	for( i = 0; i < len - dxlMIN_STATUS_PACK_LEN; i++ )
	{
		if( pucBuf[ i + 0 ] == 0xFF &&
			pucBuf[ i + 1 ] == 0xFF &&
			pucBuf[ i + 2 ] == 0xFD &&
			pucBuf[ i + 3 ] == 0x00 )
		//返回包头位置
		return i ;
	}
	
	//运行到此处即说明未找到包头
	return -2;
}
	
/* ---------------------------------------------------------------------------*/

/****
	* @brief	重设舵机及串口的波特率
	* @param  	ulBaudrate: 目标波特率
	* @retval 	None
	*/
void DXL_SetAllBaudrate(EDxlBaudrate eBaudrate)
{
	u8 i = 0;
	delay_ms( 50 );
	DXL_SetAllTorque(DISABLE);
	delay_ms( 50 );
	
	for( i = 0; i < ctrlSERVO_NUM; i++ )
	{
		DXL_RegWrite( dxlREG_Baudrate, 1, eBaudrate, ucDxlIdBuf[ i ] );
		delay_ms( 50 );
	}
	
	USART2_Init(ulDxlBaudrateBuf[eBaudrate]);
	delay_ms( 50 );
	DXL_SetAllTorque(ENABLE);
	
	blink( 4, 20, 50 );
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	设置全部舵机力矩开启/关闭(注意：全部是指id从1到SERVO_NUM的舵机)
	* @param  	bTorqueState：舵机力矩目标状态
	* @retval 	None
	*/
void DXL_SetAllTorque(FunctionalState fTorqueState)
{
	u8 i = 0;
	
	for (i=0; i<ctrlSERVO_NUM; i++) uxDxlParamBuf[i] = ((fTorqueState == DISABLE) ? 0 : 1);
	
	DXL_RegSyncWrite(dxlREG_Torque_Enable, REG_1_BYTE, ctrlSERVO_NUM,\
										uxDxlParamBuf, ucDxlIdBuf);
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	设定全部舵机目标位置(注意：全部是指id从1到SERVO_NUM的舵机)
	* @param  	ulGoalPosBuf：舵机目标位置数组	
    * @retval 	None
    */
void DXL_SetAllGoalPos(uint32_t* ulGoalPosBuf)
{
	DXL_RegSyncWrite(dxlREG_Goal_Position, REG_4_BYTE, ctrlSERVO_NUM,\
										ulGoalPosBuf, ucDxlIdBuf);
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	设定全部舵机目标位置(注意：全部是指id从1到SERVO_NUM的舵机)
	* @param  	ulGoalPosBuf：舵机目标位置数组	
    * @retval 	None
    */
void DXL_SetAllLedState(FunctionalState fLedState)
{
	u8 i = 0;
	
	for (i=0; i<ctrlSERVO_NUM; i++) uxDxlParamBuf[i] = ((fLedState == DISABLE) ? 0 : 1);
	DXL_RegSyncWrite(dxlREG_LED, REG_1_BYTE, ctrlSERVO_NUM,\
										uxDxlParamBuf, ucDxlIdBuf);
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	设定全部舵机的状态返回等级(注意：全部是指id从1到SERVO_NUM的舵机)
	* @param  	eReturnLevel：状态返回等级	
    * @retval 	None
    */
void DXL_SetAllReturnLv(EStatusReturnLv eReturnLevel)
{
	uint8_t i = 0;
	for(i=0; i<ctrlSERVO_NUM; i++)  uxDxlParamBuf[i] = eReturnLevel;
	
	DXL_RegSyncWrite(dxlREG_Status_Return_Lv, REG_1_BYTE, ctrlSERVO_NUM,\
										uxDxlParamBuf, ucDxlIdBuf);
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	设定全部舵机的状态返回延时(注意：全部是指id从1到SERVO_NUM的舵机)
	* @param  	usReturnDelayUS：状态返回延时	
    * @retval 	None
    */
void DXL_SetAllReturnDelay( uint16_t usReturnDelayUs )
{
	uint8_t i = 0;
	
	for( i = 0; i < ctrlSERVO_NUM; i++ ) uxDxlParamBuf[ i ] = usReturnDelayUs / 2;
	
	DXL_RegSyncWrite( dxlREG_Return_Delay_Us,
					  REG_1_BYTE,
					  ctrlSERVO_NUM,
					  uxDxlParamBuf,
					  ucDxlIdBuf);
}
	
/* ---------------------------------------------------------------------------*/

/****
	* @brief	读取所有舵机某个寄存器状态
	* @param  	usBuf：需要校验的数据数组
	* @param  	ucLen：数据长度
    * @retval 	None
    */
void DXL_GetRegState(uint16_t usRegAddr, uint16_t usRegSize)
{
	DXL_RegSyncRead(usRegAddr, usRegSize, s_ucOnlineServoCount, ucDxlIdBuf);
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	读取舵机LED状态
	* @param  	usBuf：需要校验的数据数组
	* @param  	ucLen：数据长度
    * @retval 	None
    */
void DXL_GetAllLedState(void)
{
	DXL_RegSyncRead(dxlREG_LED, REG_1_BYTE, ctrlSERVO_NUM, ucDxlIdBuf);
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	Ping，查看在线舵机数量
	* @param  	无
	* @retval 	在线舵机数量
    */
int DXL_Ping( void )
{
	int count;
	g_ucaServoTxBuffer[ eBYTE_ID ]			= ID_BROADCAST;
	g_ucaServoTxBuffer[ eBYTE_LEN_L ]		= LOW_BYTE( 0x03 );
	g_ucaServoTxBuffer[ eBYTE_LEN_H ]		= HIGH_BYTE( 0x00 );
	g_ucaServoTxBuffer[ eBYTE_INST ]		= dxlINST_Ping;
	g_ucaServoTxBuffer[ 8 ] 				= 0x31;
	g_ucaServoTxBuffer[ 9 ] 				= 0x42;
	
	xServoMsg.usRegAddr = dxlPing;
	xServoMsg.usRegSize = 3;
	xServoMsg.byteToRecv = dxlMIN_STATUS_PACK_LEN;
	DXL_SetPacketReadEnable(&xServoMsg, ENABLE);
	DMA_SendData( dmaServoTxStream, 10 );
	
	delay_ms( 2000 );

	count = DXL_GetPresentParam( &xServoMsg );
	s_ucOnlineServoCount = 0;
	if( count > 0 )
	{
		s_ucOnlineServoCount = count;
		return count;
	}
	else
		return -1;

}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	将舵机信息放入数组，低字节在前
	* @param  	msgType：数据类型
    * @retval 	0：无误， 小于0：有误
    */
int DXL_SetComBuf( COMM_MSG_E msgType )
{
	uint8_t pbuf[ HALF_BUF_SIZE ];
	uint8_t *p;
	uint16_t size;
	uint8_t regSize;
	uint8_t bytesCopied = 0;
	uint8_t servoOnlineCount = 0;
	uint8_t servoCount = ctrlSERVO_NUM;
	uint8_t ret;
	uint8_t i;
	if( msgType == MSG_SYS_Info || msgType == MSG_IMU )
		return 1;
	
	if( msgType == MSG_Servo_Current )
	{
		p = ( uint8_t *) s_usaServoCurrentBuf;
		size =  sizeof( uint16_t );
	}
	else if( msgType == MSG_Servo_Pos )
	{
		p = ( uint8_t *) s_ulaServoPosBuf;
		size = sizeof( uint32_t );
	}
	else if( msgType == MSG_Servo_Vel )
	{
		p = ( uint8_t *) s_ulaServoVelBuf;
		size = sizeof( uint32_t );
	}
	else if( msgType == MSG_Servo_Volt )
	{
		p = ( uint8_t *) &s_usServoVolt;
		size = sizeof( uint16_t );
		memcpy( pbuf, p, size );
		bytesCopied = 2;
	}

	
	if( msgType != MSG_Servo_Volt )
		for( i = 0; i < servoCount; i++ )
		{
			if( !ucDxlIdBuf[ i ] )
			{
				p += size;
				continue;
			}
			//id号+数据
			pbuf[ bytesCopied ] = ucDxlIdBuf[ i ];
			memcpy( pbuf + bytesCopied + 1, p, size );
			bytesCopied += size + 1;
			servoOnlineCount++;
			p += size;
		}
	
	
	ret = DC_SetBuffer( pbuf, bytesCopied, msgType );
	if( ret )
		return ( ret + 1 );
	return 0;
}

/* ---------------------------------------------------------------------------*/

/****
	* @brief	创建串口消息结构体
    * @param  	ucSrcBuf：消息指向的数据数组
    * @retval 	返回一个初始化的消息结构体
    */
ServoMsg xCreateMsg(uint8_t* ucSrcBuf)
{
	ServoMsg xMsg;
	xMsg.bDataReady 	= false;
	xMsg.fReadEnable 	= DISABLE;
	xMsg.fWriteEnable 	= DISABLE;
	xMsg.pDataBuf 		= ucSrcBuf;
	xMsg.ucByteRecved 	= 0;
	xMsg.usRegAddr 		= 0;
	xMsg.usRegSize 		= 0;
	
	return xMsg;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	重置串口消息结构体
    * @param  	stMsg：消息结构体地址
    * @retval 	None
    */
void xClearMsg(ServoMsg* pMsg)
{
	pMsg->bDataReady 	= false;
	pMsg->fWriteEnable 	= DISABLE;
	pMsg->fReadEnable 	= DISABLE;
	pMsg->ucByteRecved 	= 0;
	pMsg->usRegAddr 	= 0;
	pMsg->usRegSize 	= 0;
	pMsg->byteToRecv	= 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	检查消息结构体是否可用
    * @param  	stMsg：消息结构体
    * @retval 	None
    */
bool isMsgWriteEnable(ServoMsg xMsg)
{
	if(xMsg.fWriteEnable == DISABLE) return false;
	
	return true;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	检查消息结构体数据是否可用
    * @param  	stMsg：消息结构体
    * @retval 	None
    */
bool isMsgDataReady(ServoMsg xMsg)
{
	
	if(xMsg.fWriteEnable == DISABLE) return false;
	if(xMsg.bDataReady == false) return false;
	if(!xMsg.ucByteRecved) return false;

	return true;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	串口打印舵机状态数据
    * @param  	None
    * @retval 	None
    */
#ifdef IN_DEBUG_MODE
void DXL_PrintStatusBuf(void)
#else
static void DXL_PrintStatusBuf(void)
#endif
{
	uint8_t ucServoNum, i, j;
	
//	for(ucServoNum=0; ucServoNum<SERVO_NUM; ucServoNum++)	printf("%6d  ", ucServoNum);
//	printf("\r\n");
	
	printf( "Servo status:\t" );
	for( j = 0; i < ctrlSERVO_NUM; i++ )
			printf( "%2d\t", ( j + 1 ) );
		printf( "\r\n" );
	
	for( i = 0; i < dxlStatusCount; i++ )
	{
		printf( "%s\t", cServoStatusMsgBuf[ i ] );
		
		
		for( ucServoNum = 0; ucServoNum<ctrlSERVO_NUM; ucServoNum++)	
			printf("%2d: %6.3f  ", i, dServoStatusBuf[i][ucServoNum]);
		
		printf("\r\n");
	}

}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	打印数组的值或ASCII字符
	* @param  	ucBuf：数据数组
	* @param  	ucLen：数据长度
	* @param	bInchar：是否以字符形式打印
    * @retval 	None
    */
#ifdef IN_DEBUG_MODE
void DXL_PrintBuf(uint8_t* ucBuf, uint8_t ucLen, bool bInChar)
#else
static void DXL_PrintBuf(uint8_t* ucBuf, uint8_t ucLen, EPrintFormat ePrintFormat)
#endif
{
	while(ucLen--)	
	{
		if(ePrintFormat == eHex02) 
			printf("%02X ", *(ucBuf++));
		else 
			printf("%3d", *(ucBuf++));
	}
	printf("\r\n");
}



