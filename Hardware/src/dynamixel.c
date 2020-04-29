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
 
#include "stdio.h"

#include "dynamixel.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "dma.h"
#include "controller.h"

/* Static Vaiables------------------------------------------------------------*/
static uint8_t ucDxlIdBuf[ctrlSERVO_NUM] = {0U};
static uint32_t uxDxlParamBuf[ctrlSERVO_NUM] = {0U};

/* Global Vaiables------------------------------------------------------------*/
float fServoStatusBuf[SERVO_STATUS_NUM][ctrlSERVO_NUM] = {0U};
ServoMsg xServoMsg;


/* Constants------------------------------------------------------------------*/
//DXL servo supported baudrates
static const uint32_t ulDxlBaudrateBuf[8] = {9600, 57600, 115200,
											1000000, 2000000, 3000000,
											4000000, 4500000};			



static const u16 DXL_CrcTable[256] = {
	0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
	0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
	0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
	0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
	0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
	0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
	0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
	0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
	0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
	0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
	0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
	0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
	0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
	0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
	0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
	0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
	0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
	0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
	0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
	0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
	0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
	0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
	0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
	0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
	0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
	0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
	0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

static const char* cServoStatusMsgBuf[SERVO_STATUS_NUM] = {		
								"Present position:      ",
								"Present voltage:       ",
								"Present current:       ",
								"Present velocity:      ",
								"Present realtime tick: ",};
/* ---------------------------------------------------------------------------*/

static uint16_t DXL_CrcCheck(uint8_t* ucBuf, uint8_t ucLen);
									   
/* Dynamixel 相关函数定义-----------------------------------------------------*/		
											   
/****
	* @brief	初始化舵机通讯数组、串口、DMA，并使能舵机力矩
	* @param  	ulBaudrate: 对应串口的波特率
	* @param  	bTorqueState: 舵机力矩使能
	* @retval 	None
	*/
void DXL_ServoInit(EDxlBaudrate eBaudrate, FunctionalState fTorqueState)
{
	uint8_t i = 0;
	
	//初始化舵机id数组
	for (i=0; i<ctrlSERVO_NUM; i++)
		ucDxlIdBuf[i] = i+1;
	
	//初始化舵机通讯数组包头
	ucServoTxBuffer[eBYTE_HEADER_1]	= 0xff;
	ucServoTxBuffer[eBYTE_HEADER_2]	= 0xff;
	ucServoTxBuffer[eBYTE_HRADER_3]	= 0xfd;
	ucServoTxBuffer[eBYTE_RESV]		= 0x00;
	
	//初始化串口及dma
	USART2_Init(ulDxlBaudrateBuf[eBaudrate]);
	DMA_Usart2_Tx_Init();
	DMA_Usart2_Rx_Init();
	
	//设定舵机力矩
	DXL_SetAllTorque(fTorqueState);
	
	xServoMsg = xCreateMsg(ucServoRxBuffer);
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
	
	uint16_t usDxlCrcCheck = 0U;

	ucServoTxBuffer[eBYTE_ID]			= ucId;
	ucServoTxBuffer[eBYTE_LEN_L]		= LOW_BYTE(usPacketLen);
	ucServoTxBuffer[eBYTE_LEN_H]		= HIGH_BYTE(usPacketLen);
	ucServoTxBuffer[eBYTE_INST]			= dxlINST_REG_Write;
	ucServoTxBuffer[BYTE_ParamN(1)]		= LOW_BYTE(usRegAddr);
	ucServoTxBuffer[BYTE_ParamN(2)]		= HIGH_BYTE(usRegAddr);

	
	for(i=0; i<usRegSize; i++) ucServoTxBuffer[4+i] = nthByteOf(ulData, i);
	
	
	usDxlCrcCheck = DXL_CrcCheck(ucServoTxBuffer, usPacketSize - 2);
	ucServoTxBuffer[usPacketSize - 2] = LOW_BYTE(usDxlCrcCheck);
	ucServoTxBuffer[usPacketSize - 1] = HIGH_BYTE(usDxlCrcCheck);

/*	
//////////////////////////////////////////////////////////////////////////////
#ifdef IN_DEBUG_MODE
	
	printf("in file: %s, line: %d\r\n", __FILE__, __LINE__);
	DXL_PrintBuf(ucServoTxBuffer, usPacketSize, false);
	
#endif
//////////////////////////////////////////////////////////////////////////////
*/
	
	DMA_SendData(ucServoTxBuffer, usPacketSize);
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
	
	/* 参数长度：数据长度（Para1~ParaN） + 附加长度（寄存器地址，寄存器长度）*/
	uint16_t usParamLen = 2+PARAM_ADD_LEN + ucServoNum * ( usRegSize + 1 );
	/* 包体长度：参数长度 + 附加长度（8位指令 + 16位校验码）*/
	uint16_t usPacketLen = PACKET_ADD_LEN + usParamLen;
	/* 数据包总长：包体长度 + 剩余长度*/
	uint16_t usPacketSize = usPacketLen + PACKET_LEFT_LEN;
	
	uint16_t usDxlCrcCheck = 0U;
	
	ucServoTxBuffer[eBYTE_ID]			= ID_BROADCAST;
	ucServoTxBuffer[eBYTE_LEN_L]		= LOW_BYTE(usPacketLen);
	ucServoTxBuffer[eBYTE_LEN_H]		= HIGH_BYTE(usPacketLen);
	ucServoTxBuffer[eBYTE_INST]			= dxlINST_Sync_Write;
	ucServoTxBuffer[BYTE_ParamN(1)]		= LOW_BYTE(usRegAddr);
	ucServoTxBuffer[BYTE_ParamN(2)]		= HIGH_BYTE(usRegAddr);
	ucServoTxBuffer[BYTE_ParamN(3)]		= LOW_BYTE(usRegSize);
	ucServoTxBuffer[BYTE_ParamN(4)]		= HIGH_BYTE(usRegSize);
	
	for(i=0; i<ucServoNum; i++) 
	{
		ucServoTxBuffer[SYNC_OPT_START_ADDR + i*(usRegSize+1)] = ucIdBuf[i];

		for(j=0; j<usRegSize; j++)
			ucServoTxBuffer[SYNC_OPT_START_ADDR+i*(usRegSize+1)+1 + j] 
												= nthByteOf(ulDataBuf[i], j);
	}
	
	usDxlCrcCheck = DXL_CrcCheck(ucServoTxBuffer, usPacketSize - 2);
	ucServoTxBuffer[usPacketSize - 2] = LOW_BYTE(usDxlCrcCheck);
	ucServoTxBuffer[usPacketSize - 1] = HIGH_BYTE(usDxlCrcCheck);
	
/*	
//////////////////////////////////////////////////////////////////////////////
#ifdef IN_DEBUG_MODE
	
	printf("in file: %s, line: %d\r\n", __FILE__, __LINE__);
	DXL_PrintBuf(ucServoTxBuffer, usPacketSize, false);
	
#endif
//////////////////////////////////////////////////////////////////////////////
*/

	DMA_SendData(ucServoTxBuffer, usPacketSize);
	
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	读取单个舵机的寄存器值(可以是多个连续的寄存器)
    * @param  	usRegAddr：寄存器地址
    * @param  	usRegSize：寄存器大小(Byte)
    * @param  	ucId：舵机id数组
    * @retval 	None
    */
void DXL_RegRead(uint16_t usRegAddr, uint16_t usRegSize, uint8_t ucId)
{
	/* 参数长度：寄存器地址+寄存器长度*/
	uint16_t usParamLen = 4;
	/* 包体长度：参数长度 + 附加长度3（8位指令 + 16位校验码）*/
	uint16_t usPacketLen = PACKET_ADD_LEN + usParamLen;
	/* 数据包总长：包体长度 + 剩余长度7*/
	uint16_t usPacketSize = usPacketLen + PACKET_LEFT_LEN;
	
	uint16_t usDxlCrcCheck = 0U;
	
	ucServoTxBuffer[eBYTE_ID]		= ucId;
	ucServoTxBuffer[eBYTE_LEN_L]	= LOW_BYTE(usPacketLen);
	ucServoTxBuffer[eBYTE_LEN_H]	= HIGH_BYTE(usPacketLen);
	ucServoTxBuffer[eBYTE_INST]		= dxlINST_Read;
	ucServoTxBuffer[BYTE_ParamN(1)]	= LOW_BYTE(usRegAddr);
	ucServoTxBuffer[BYTE_ParamN(2)]	= HIGH_BYTE(usRegAddr);
	ucServoTxBuffer[BYTE_ParamN(3)]	= LOW_BYTE(usRegSize);
	ucServoTxBuffer[BYTE_ParamN(4)]	= HIGH_BYTE(usRegSize);
	
	usDxlCrcCheck = DXL_CrcCheck(ucServoTxBuffer, usPacketSize - 2);
	ucServoTxBuffer[usPacketSize - 2] = LOW_BYTE(usDxlCrcCheck);
	ucServoTxBuffer[usPacketSize - 1] = HIGH_BYTE(usDxlCrcCheck);
	
/*	
//////////////////////////////////////////////////////////////////////////////
#ifdef IN_DEBUG_MODE
	
	printf("in file: %s, line: %d\r\n", __FILE__, __LINE__);
	DXL_PrintBuf(ucServoTxBuffer, usPacketSize, false);
	
#endif
//////////////////////////////////////////////////////////////////////////////
*/
	
	DMA_SendData(ucServoTxBuffer, usPacketSize);
	
	xServoMsg.usRegAddr = usRegAddr;
	xServoMsg.usRegSize = usRegSize;
	DXL_SetPacketReadEnable(&xServoMsg, ENABLE);
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
	/* 参数长度：寄存器地址+寄存器长度*/
	uint16_t usParamLen = 4 +  ucServoNum;
	/* 包体长度：参数长度 + 附加长度3（8位指令 + 16位校验码）*/
	uint16_t usPacketLen = PACKET_ADD_LEN + usParamLen;
	/* 数据包总长：包体长度 + 剩余长度7*/
	uint16_t usPacketSize = usPacketLen + PACKET_LEFT_LEN;
	
	uint16_t usDxlCrcCheck = 0U;
	
	ucServoTxBuffer[eBYTE_ID]		= ID_BROADCAST;
	ucServoTxBuffer[eBYTE_LEN_L]	= LOW_BYTE(usPacketLen);
	ucServoTxBuffer[eBYTE_LEN_H]	= HIGH_BYTE(usPacketLen);
	ucServoTxBuffer[eBYTE_INST]		= dxlINST_Sync_Read;
	ucServoTxBuffer[BYTE_ParamN(1)]	= LOW_BYTE(usRegAddr);
	ucServoTxBuffer[BYTE_ParamN(2)]	= HIGH_BYTE(usRegAddr);
	ucServoTxBuffer[BYTE_ParamN(3)]	= LOW_BYTE(usRegSize);
	ucServoTxBuffer[BYTE_ParamN(4)]	= HIGH_BYTE(usRegSize);
	
	for(i=0; i<ucServoNum; i++)
	{
		ucServoTxBuffer[SYNC_OPT_START_ADDR + i] = ucDxlIdBuf[i];
	}
	
	usDxlCrcCheck = DXL_CrcCheck(ucServoTxBuffer, usPacketSize - 2);
	ucServoTxBuffer[usPacketSize - 2] = LOW_BYTE(usDxlCrcCheck);
	ucServoTxBuffer[usPacketSize - 1] = HIGH_BYTE(usDxlCrcCheck);

/*	
//////////////////////////////////////////////////////////////////////////////
#ifdef IN_DEBUG_MODE
	
	printf("in file: %s, line: %d\r\n", __FILE__, __LINE__);
	DXL_PrintBuf(ucServoTxBuffer, usPacketSize, false);
	
#endif
//////////////////////////////////////////////////////////////////////////////
*/
	
	DMA_SendData(ucServoTxBuffer, usPacketSize);
	
	xServoMsg.usRegAddr = usRegAddr;
	xServoMsg.usRegSize = usRegSize;
	DXL_SetPacketReadEnable(&xServoMsg, ENABLE);
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
	if(fWriteEnable != DISABLE) pServoMsg->fWriteEnable = ENABLE;
	
	else pServoMsg->fWriteEnable = DISABLE;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	解析舵机返回参数，放入舵机状态数组,
				此函数会调用下ClearMsg()清理结构体内容
	* @param  	pServoMsg：消息结构体指针
	* @retval 	None
	*/
uint8_t DXL_GetPresentParam(ServoMsg* pServoMsg)
{
	uint16_t usBufLen = pServoMsg->ucByteRecved;
	uint16_t usRegAddr = pServoMsg->usRegAddr;
	uint16_t usRegSize = pServoMsg->usRegSize;
	uint8_t* ucSrcBuf = pServoMsg->pDataBuf;
	uint16_t usStartByte = 0, usCrcByte = 0, usSinglePackLen = 0;
	uint8_t ucCol = 0, ucId = 0;
	float fCoff = 0.0;
	uint16_t usCrc = 0;
	uint16_t i = 0, j=0, ucPackNum = 0;
	uint32_t ulStatus = 0;
	int lStatus = 0;
	signed short sStatus = 0;
	
	//如果数据量不足一个最小的数据包长度
	if(usBufLen < MIN_STATUS_PACK_LEN)
		return 1;
	
	//设定当前数据处理方式及存储位置
	if(usRegAddr == dxlREG_Present_Position) 	{ucCol = COL_Pos;		fCoff = 1.0;}
	if(usRegAddr == dxlREG_Present_Voltage) 	{ucCol = COL_Volt; 		fCoff = 0.10;}
	if(usRegAddr == dxlREG_Present_Current) 	{ucCol = COL_Current; 	fCoff = 2.29;}
	if(usRegAddr == dxlREG_Present_Velocity) 	{ucCol = COL_Velocity; 	fCoff = 0.229*360.0/60.0;}
	if(usRegAddr == dxlREG_Realtime_Tick) 	{ucCol = COL_Tick; 		fCoff = 1.0;}

	//寻找数据包起始位置
	for(i=0; i<(usBufLen-3); i++)
	{
		if((ucSrcBuf[i]==0xFF) && (ucSrcBuf[i+1]==0xFF) &&\
			(ucSrcBuf[i+2]==0xFD) &&(ucSrcBuf[i+3]==0x00))
		{
			usStartByte = i;
			break;
		}
			
		return 2;
	}

	//计算单个数据包长度
	usSinglePackLen = MIN_STATUS_PACK_LEN + usRegSize;
	
	//计算数组内数据包数量
	ucPackNum = ((usBufLen - usStartByte) / usSinglePackLen);

	//如果没有数据包
	if(!ucPackNum)
	{
		return 2;
	}

	//参数解析
	for(i=0; i<ucPackNum; i++)
	{
		
		//若收到了自己发送的数据，则跳过
		if(*(ucSrcBuf + usStartByte + STATUS_INST_BYTE) != 0x55) continue;
		
		//计算校验码
		usCrc = DXL_CrcCheck((ucSrcBuf + usStartByte +i*usSinglePackLen), (usSinglePackLen-2));
		usCrcByte = usStartByte + i*usSinglePackLen + usSinglePackLen - 2;
		
		//若校验码有误，则跳过此数据包
		if((LOW_BYTE(usCrc) != ucSrcBuf[usCrcByte]) ||\
			(HIGH_BYTE(usCrc) != ucSrcBuf[usCrcByte+1])) continue;
		
		//提取数据
		for(j=0; j<usRegSize; j++) 	
			ulStatus |= ((ucSrcBuf[usStartByte + i*usSinglePackLen\
						+ Stasus_Param_Byte + j]) << (8 * j));
		
		//提取数据包对应id
		ucId = ucSrcBuf[usStartByte + i*usSinglePackLen + Status_ID_Byte];	
		
		//部分数据为有符号数据，需转换成int或short
		if(usRegSize == 2) 
		{
			sStatus = (short) (ulStatus & 0xffff);
			//通过系数计算实际值
			fServoStatusBuf[ucCol][ucId-1] = sStatus * fCoff;
		}
		else 
		{
			lStatus = (int)ulStatus;
			//通过系数计算实际值
			fServoStatusBuf[ucCol][ucId-1] = lStatus * fCoff;
		}
		
	}

	//清理消息结构体内容
	xClearMsg(pServoMsg);
	return 0;
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
	
	DXL_SetAllTorque(DISABLE);
	delay_ms(1000);
	
	for (i=0; i<ctrlSERVO_NUM; i++) uxDxlParamBuf[i] = eBaudrate;
	DXL_RegSyncWrite(dxlREG_Baudrate, REG_1_BYTE, ctrlSERVO_NUM,
										uxDxlParamBuf, ucDxlIdBuf);
	delay_ms(1000);
	USART2_Init(ulDxlBaudrateBuf[eBaudrate]);
	delay_ms(1000);
	DXL_SetAllTorque(ENABLE);
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
	* @param  	ulGoalPosBuf：舵机目标位置数组	
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
	* @brief	读取所有舵机某个寄存器状态
	* @param  	usBuf：需要校验的数据数组
	* @param  	ucLen：数据长度
    * @retval 	None
    */
void DXL_GetRegState(uint16_t usRegAddr, uint16_t usRegSize)
{
	DXL_RegSyncRead(usRegAddr, usRegSize, 16, ucDxlIdBuf);
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
	* @brief	创建串口消息结构体
    * @param  	ucSrcBuf：消息指向的数据数组
    * @retval 	返回一个初始化的消息结构体
    */
ServoMsg xCreateMsg(uint8_t* ucSrcBuf)
{
	ServoMsg xMsg;
	xMsg.bDataReady = false;
	xMsg.fWriteEnable = DISABLE;
	xMsg.pDataBuf = ucSrcBuf;
	xMsg.ucByteRecved = 0;
	xMsg.usRegAddr = 0;
	xMsg.usRegSize = 0;
	
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
	pMsg->bDataReady = false;
	pMsg->fWriteEnable = DISABLE;
	pMsg->ucByteRecved = 0;
	pMsg->usRegAddr = 0;
	pMsg->usRegSize = 0;
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
	* @brief	计算校验码
	* @param  	usBuf：需要校验的数据数组
	* @param  	ucLen：数据长度
    * @retval 	None
    */
#ifdef IN_DEBUG_MODE
uint16_t DXL_CrcCheck(uint8_t* ucBuf, uint8_t ucLen)
#else
static uint16_t DXL_CrcCheck(uint8_t* ucBuf, uint8_t ucLen)
#endif

{
	uint16_t i, j;
    uint16_t usCrcAccum = 0;

    for(j = 0; j < ucLen; j++)
    {
        i = ((u16)(usCrcAccum >> 8) ^ (*(ucBuf + j))) & 0xFF;
        usCrcAccum = (usCrcAccum << 8) ^ DXL_CrcTable[i];
    }

    return usCrcAccum;
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
	uint8_t ucServoNum, ucStatusNum;
	
//	for(ucServoNum=0; ucServoNum<SERVO_NUM; ucServoNum++)	printf("%6d  ", ucServoNum);
//	printf("\r\n");
	
	for(ucStatusNum=0; ucStatusNum<SERVO_STATUS_NUM; ucStatusNum++)
	{
		printf("%s", cServoStatusMsgBuf[ucStatusNum]);
		
		for(ucServoNum=0; ucServoNum<ctrlSERVO_NUM; ucServoNum++)	
			printf("%2d: %6.3f  ", ucStatusNum, fServoStatusBuf[ucStatusNum][ucServoNum]);
		
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
			printf("%02X", *(ucBuf++));
		else 
			printf("%3d", *(ucBuf++));
	}
	printf("\r\n");
}



