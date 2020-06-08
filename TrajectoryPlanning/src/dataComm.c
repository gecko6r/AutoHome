/**
  ***********************************UTF-8**************************************
  * @file    dataComm.c
  * @author  Xiong
  * @version V1.0
  * @date    13-Aug-2020
  * @brief   此文件用于定义机器人通讯的数据缓冲区和读写函数,
  ******************************************************************************  
  */ 
  /*-----------------------------------上下位机通信格式-----------------------------------------
   ______________________________________________ ______________________________________________
  |                                                            |                   |           |
  |                                  7                         |     para_len      |     2     |
  |____________________________________________________________|___________________|___________|
  |                                                            |                   |     |     |  
  |Header1|Header2|Reserved||Length1|Length2Instruction|MsgType|       Param       |CRC1 | CRC2|
  |____________________________________________________________|___________________|_____|_____|
  |                                                            |                   |     |     |
  | 0xFF  | 0xFE  |  0xFD  | Len_L | Len_H |Instruction|MsgType|Param1|.....|ParamN|CRC_L|CRC_H|
  |____________________________________________________________|___________________|_____|_____|
  --------------------------------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "dataComm.h"
#include "nrf24l01p.h"
#include "dynamixel.h"
#include "usart.h"

/* 全局变量定义 */
BotStatusUpdated_t g_tStatusUpdated;


/* 局部变量定义 */
static uint8_t s_ucaSendBuf[ MAX_BUF_SIZE ];
static uint8_t s_ucaRecvBuf[ MAX_BUF_SIZE ];

static uint16_t s_sendPackLen;
static ComReg_t s_tComReg;

static bool s_packHeadRecved = false;
static uint16_t s_bytesRecved;
static uint16_t s_recvPackLen;
static bool s_packTotalRecved = false;

/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化DataManager
	* @param  	None
	* @retval 	成功则返回0
				否则返回1
	*/
uint8_t DC_Init(void)
{
	s_tComReg.ucReg &= 0x00;
	
	return 0;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	写入指定数组
	* @param  	None
	* @retval 	None
	*/	
uint8_t DC_SetBuffer(uint8_t *ucpSrcBuf, uint8_t ucSize, COMM_MSG_E eMsgType)
{
	uint16_t crc_code = 0;
	uint8_t *p = s_ucaSendBuf;
	
	if( s_tComReg.ucReg & 0x3f )
		return 1;
	
	if( ucpSrcBuf == NULL )
		return 2;
	
	if( !ucSize )
		return 2;
	
	
	//数组大小：数据长度加上头尾的9字节
	s_sendPackLen = ucSize + 9;
	
	//填充发送数组
	s_ucaSendBuf[ comBYTE_HEAD_1 ] 		= comPACK_HEAD_1;
	s_ucaSendBuf[ comBYTE_HEAD_2 ] 		= comPACK_HEAD_2;
	s_ucaSendBuf[ comBYTE_RESERVE ] 	= comPACK_HEAD_3;
	s_ucaSendBuf[ comBYTE_LEN_LOW ] 	= LOW_BYTE( s_sendPackLen - 5 );
	s_ucaSendBuf[ comBYTE_LEN_HIGH ] 	= HIGH_BYTE( s_sendPackLen - 5 );
	s_ucaSendBuf[ comBYTE_INSTRUCTION ] = eInstUploadData;
	s_ucaSendBuf[ comBYTE_MSG_TYPE ] 	= eMsgType;
	
	//拷贝数据
	memcpy( p + comBYTE_PARA_START, ucpSrcBuf, ucSize );

	
	//校验码计算
	crc_code = CrcCheck( s_ucaSendBuf, s_sendPackLen - 2 );
	s_ucaSendBuf[ s_sendPackLen - 2 ] = LOW_BYTE( crc_code );
	s_ucaSendBuf[ s_sendPackLen - 1 ] = HIGH_BYTE( crc_code );	
	
//	for( i = 0; i < s_sendPackLen; i++ )
//	{
//		printf("%02X ", s_ucaSendBuf[ i ] );
//	}
//	printf("\r\n");
	
	s_tComReg.ucReg |= ( 1 << eMsgType );
	
	return 0;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	flush指定数组
	* @param  	None
	* @retval 	None
	*/	
uint8_t DC_Send( void )
{
	if( !( s_tComReg.ucReg & 0x3f ) )
		return 1;
	
	NRF_SendData( s_ucaSendBuf, s_sendPackLen );
//	printf("\t%d\t", s_sendPackLen );
	
	s_tComReg.ucReg = 0x00;
	
	return 0;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	接收无线模块数据
	* @param  	单次接收到的数据长度
	* @retval 	PaclRecvStatus_e
	*/	
int DC_Recv( uint8_t size )
{
	uint8_t buf[ 32 ];
	uint8_t bytes;
	uint8_t *p = s_ucaRecvBuf;
	uint16_t crc;
	int index;
	int ret;
	
	
	bytes = NRF_RxPacket( buf, size );
	if( !bytes )
		return ePackRecvError;
	
	for( index = 0; index < bytes; index++ )
		printf("%02X ", buf[ index ] );
	printf("\r\n");
	
	memcpy( &s_ucaRecvBuf[ s_bytesRecved ], buf, bytes );
	s_bytesRecved += bytes;

	//没有收到包头
	if( !s_packHeadRecved )
	{
		if( p[ 0 ] == 0xFF && p[ 1 ] == 0xFE && p[ 2 ] == 0xFD )
		{
			s_packHeadRecved = true ;
			s_recvPackLen = 5 + ( p[ comBYTE_LEN_HIGH ] << 8 | p[ comBYTE_LEN_LOW ] );
			ret = ePackHeadRecved;
		}
		else
		{
			s_bytesRecved = 0;
			ret = ePackRecvError;
		}
	}	
	
	//如果已经收到包头
	if( s_packHeadRecved )
	{
		//如果数据帧长度小于数据包剩余长度
		if( s_bytesRecved >= s_recvPackLen )
		{
			s_packTotalRecved = true;
			ret = ePackDataRecved;
		}			
	}
	
	if( s_packTotalRecved )
	{
		//CRC校验
		crc = CrcCheck( p, s_recvPackLen - 2 );
		if( crc != ( p[ s_recvPackLen - 1 ] << 8 | p[ s_recvPackLen - 2 ] ) )
		{
			ret = ePackCrcError;
			
			//找到下一个包头，删除之前的数据
			index = DC_FindFirtPackHead( p + 3, s_bytesRecved - 3 );
			if( index >= 0 )
			{
				memmove( p, p + index, s_bytesRecved - index );
				s_bytesRecved -= index;
			}
			else
				s_bytesRecved = 0;	
		}
		else
		{
			ret = ePackTotalRecved;
			DC_GetPackParam( p + 5, s_recvPackLen - 7 );
			s_bytesRecved -= s_recvPackLen;
		}	
	}	
	return ret;	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	拆包
	* @param  	无
	* @retval 	0， 无误； 非0， 有误
	*/
uint8_t DC_GetPackParam( uint8_t *pbuf, uint16_t size )
{
	uint8_t inst, msg;
	int param;
	
	inst = pbuf[ 0 ];
	msg = pbuf[ 1 ];
	
	if( inst == eInstMANI )
	{
		if( msg == eMsgAdhesionCtrl )
		{
			param = pbuf[ 2 ];
			printf("param: %02X\r\n", param );
			CTRL_SetAdhesionPos( param );
		}
	}
	
	return 0;	
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	找到数组中的第一个包头
	* @param  	pbuf：数组
	* @param  	size：数据长度
    * @retval 	非负：包头起始位置；负数：错误
    */
int DC_FindFirtPackHead( uint8_t *pbuf, uint16_t size )
{
	int index = 0;
	while( size-- >= comMIN_PACK_SIZE )
	{
		if( *pbuf == 0xFF && *( pbuf + 1 ) == 0xFE && *( pbuf + 2 ) == 0xFD )
			return index;
		
		pbuf++;
		index++;
	}
	
	return -1;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	计算校验码
	* @param  	ucBuf：需要校验的数据数组
	* @param  	usLen：数据长度
    * @retval 	None
    */
uint16_t CrcCheck(uint8_t* ucBuf, uint16_t usLen)
{
	static const uint16_t CrcTable[256] = {
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
	uint16_t i, j;
    uint16_t usCrcAccum = 0;

    for(j = 0; j < usLen; j++)
    {
        i = ((u16)(usCrcAccum >> 8) ^ (*(ucBuf + j))) & 0xFF;
        usCrcAccum = (usCrcAccum << 8) ^ CrcTable[i];
    }

    return usCrcAccum;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	环形缓冲区创建函数
	* @param  	prbuf：环形缓冲区结构体指针
	* @retval 	0:创建成功；非0：创建失败
    */
int RingBuf_Create( RingBuf_t *prbuf, int size )
{
	uint8_t *p;
	
	if( size <= 0 )
		return eRingBufCreateError;
	
	p = malloc( size );
	if( p == NULL )
		return eRingBufCreateError;
	prbuf->pbuf = p;
	prbuf->head = 0;
	prbuf->tail = 0;
	prbuf->size = size;
	
	return eRingBufNoError;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	环形缓冲区添加数据
	* @param  	prbuf：环形缓冲区结构体指针
	* @param  	pbSrc：数据数组
	* @param  	size：数据数组大小
	* @retval 	0:添加数据成功；非0：添加失败
	* @note		如果不能成功添加数据，此次操作会直接作废（原子操作）
    */
int RingBuf_Append( RingBuf_t *prbuf, uint8_t *pbSrc, uint16_t len )
{
	uint8_t *p = prbuf->pbuf;
	uint16_t dataLeftLen;
	uint16_t ringBufAvailableLen;
	int head, tail, size;
	
	if( prbuf->pbuf == NULL )
		return eRingBufNull;
	
	ringBufAvailableLen = RingBuf_AvailableBytes( prbuf );
	if( len > ringBufAvailableLen )
		return eRingBufOverFlow;
	
	head = prbuf->head;
	tail = prbuf->tail;
	size = prbuf->size;
	
	//尾指针在头指针后
	if( tail > head )
	{
		//后部剩余空间不足以储存全部数据
		if( ( size - tail ) < len )
		{
			//如果尾指针指向数组末尾，则空余空间集中在开头
			if( tail ==  size )
				memcpy( p, pbSrc, len );
			//否则，剩余空间在开头和结尾
			else
			{
				memcpy( p + tail, pbSrc, ( size - tail ) );
				memcpy( p, pbSrc + ( size - tail ), len - ( size - tail ) );
			}
		}
		//后部剩余空间足够存储
		else
			memcpy( p, pbSrc, len );
	}
	//尾指针在头指针前
	else
	{
		memcpy( p + head, pbSrc, len );
	}
	
	prbuf->tail = ( tail + len ) % size;
	
	return eRingBufNoError;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	环形缓冲区删除数据
	* @param  	prbuf：环形缓冲区结构体指针
	* @param	start：删除元素起始位置
	* @param  	len：要删除的数据长度
	* @retval 	0:数据删除成功；非0：删除失败
	* @note		如果不能成功删除数据，此次操作会直接取消（原子操作）
    */		
int RingBuf_Remove( RingBuf_t *prbuf, uint16_t len )
{
	int bytesInBuf = RingBuf_UsedBytes( prbuf );
	
	if( prbuf->pbuf == NULL || prbuf->size == 0 )
		return eRingBufNull;
	
	if( len > bytesInBuf )
		return eRingBufUnderFlow;
	
	prbuf->head = ( prbuf->head + len ) % prbuf->size;
	
	return eRingBufNoError;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	查询数组已用空间
	* @param  	prbuf：环形缓冲区结构体指针
	* @retval 	非负：返回环形数组已用空间；负数：返回错误
    */	
int RingBuf_UsedBytes( RingBuf_t *prbuf )
{
	if( prbuf->pbuf == NULL || prbuf->size == 0 )
		return eRingBufNull;
	
	return ( prbuf->tail + prbuf->size - prbuf->head ) % prbuf->size;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	查询数组可用空间
	* @param  	prbuf：环形缓冲区结构体指针
	* @retval 	非负：返回环形数组可用空间；负数：返回错误
    */	
int RingBuf_AvailableBytes( RingBuf_t *prbuf )
{
	int bytes = RingBuf_UsedBytes( prbuf );
	if( bytes < 0 )
		return bytes;
	
	return prbuf->size - bytes;
}

void RingBuf_Clear( RingBuf_t *prbuf )
{
	if( prbuf->pbuf != NULL )
		free( prbuf->pbuf );
	
	prbuf->head = 0;
	prbuf->tail = 0;
	prbuf->size = 0;
}
