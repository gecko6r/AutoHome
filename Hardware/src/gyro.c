/**
  ***********************************UTF-8**************************************
  * @file    gyro.c
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于定义陀螺仪数据读写函数
  ******************************************************************************  
  */ 

#include <string.h>

#include "gyro.h"
#include "i2c.h"
#include "usart.h"
#include "dataComm.h"

/* 全局变量定义 */
GyroAcc_t 		g_tGyroAcc;
GyroAngVel_t 	g_tGyroAngVel;
GyroAngle_t 	g_tGyroAngle;


/* 局部变量定义 */
static const uint8_t s_ucDataBufSize = 18;
//加速度( x, y, z ), 速度( x, y, z ), 角度( x, y, z ), 每个数据2字节（低字节在前)
static uint8_t s_ucaDataBuf[ s_ucDataBufSize ];


/****
	* @brief	初始化陀螺仪
	* @param  	None
	* @retval 	None
	*/
void GYRO_Init(void)
{
	IIC_Init();
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	写入陀螺仪单个寄存器
	* @param  	ucRegAddr：目标寄存器地址
	* @param  	usSrc：写入寄存器的值
	* @param  	pErr：错误信息指针
	* @retval 	None
	*/
uint8_t GYRO_WriteReg( uint8_t ucRegAddr, uint16_t usSrc )
{
	uint8_t ucBuf[2] ={0};
	ucBuf[0] = LOW_BYTE(usSrc);
	ucBuf[1] = HIGH_BYTE(usSrc);
	
	return I2C_MultiWrite( GYRO_I2C, GYRO_SLAVE_ADDR<<1, ucRegAddr, 2, ucBuf );
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪单个寄存器
	* @param  	ucRegAddr：目标寄存器地址
	* @param  	usDst：存储寄存器数据的变量指针
	* @param  	pErr：错误信息指针
	* @retval 	None
	*/
uint8_t GYRO_ReadReg( uint8_t ucRegAddr, uint16_t* pusDst )
{
	uint8_t ucBuf[2] = {0};
	uint8_t ret = 0;
	ret = I2C_MultiRead(GYRO_I2C, GYRO_SLAVE_ADDR<<1, ucRegAddr, 2, ucBuf );
	*pusDst = (ucBuf[1]<<8 | ucBuf[0]);
	return ret;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	写入陀螺仪多个寄存器
	* @param  	ucRegAddr：目标寄存器起始地址
	* @param  	ucRegCount：寄存器数量
	* @param  	pusSrcBuf：写入寄存器的值数组
	* @param  	pErr：错误信息指针
	* @retval 	None
	*/
uint8_t GYRO_WriteBuf( uint8_t ucRegAddr,
							uint8_t ucRegCount,
							uint16_t* pusSrcBuf )
{
	//要写入的字节数量
	const uint8_t ucByteCount = 2 * ucRegCount;
	uint8_t ucBuf[GYRO_BUF_SIZE] = {0};
	uint8_t i = 0;
	
	//将uint16数组转换成uint8数组，低字节在前
	for(i=0; i<ucRegCount; i++)
	{
		ucBuf[2 * i] 		= LOW_BYTE(*(pusSrcBuf+i));
		ucBuf[2 * i + 1] 	= HIGH_BYTE(*(pusSrcBuf+i));
	}
	
	return I2C_MultiWrite(GYRO_I2C, GYRO_SLAVE_ADDR<<1, ucRegAddr,
									ucByteCount, ucBuf );
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪多个寄存器
	* @param  	ucRegAddr：目标寄存器起始地址
	* @param  	ucRegCount：寄存器数量
	* @param  	pusSrcBuf：存储寄存器值的值数组
	* @param  	pErr：错误信息指针
	* @retval 	None
	*/
uint8_t GYRO_ReadBuf(	uint8_t ucRegAddr,
							uint8_t ucRegCount,
							uint16_t* pusDstBuf )
{
	//要读取的字节数量
	const uint8_t ucByteCount = 2 * ucRegCount;
	uint8_t ucBuf[GYRO_BUF_SIZE] = {0};
	uint8_t i = 0, ret;
	
	ret = I2C_MultiRead(GYRO_I2C, 0xA0, ucRegAddr,
									ucByteCount, ucBuf );
	
	//将uint8数组转换成16数组，低字节在前
	for(i=0; i<ucRegCount; i++)
	{
		*(pusDstBuf + i) = (ucBuf[2 * i + 1]<<8 | ucBuf[2*i]);
	}
	
	return ret;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪时间信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪时间信息
	*/
uint8_t	GYRO_GetCurrTime( GyroTime_t* pTime)
{
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_ReadBuf(gyroREG_YYMM, 4, usBuf );
	
	pTime->ucYear 		= LOW_BYTE(usBuf[0]);
	pTime->ucMonth 		= HIGH_BYTE(usBuf[0]);
	pTime->ucDay 		= LOW_BYTE(usBuf[1]);
	pTime->ucHour 		= HIGH_BYTE(usBuf[1]);
	pTime->ucMinute		= LOW_BYTE(usBuf[2]);
	pTime->ucSecond 	= HIGH_BYTE(usBuf[2]);
	pTime->usMiliSecond = usBuf[3];
	
	return ret;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪加速度信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪加速度信息
	*/
uint8_t	GYRO_GetCurrAcc( GyroAcc_t* pAcc)
{
	short sBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_ReadBuf(gyroREG_ACC_AXISX, 3, ( uint16_t* )sBuf );

	pAcc->x = ((short)sBuf[0] / 32768.0 * 16.0 * 9.8);
	pAcc->y = ((short)sBuf[1] / 32768.0 * 16.0 * 9.8);
	pAcc->z = ((short)sBuf[2] / 32768.0 * 16.0 * 9.8);
	
	return ret;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪角速度信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪角速度信息
	*/
uint8_t	GYRO_GetCurrAngVel( GyroAngVel_t* pAngleVel )
{
	short sBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_ReadBuf( gyroREG_ANGV_AXISX, 3, ( uint16_t* )sBuf );
	
	pAngleVel->x = sBuf[0] / 32768.0 * 2000.0;
	pAngleVel->y = sBuf[1] / 32768.0 * 2000.0;
	pAngleVel->z = sBuf[2] / 32768.0 * 2000.0;
	
	return ret;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪磁场信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪磁场信息
	*/
uint8_t	GYRO_GetCurrMag( GyroMag_t* pMag)
{
	short sBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_ReadBuf(gyroREG_MAG_AXISX, 3, ( uint16_t* )sBuf );
	
	pMag->roll 	= sBuf[0];
	pMag->pitch = sBuf[1];
	pMag->yaw 	= sBuf[2];
	
	return ret;
}
/* ---------------------------------------------------------------------------*/
											   
/****
	* @brief	读取陀螺仪角度信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪角度信息
	*/
uint8_t	GYRO_GetCurrAng( GyroAngle_t* pAngle)
{
	short sBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_ReadBuf( gyroREG_ROLL, 3, ( uint16_t* )sBuf );
	
	pAngle->pitch 	= sBuf[0] / 32768.0 * 180.0;
	pAngle->roll 	= sBuf[1] / 32768.0 * 180.0;
	pAngle->yaw 	= sBuf[2] / 32768.0 * 180.0;
	
	return ret;
}	
/* ---------------------------------------------------------------------------*/
											   
/****
	* @brief	读取陀螺仪状态信息( 加速度， 角速度， 角度 )
	* @param  	无
	* @retval 	0，无误；非0，有误
	*/
uint8_t GYRO_GetImuData( void )
{
	short buf[ 12 ];
	uint8_t ret = 0;
	
	ret = GYRO_ReadBuf( gyroREG_ACC_AXISX, 12, ( uint16_t* )buf );
	if( ret )
		return ret;
	
	g_tGyroAcc.x 		= buf[ 0 ] / 32768.0 * 8.0 * 9.8;
	g_tGyroAcc.y 		= buf[ 1 ] / 32768.0 * 8.0 * 9.8;
	g_tGyroAcc.z 		= buf[ 2 ] / 32768.0 * 8.0 * 9.8;
	
	g_tGyroAngVel.x 	= buf[ 3 ] / 32768.0 * 2000.0;
	g_tGyroAngVel.y 	= buf[ 4 ] / 32768.0 * 2000.0;
	g_tGyroAngVel.z 	= buf[ 5 ] / 32768.0 * 2000.0;
	
	g_tGyroAngle.pitch	= buf[ 9 ] / 32768.0 * 180.0;
	g_tGyroAngle.roll	= buf[ 10 ] / 32768.0 * 180.0;
	g_tGyroAngle.yaw	= buf[ 11 ] / 32768.0 * 180.0;
	
	memcpy( s_ucaDataBuf, buf, 12 );
	memcpy( &s_ucaDataBuf[ 12 ], &buf[ 9 ], 6 );
	
	DC_SetBuffer( s_ucaDataBuf, 18, eMsgIMU );
	
	return ret;

}
/* ---------------------------------------------------------------------------*/
											   
/****
	* @brief	读取陀螺仪的加速度、角速度和角度数据，并存入数组,
	* @param  	ucpDst: 数组地址
	* @retval 	大于0，表示读出的数据长度，小于0，则有错误
	*/
int GYRO_SetBuffer( void )
{
	return DC_SetBuffer( s_ucaDataBuf, 18, eMsgIMU );
	
}
