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
uint8_t GYRO_RegSingleWrite( uint8_t ucRegAddr, uint16_t usSrc )
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
uint8_t GYRO_RegSingleRead( uint8_t ucRegAddr, uint16_t* pusDst )
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
uint8_t GYRO_RegMultiWrite( uint8_t ucRegAddr,
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
uint8_t GYRO_RegMultiRead(	uint8_t ucRegAddr,
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
	
	ret = GYRO_RegMultiRead(gyroYYMM, 4, usBuf );
	
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
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_RegMultiRead(gyroACC_AXISX, 3, usBuf );

	pAcc->x = ((short)usBuf[0] / 32768.0 * 16.0 * 9.8);
	pAcc->y = ((short)usBuf[1] / 32768.0 * 16.0 * 9.8);
	pAcc->z = ((short)usBuf[2] / 32768.0 * 16.0 * 9.8);
	
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
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_RegMultiRead( gyroANGV_AXISX, 3, usBuf );
	
	pAngleVel->x = usBuf[0] / 32768.0 * 2000.0;
	pAngleVel->y = usBuf[1] / 32768.0 * 2000.0;
	pAngleVel->z = usBuf[2] / 32768.0 * 2000.0;
	
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
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_RegMultiRead(gyroMAG_AXISX, 3, usBuf );
	
	pMag->roll 	= usBuf[0];
	pMag->roll 	= usBuf[1];
	pMag->roll 	= usBuf[2];
	
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
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = GYRO_RegMultiRead( gyroROLL, 3, usBuf );
	
	pAngle->pitch 	= (short)usBuf[0] / 32768.0 * 180.0;
	pAngle->roll 	= (short)usBuf[1] / 32768.0 * 180.0;
	pAngle->yaw 	= (short)usBuf[2] / 32768.0 * 180.0;
	
	return ret;
}	
/* ---------------------------------------------------------------------------*/
											   
/****
	* @brief	读取陀螺仪的加速度、角速度和角度数据，并存入数组
	* @param  	ucpDst: 数组地址
	* @retval 	大于0，表示读出的数据长度，小于0，则有错误
	*/
int GYRO_GetImuData( uint8_t *ucpDst )
{
	uint8_t tmp = 0;
	uint16_t usaTmp[ 3 ];
	int res = 0;
	
	res = GYRO_RegMultiRead( gyroACC_AXISX, 3, usaTmp );
	if( res )
		return -res;
	memcpy( ucpDst, usaTmp, 6 );
	
	res = GYRO_RegMultiRead( gyroANGV_AXISX, 3, usaTmp );
	if( res )
		return -res;
	memcpy( ucpDst + 6, usaTmp, 6 );
	
	res = GYRO_RegMultiRead( gyroROLL, 3, usaTmp );
	if( res )
		return -res;
	memcpy( ucpDst + 12, usaTmp, 6 );
		
	return 18;
	
}
	
	
	