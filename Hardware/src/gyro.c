/**
  ***********************************UTF-8***************************************
  * @file    gyro.c
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于定义陀螺仪数据读写函数
  ******************************************************************************  
  */ 

#include "Gyro.h"
#include "i2c.h"
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
uint8_t Gyro_RegSingleWrite(uint8_t ucRegAddr,
							uint16_t usSrc,
							GyroErrType_t* pErr)
{
	uint8_t ucBuf[2] = {LOW_BYTE(usSrc), HIGH_BYTE(usSrc)};
	return I2C_MultiWrite(GYRO_I2C, GYRO_SLAVE_ADDR<<1, ucRegAddr, 2, ucBuf, pErr);
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪单个寄存器
	* @param  	ucRegAddr：目标寄存器地址
	* @param  	usDst：存储寄存器数据的变量指针
	* @param  	pErr：错误信息指针
	* @retval 	None
	*/
uint8_t Gyro_RegSingleRead( uint8_t ucRegAddr,
							uint16_t* pusDst,
							GyroErrType_t* pErr)
{
	uint8_t ucBuf[2] = {0};
	uint8_t ret = 0;
	ret = I2C_MultiRead(GYRO_I2C, GYRO_SLAVE_ADDR<<1, ucRegAddr, 2, ucBuf, pErr);
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
uint8_t Gyro_RegMultiWrite( uint8_t ucRegAddr,
							uint8_t ucRegCount,
							uint16_t* pusSrcBuf,
							GyroErrType_t* pErr)
{
	//要写入的字节数量
	const uint8_t ucByteCount = 2 * ucRegCount;
	uint8_t ucBuf[ucByteCount];
	uint8_t i = 0;
	
	//将uint16数组转换成uint8数组，低字节在前
	for(i=0; i<ucRegCount; i++)
	{
		ucBuf[2 * i] 		= LOW_BYTE(*(pusSrcBuf+i));
		ucBuf[2 * i + 1] 	= HIGH_BYTE(*(pusSrcBuf+i));
	}
	
	return I2C_MultiWrite(GYRO_I2C, GYRO_SLAVE_ADDR<<1, ucRegAddr,
									ucByteCount, ucBuf, pErr);
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
uint8_t Gyro_RegMultiRead(	uint8_t ucRegAddr,
							uint8_t ucRegCount,
							uint16_t* pusDstBuf,
							GyroErrType_t* pErr)
{
	//要读取的字节数量
	const uint8_t ucByteCount = 2 * ucRegCount;
	uint8_t ucBuf[ucByteCount];
	uint8_t i = 0, ret;
	
	ret = I2C_MultiRead(GYRO_I2C, GYRO_SLAVE_ADDR<<1, ucRegAddr,
									ucByteCount, ucBuf, pErr);
	
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
GyroTimeType_t GYRO_GetCurrTime(GyroErrType_t* pErr)
{
	GyroTimeType_t xGyroTime;
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = Gyro_RegMultiRead(gyroYYMM, 4, usBuf, pErr);
	
	xGyroTime.ucYear 		= LOW_BYTE(usBuf[0]);
	xGyroTime.ucMonth 		= HIGH_BYTE(usBuf[0]);
	xGyroTime.ucDay 		= LOW_BYTE(usBuf[1]);
	xGyroTime.ucHour 		= HIGH_BYTE(usBuf[1]);
	xGyroTime.ucMinute		= LOW_BYTE(usBuf[2]);
	xGyroTime.ucSecond 		= HIGH_BYTE(usBuf[2]);
	xGyroTime.usMiliSecond 	= usBuf[3];
	
	return xGyroTime;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪加速度信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪加速度信息
	*/
GyroAccType_t Gyro_GetCurrAcc(GyroErrType_t* pErr)
{
	GyroAccType_t xGyroAcc;
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = Gyro_RegMultiRead(gyroACC_AXISX, 3, usBuf, pErr);

	xGyroAcc.x = ((short)usBuf[0] / 32768.0 * 16.0 * 9.8);
	xGyroAcc.y = ((short)usBuf[1] / 32768.0 * 16.0 * 9.8);
	xGyroAcc.z = ((short)usBuf[2] / 32768.0 * 16.0 * 9.8);
	
	return xGyroAcc;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪角速度信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪角速度信息
	*/
GyroAngVelType_t Gyro_GetCurrAngVel(GyroErrType_t* pErr)
{
	GyroAngVelType_t xGyroAngVel;
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = Gyro_RegMultiRead(gyroANGV_AXISX, 3, usBuf, pErr);
	
	xGyroAngVel.x = usBuf[0] / 32768.0 * 2000.0;
	xGyroAngVel.y = usBuf[1] / 32768.0 * 2000.0;
	xGyroAngVel.z = usBuf[2] / 32768.0 * 2000.0;
	
	return xGyroAngVel;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪磁场信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪磁场信息
	*/
GyroMagType_t Gyro_GetCurrMag(GyroErrType_t* pErr)
{
	GyroMagType_t xGyroMag;
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = Gyro_RegMultiRead(gyroMAG_AXISX, 3, usBuf, pErr);
	
	xGyroMag.roll 	= usBuf[0];
	xGyroMag.pitch 	= usBuf[1];
	xGyroMag.yaw 	= usBuf[2];
	
	return xGyroMag;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	读取陀螺仪角度信息
	* @param  	pErr：错误信息指针
	* @retval 	陀螺仪角度信息
	*/
GyroAngleType_t Gyro_GetCurrAng(GyroErrType_t* pErr)
{
	GyroAngleType_t xGyroAngle;
	uint16_t usBuf[GYRO_BUF_SIZE] = {0};
	uint8_t ret;
	
	ret = Gyro_RegMultiRead(gyroANGV_AXISX, 3, usBuf, pErr);
	
	xGyroAngle.roll 	= usBuf[0] / 32768.0 * 180.0;
	xGyroAngle.pitch 	= usBuf[1] / 32768.0 * 180.0;
	xGyroAngle.yaw 		= usBuf[2] / 32768.0 * 180.0;
	
	return xGyroAngle;
}	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	