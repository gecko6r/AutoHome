/**
  ***********************************UTF-8***************************************
  * @file    gyro.h
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于定义陀螺仪数据读写函数
  ******************************************************************************  
  */ 

#ifndef _GYRO_H
#define _GYRO_H

#include "sys.h"
#include "i2c.h"

/* Macro definations -----------------------------------------*/

#define GYRO_I2C				((I2C_TypeDef*) I2C2)
#define GYRO_SLAVE_ADDR			((uint8_t) 0x50)
#define GYRO_BUF_SIZE			((uint8_t) 20)
#define LOW_BYTE(a)				((uint8_t)	(((uint16_t)(a)) &	0xff))
#define HIGH_BYTE(a)			((uint8_t)	(((uint16_t)(a)) >> 8))

/* Macro type definations -----------------------------------------*/
#define GyroErr_t 				I2cErr_t
#define GyroTime_t				GyroTimeType
#define GyroAcc_t				GyroAccType
#define GyroAngVel_t			GyroAngVelType
#define GyroMag_t				GyroMagType
#define GyroAngle_t				GyroAngleType
#define GyroPortStatus_t		GyroPortStatusType
#define GyroGPSInfo_t			GyroGPSInfoType
#define GyroQ_t					GyroQType

/* 陀螺仪寄存器地址 -----------------------------------------*/	
#define gyroSAVE				((uint8_t) 0x00)		//保存当前配置
#define gyroCALIB				((uint8_t) 0x01)		//校准
#define gyroRSW					((uint8_t) 0x02)		//回传数据内容
#define gyroRATE				((uint8_t) 0x03)		//回传数据速率
#define gyroBAUDRATE			((uint8_t) 0x04)		//串口波特率
#define gyroACC_AXISX_OFFSET	((uint8_t) 0x05)		//X轴加速度零偏
#define gyroACC_AXISY_OFFSET	((uint8_t) 0x06)		//y轴加速度零偏
#define gyroACC_AXISZ_OFFSET	((uint8_t) 0x07)		//z轴加速度零偏
#define gyroANGV_AXISX_OFFSET	((uint8_t) 0x08)		//x轴角速度零偏
#define gyroANGV_AXISY_OFFSET	((uint8_t) 0x09)		//y轴角速度零偏
#define gyroANGV_AXISZ_OFFSET	((uint8_t) 0x0A)		//z轴角速度零偏
#define gyroMAG_AXISX_OFFSET	((uint8_t) 0x0B)		//x轴磁场零偏
#define gyroMAG_AXISY_OFFSET	((uint8_t) 0x0C)		//y轴磁场零偏
#define gyroMAG_AXISZ_OFFSET	((uint8_t) 0x0D)		//z轴磁场零偏
#define gyroD0_MODE				((uint8_t) 0x0E)		//D0模式
#define gyroD1_MODE				((uint8_t) 0x0F)		//D1模式
#define gyroD2_MODE				((uint8_t) 0x10)		//D2模式
#define gyroD3_MODE				((uint8_t) 0x11)		//D3模式
#define gyroD0_PWM_WIDTH		((uint8_t) 0x12)		//D0PWM脉宽
#define gyroD1_PWM_WIDTH		((uint8_t) 0x13)		//D1PWM脉宽
#define gyroD2_PWM_WIDTH		((uint8_t) 0x14)		//D2PWM脉宽
#define gyroD3_PWM_WIDTH		((uint8_t) 0x15)		//D3PWM脉宽
#define gyroD0_PWM_PERIOD		((uint8_t) 0x16)		//D0PWM周期
#define gyroD1_PWM_PERIOD		((uint8_t) 0x17)		//D1PWM周期
#define gyroD2_PWM_PERIOD		((uint8_t) 0x18)		//D2PWM周期
#define gyroD3_PWM_PERIOD		((uint8_t) 0x19)		//D3PWM周期
#define gyroIIC_ADDR			((uint8_t) 0x1A)		//IIC地址
#define gyroLED_DISABLE			((uint8_t) 0x1B)		//关闭LED指示灯
#define gyroGPS_BAUDRATE		((uint8_t) 0x1C)		//GPS连接波特率

#define gyroYYMM				((uint8_t) 0x30)		//年、月
#define gyroDDHH				((uint8_t) 0x31)		//日、时
#define gyroMMSS				((uint8_t) 0x32)		//分、秒
#define gyroMS					((uint8_t) 0x33)		//毫秒
#define gyroACC_AXISX			((uint8_t) 0x34)		//x轴加速度
#define gyroACC_AXISY			((uint8_t) 0x35)		//y轴加速度
#define gyroACC_AXISZ			((uint8_t) 0x36)		//z轴加速度
#define gyroANGV_AXISX			((uint8_t) 0x37)		//x轴角速度
#define gyroANGV_AXISY			((uint8_t) 0x38)		//y轴角速度
#define gyroANGV_AXISZ			((uint8_t) 0x39)		//z轴角速度
#define gyroMAG_AXISX			((uint8_t) 0x3a)		//x轴磁场
#define gyroMAG_AXISY			((uint8_t) 0x3b)		//y轴磁场
#define gyroMAG_AXISZ			((uint8_t) 0x3c)		//z轴磁场
#define gyroROLL				((uint8_t) 0x3d)		//横滚角
#define gyroPITCH				((uint8_t) 0x3e)		//俯仰角
#define gyroYAW					((uint8_t) 0x3f)		//偏航角
#define gyroTEMPERATURE			((uint8_t) 0x40)		//模块温度
#define gyroD0_STATUS			((uint8_t) 0x41)		//端口D0状态
#define gyroD1_STATUS			((uint8_t) 0x42)		//端口D1状态
#define gyroD2_STATUS			((uint8_t) 0x43)		//端口D2状态
#define gyroD3_STATUS			((uint8_t) 0x44)		//端口D3状态
#define gyroPRESSURE_LBYTE		((uint8_t) 0x45)		//气压低字节
#define gyroPRESSURE_HBYTE		((uint8_t) 0x46)		//气压高字节
#define gyroHEIGHT_LBYTE		((uint8_t) 0x47)		//高度低字节
#define gyroHEIGHT_HBYTE		((uint8_t) 0x48)		//高度高字节
#define gyroLON_LBYTE			((uint8_t) 0x49)		//经度低字节
#define gyroLON_HBYTE			((uint8_t) 0x4a)		//经度高字节
#define gyroLAT_LBYTE			((uint8_t) 0x4b)		//纬度低字节
#define gyroLAT_HBYTE			((uint8_t) 0x4c)		//纬度低字节
#define gyroGPS_HEIGHT			((uint8_t) 0x4d)		//GPS高度
#define gyroGPS_YAW				((uint8_t) 0x4e)		//GPS航向角
#define gyroGPS_VEL_LBYTE		((uint8_t) 0x4f)		//GPS地速低字节
#define gyroGPS_VEL_HBYTE		((uint8_t) 0x50)		//GPS地速高字节
#define gyroQ0					((uint8_t) 0x51)		//四元数Q0
#define gyroQ1					((uint8_t) 0x52)		//四元数Q1
#define gyroQ2					((uint8_t) 0x53)		//四元数Q2
#define gyroQ3					((uint8_t) 0x54)		//四元数Q3
/* ---------------------------- -----------------------------------------*/




/* Private structs definations -----------------------------------------*/

//陀螺仪时间
struct GyroTime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};

//陀螺仪加速度
struct GyroAcc
{
	float x;
	float y;
	float z;
};

//陀螺仪角速度
struct GyroAngVel
{
	float x;
	float y;
	float z;
};

//陀螺仪磁场强度
struct GyroMag
{
	float roll;
	float pitch;
	float yaw;
};

//陀螺仪欧拉角
struct GyroAngle
{
	float roll;
	float pitch;
	float yaw;
//	uint16_t roll;
//	uint16_t pitch;
//	uint16_t yaw;
};

//陀螺仪端口状态
struct GyroPortStatus
{
	short sDStatus[4];
};

//陀螺仪GPS信息
struct GPSInfo
{
	float longitude;
	float latitude;
	float altitude;
	float yaw;
	float velocity;
	
};

//陀螺仪四元数
struct GyroQ
{ short q[4];
};

/* Private typedefs definations ----------------------------------------------*/
typedef struct GyroTime 		GyroTimeType;
typedef struct GyroAcc 			GyroAccType;
typedef struct GyroAngVel 		GyroAngVelType;
typedef struct GyroMag 			GyroMagType;
typedef struct GyroAngle 		GyroAngleType;
typedef struct GyroPortStatus 	GyroPortStatusType;
typedef struct GPSInfo 			GyroGPSInfoType;
typedef struct GyroQ 			GyroQType;

/* Private enum definations --------------------------------------------------*/
enum ePackType{
	PACK_ACC 		= 0x51,
	PACK_ANG_VEL 	= 0x52,
	PACK_ANG 		= 0x53,
	PACK_MAG 		= 0x54,
	
};

/* Private variables definations ---------------------------------------------*/

/* 陀螺仪初始化 --------------------------------------------------------------*/	
void GYRO_Init(void);

/* 陀螺仪寄存器操作 ----------------------------------------------------------*/	
uint8_t GYRO_RegSingleWrite( uint8_t ucRegAddr, uint16_t usSrc );
uint8_t GYRO_RegSingleRead(	uint8_t ucRegAddr, uint16_t* pusDst );
uint8_t GYRO_RegMultiWrite(	uint8_t ucRegAddr, uint8_t ucRegCount, 
							uint16_t* pusSrcBuf );
uint8_t GYRO_RegMultiRead( uint8_t ucRegAddr, uint8_t ucRegCount, 
							uint16_t* pusDstBuf );

/* 陀螺仪数据函获取函数 ------------------------------------------------------*/	
uint8_t	GYRO_GetCurrTime( GyroTime_t* );
uint8_t	GYRO_GetCurrAcc( GyroAcc_t* );
uint8_t	GYRO_GetCurrAngVel( GyroAngVel_t* );
uint8_t	GYRO_GetCurrMag( GyroMag_t* );
uint8_t	GYRO_GetCurrAng( GyroAngle_t* );
uint8_t	GYRO_GetCurrPortStatus( GyroPortStatus_t* );
uint8_t	GYRO_GetCurrGPSInfo( GyroGPSInfo_t* );
uint8_t	GYRO_GetCurrQ( GyroQ_t* );
int GYRO_GetImuData( uint8_t *ucpDst );

/* 陀螺仪参数设定函数 --------------------------------------------------------*/	
 
#endif
