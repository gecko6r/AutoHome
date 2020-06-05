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
#define gyroREG_SAVE				((uint8_t) 0x00)		//保存当前配置
#define gyroREG_CALIB				((uint8_t) 0x01)		//校准
#define gyroREG_RSW					((uint8_t) 0x02)		//回传数据内容
#define gyroREG_RATE				((uint8_t) 0x03)		//回传数据速率
#define gyroREG_BAUDRATE			((uint8_t) 0x04)		//串口波特率
#define gyroREG_ACC_AXISX_OFFSET	((uint8_t) 0x05)		//X轴加速度零偏
#define gyroREG_ACC_AXISY_OFFSET	((uint8_t) 0x06)		//y轴加速度零偏
#define gyroREG_ACC_AXISZ_OFFSET	((uint8_t) 0x07)		//z轴加速度零偏
#define gyroREG_ANGV_AXISX_OFFSET	((uint8_t) 0x08)		//x轴角速度零偏
#define gyroREG_ANGV_AXISY_OFFSET	((uint8_t) 0x09)		//y轴角速度零偏
#define gyroREG_ANGV_AXISZ_OFFSET	((uint8_t) 0x0A)		//z轴角速度零偏
#define gyroREG_MAG_AXISX_OFFSET	((uint8_t) 0x0B)		//x轴磁场零偏
#define gyroREG_MAG_AXISY_OFFSET	((uint8_t) 0x0C)		//y轴磁场零偏
#define gyroREG_MAG_AXISZ_OFFSET	((uint8_t) 0x0D)		//z轴磁场零偏
#define gyroREG_D0_MODE				((uint8_t) 0x0E)		//D0模式
#define gyroREG_D1_MODE				((uint8_t) 0x0F)		//D1模式
#define gyroREG_D2_MODE				((uint8_t) 0x10)		//D2模式
#define gyroREG_D3_MODE				((uint8_t) 0x11)		//D3模式
#define gyroREG_D0_PWM_WIDTH		((uint8_t) 0x12)		//D0PWM脉宽
#define gyroREG_D1_PWM_WIDTH		((uint8_t) 0x13)		//D1PWM脉宽
#define gyroREG_D2_PWM_WIDTH		((uint8_t) 0x14)		//D2PWM脉宽
#define gyroREG_D3_PWM_WIDTH		((uint8_t) 0x15)		//D3PWM脉宽
#define gyroREG_D0_PWM_PERIOD		((uint8_t) 0x16)		//D0PWM周期
#define gyroREG_D1_PWM_PERIOD		((uint8_t) 0x17)		//D1PWM周期
#define gyroREG_D2_PWM_PERIOD		((uint8_t) 0x18)		//D2PWM周期
#define gyroREG_D3_PWM_PERIOD		((uint8_t) 0x19)		//D3PWM周期
#define gyroREG_IIC_ADDR			((uint8_t) 0x1A)		//IIC地址
#define gyroREG_LED_DISABLE			((uint8_t) 0x1B)		//关闭LED指示灯
#define gyroREG_GPS_BAUDRATE		((uint8_t) 0x1C)		//GPS连接波特率

#define gyroREG_YYMM				((uint8_t) 0x30)		//年、月
#define gyroREG_DDHH				((uint8_t) 0x31)		//日、时
#define gyroREG_MMSS				((uint8_t) 0x32)		//分、秒
#define gyroREG_MS					((uint8_t) 0x33)		//毫秒
#define gyroREG_ACC_AXISX			((uint8_t) 0x34)		//x轴加速度
#define gyroREG_ACC_AXISY			((uint8_t) 0x35)		//y轴加速度
#define gyroREG_ACC_AXISZ			((uint8_t) 0x36)		//z轴加速度
#define gyroREG_ANGV_AXISX			((uint8_t) 0x37)		//x轴角速度
#define gyroREG_ANGV_AXISY			((uint8_t) 0x38)		//y轴角速度
#define gyroREG_ANGV_AXISZ			((uint8_t) 0x39)		//z轴角速度
#define gyroREG_MAG_AXISX			((uint8_t) 0x3a)		//x轴磁场
#define gyroREG_MAG_AXISY			((uint8_t) 0x3b)		//y轴磁场
#define gyroREG_MAG_AXISZ			((uint8_t) 0x3c)		//z轴磁场
#define gyroREG_ROLL				((uint8_t) 0x3d)		//横滚角
#define gyroREG_PITCH				((uint8_t) 0x3e)		//俯仰角
#define gyroREG_YAW					((uint8_t) 0x3f)		//偏航角
#define gyroREG_TEMPERATURE			((uint8_t) 0x40)		//模块温度
#define gyroREG_D0_STATUS			((uint8_t) 0x41)		//端口D0状态
#define gyroREG_D1_STATUS			((uint8_t) 0x42)		//端口D1状态
#define gyroREG_D2_STATUS			((uint8_t) 0x43)		//端口D2状态
#define gyroREG_D3_STATUS			((uint8_t) 0x44)		//端口D3状态
#define gyroREG_PRESSURE_LBYTE		((uint8_t) 0x45)		//气压低字节
#define gyroREG_PRESSURE_HBYTE		((uint8_t) 0x46)		//气压高字节
#define gyroREG_HEIGHT_LBYTE		((uint8_t) 0x47)		//高度低字节
#define gyroREG_HEIGHT_HBYTE		((uint8_t) 0x48)		//高度高字节
#define gyroREG_LON_LBYTE			((uint8_t) 0x49)		//经度低字节
#define gyroREG_LON_HBYTE			((uint8_t) 0x4a)		//经度高字节
#define gyroREG_LAT_LBYTE			((uint8_t) 0x4b)		//纬度低字节
#define gyroREG_LAT_HBYTE			((uint8_t) 0x4c)		//纬度低字节
#define gyroREG_GPS_HEIGHT			((uint8_t) 0x4d)		//GPS高度
#define gyroREG_GPS_YAW				((uint8_t) 0x4e)		//GPS航向角
#define gyroREG_GPS_VEL_LBYTE		((uint8_t) 0x4f)		//GPS地速低字节
#define gyroREG_GPS_VEL_HBYTE		((uint8_t) 0x50)		//GPS地速高字节
#define gyroREG_Q0					((uint8_t) 0x51)		//四元数Q0
#define gyroREG_Q1					((uint8_t) 0x52)		//四元数Q1
#define gyroREG_Q2					((uint8_t) 0x53)		//四元数Q2
#define gyroREG_Q3					((uint8_t) 0x54)		//四元数Q3
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
uint8_t GYRO_WriteReg( uint8_t ucRegAddr, uint16_t usSrc );
uint8_t GYRO_ReadReg(	uint8_t ucRegAddr, uint16_t* pusDst );
uint8_t GYRO_WriteBuf(	uint8_t ucRegAddr, uint8_t ucRegCount, 
							uint16_t* pusSrcBuf );
uint8_t GYRO_ReadBuf( uint8_t ucRegAddr, uint8_t ucRegCount, 
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
uint8_t GYRO_GetImuData( void );

int GYRO_SetBuffer( void );

/* 陀螺仪参数设定函数 --------------------------------------------------------*/	
 
#endif
