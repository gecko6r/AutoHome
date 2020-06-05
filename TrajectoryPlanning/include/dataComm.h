/**
  ***********************************UTF-8**************************************
  * @file    dataComm.h
  * @author  Xiong
  * @version V1.0
  * @date    18-July-2020
  * @brief   此文件用于定义机器人通讯的数据缓冲区和读写函数，内部封装了环形数组
  ******************************************************************************  
  */ 

#ifndef _DATA_COMM_H
#define _DATA_COMM_H

#include "sys.h"
#include "gyro.h"

#define comMIN_PACK_SIZE		( ( uint16_t ) 10 )

#define comPACK_HEAD_1			( ( uint8_t ) 0xFF )
#define comPACK_HEAD_2			( ( uint8_t ) 0xFE )
#define comPACK_HEAD_3			( ( uint8_t ) 0xFD )


#define comBYTE_HEAD_1			0x00
#define comBYTE_HEAD_2			0x01
#define comBYTE_RESERVE			0x02
#define comBYTE_LEN_LOW			0x03
#define comBYTE_LEN_HIGH		0x04
#define comBYTE_INSTRUCTION		0x05
#define comBYTE_MSG_TYPE		0x06
#define comBYTE_PARA_START		0x07




typedef enum dataCommInst
{
	Inst_Robot_Reset = 0x00,
	Inst_Set_Torque,
	Inst_Set_SPEED,
	Inst_Set_COMP_Mode,
	Inst_Set_Step_Height,
	Inst_Set_Step_Len,
	Inst_Upload_Data,
	Inst_MANI,
	
}COMM_Inst_E;


typedef enum msgTpye
{
	MSG_Servo_Current = 0,
	MSG_Servo_Pos,
	MSG_Servo_Vel,
	MSG_Servo_Volt,
	MSG_IMU,
	MSG_SYS_Info,
	
}COMM_MSG_E;

//上下位机通讯服务寄存器，用于表示当前缓冲区存储的数据包
typedef struct CommRegBits
{
	char reserve			: 2;	//bit7:6, 保留位,必须置0
	char sysInfoInBuf		: 1;	//bit5， 表示系统信息数据是否在数组中
	char imuDataInBuf		: 1;	//bit4， 表示陀螺仪数据是否在数组中
	char servoVoltInBuf		: 1;	//bit3， 表示舵机电压数据是否在数组中
	char servoVelInBuf 		: 1;	//bit2， 表示舵机速度数据是否在数组中
	char servoPosInBuf 		: 1;	//bit1， 表示舵机位置数据是否在数组中
	char servoCurrentInBuf 	: 1;	//bit0， 表示舵机电流数据是否在数组中
}CommRegBits_t;

typedef struct BotStatusUpdated
{
	char reserved				: 2;
	char servoCurrentUpdated	: 1;
	char servoVoltUpdated		: 1;
	char servoPositionUpdated	: 1;
	char servoVelocityUpdated	: 1;
	char imuDataUpdated			: 1;
	char systemInfoUpdated		: 1;
}BotStatusUpdated_t;
	
typedef union ComReg
{
	CommRegBits_t tReg;
	uint8_t ucReg;
	
}ComReg_t;

typedef enum packRecvStatus
{
	PackHeadRecved,
	PackDataRecved,
	PackTotalRecved,
	PackRecvError,
}PaclRecvStatus_e;
	

uint8_t DC_Init( void );
uint8_t DC_SetBuffer(uint8_t *ucSrcBuf, uint8_t ucSize, COMM_MSG_E );
uint8_t DC_Send( void );
PaclRecvStatus_e DC_Recv( uint8_t size );
uint8_t DC_GetPackParam( void );
uint16_t CrcCheck( uint8_t* ucBuf, uint16_t usLen );







#endif
