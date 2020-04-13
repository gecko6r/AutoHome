/**
  ***********************************UTF-8***************************************
  * @file    dunamixel.h
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   This file contains all the functions prototypes for the Dynamixel 
  *          servo control
  ******************************************************************************  
  */ 

#ifndef _HARDWARE_DYNAMIXEL_H
#define _HARDWARE_DYNAMIXEL_H


#endif

#include "sys.h"


#define DATA_BUF_MAX_LEN			( 255 )	//max buffer size


#define LOW_BYTE(a)					((uint8_t)	(((uint16_t)(a)) &	0xff))
#define HIGH_BYTE(a)				((uint8_t)	(((uint16_t)(a)) >> 8))
#define LOW_16BIT(a)				((uint16_t)	(((uint32_t)(a)) &	0xffff))
#define HIGH_16BIT(a)				((uint16_t)	(((uint32_t)(a)) >> 16))
#define nthByteOf(data, n)			((uint8_t)	(((uint32_t)(data) >> (n*8)) & 0xff))
#define BYTE_ParamN(n)				((uint16_t) (7 + n))

#define SERVO_NUM					((uint8_t) 16 )		//number of servos connected
#define SYNC_OPT_START_ADDR			((uint8_t) 12 )		//in syncRead(Write) mode, the address of para5
#define ID_BROADCAST				((uint8_t) 0xfe )	//the ID when operates multiple servos

#define PARAM_ADD_LEN				((uint16_t) 2 )
#define PACKET_ADD_LEN				((uint16_t) 3 )
#define PACKET_LEFT_LEN				((uint16_t) 7 )

#define REG_1_BYTE					((uint8_t) 1 )
#define REG_2_BYTE					((uint8_t) 2 )
#define REG_3_BYTE					((uint8_t) 3 )
#define REG_4_BYTE					((uint8_t) 4 )


/* Servo Status Matrix Buffer Definition -----------------------------------------------------------*/
#define MIN_STATUS_PACK_LEN			((uint16_t) 11 )	//
#define Stasus_Param_Byte			((uint16_t) 9 )
#define STATUS_INST_BYTE			((uint16_t) 7)
#define Status_ID_Byte				((uint8_t) 4 )
#define SERVO_STATUS_MSG_LEN		((uint8_t) 24 )
#define SERVO_STATUS_NUM			((uint8_t) 5 )
#define COL_Pos				((uint8_t) 0 )
#define COL_Volt			((uint8_t) 1 )
#define COL_Current			((uint8_t) 2 )
#define COL_Velocity		((uint8_t) 3 )
#define COL_Tick			((uint8_t) 4 )


/* Servo Communication Macros Definition -----------------------------------------------------------*/

/*EEPROM Area*/
#define ADDR_Mode_Number			(uint16_t)( 0 )
#define ADDR_Model_Information		(uint16_t)( 2 )
#define ADDR_Firmware_version		(uint16_t)( 6 )
#define ADDR_ID						(uint16_t)( 7 )
#define ADDR_Baudrate				(uint16_t)( 8 )
#define ADDR_Return_Delay_us		(uint16_t)( 9 )
#define ADDR_Drive_Mode				(uint16_t)( 10 )
#define ADDR_Operating_Mode			(uint16_t)( 11 )
#define ADDR_SEC_ID					(uint16_t)( 12 )
#define ADDR_Protocol_type			(uint16_t)( 13 )
#define ADDR_Homing_Offset			(uint16_t)( 20 )
#define ADDR_Moving_Threshold		(uint16_t)( 24 )
#define ADDR_Temperature_Limit		(uint16_t)( 31 )
#define ADDR_Max_Volt_Limit			(uint16_t)( 32 )
#define ADDR_Min_Volt_Limit			(uint16_t)( 34 )
#define ADDR_PWM_Limit				(uint16_t)( 36 )
#define ADDR_Current_Limit			(uint16_t)( 38 )
#define ADDR_Velocity_Limit			(uint16_t)( 44 )
#define ADDR_Max_Pos_Limit			(uint16_t)( 48 )
#define ADDR_Min_Pos_Limit			(uint16_t)( 52 )
#define ADDR_Shutdown				(uint16_t)( 63 )

/*RAM Area*/
#define ADDR_Torque_Enable			(uint16_t)( 64 )
#define ADDR_LED					(uint16_t)( 65 )
#define ADDR_Status_Return_Lv		(uint16_t)( 68 )
#define ADDR_REG_Instruction		(uint16_t)( 69 )
#define ADDR_Hardware_ERR_Status	(uint16_t)( 70 )
#define ADDR_Vec_I_Gain				(uint16_t)( 76 )
#define ADDR_Vec_P_Gain				(uint16_t)( 78 )
#define ADDR_Pos_D_GAain			(uint16_t)( 80 )
#define ADDR_Pos_I_Gain				(uint16_t)( 82 )
#define ADDR_Pos_P_Gain				(uint16_t)( 84 )
#define ADDR_Feed_FW_Sec_Gain		(uint16_t)( 88 )
#define ADDR_Feed_FW_First_Gain		(uint16_t)( 90 )
#define ADDR_Bus_Watchdog			(uint16_t)( 98 )
#define ADDR_Goal_PWM				(uint16_t)( 100 )
#define ADDR_Goal_Current			(uint16_t)( 102 )
#define ADDR_Goal_Velocity			(uint16_t)( 104 )
#define ADDR_Profile_Acceleration	(uint16_t)( 108 )
#define ADDR_Profile_Velocity		(uint16_t)( 112 )
#define ADDR_Goal_Position			(uint16_t)( 116 )
#define ADDR_Realtime_Tick			(uint16_t)( 120 )
#define ADDR_Moving					(uint16_t)( 122 )
#define ADDR_Moving_Status			(uint16_t)( 123 )
#define ADDR_Present_PWM			(uint16_t)( 124 )
#define ADDR_Present_Current		(uint16_t)( 126 )
#define ADDR_Present_Velocity		(uint16_t)( 128 )
#define ADDR_Present_Position		(uint16_t)( 132 )
#define ADDR_Velocity_Tracjectory	(uint16_t)( 136 )
#define ADDR_Position_Tracjectory	(uint16_t)( 140 )
#define ADDR_Present_Voltage		(uint16_t)( 144 )
#define ADDR_Present_Temperature	(uint16_t)( 146 )

/* Instruction Definition */
#define INST_Ping					(uint8_t)( 0x01 )
#define INST_Read					(uint8_t)( 0x02 )
#define INST_Write					(uint8_t)( 0x03 )
#define INST_REG_Write				(uint8_t)( 0x04 )
#define INST_Action					(uint8_t)( 0x05 )
#define INST_Factory_Reset			(uint8_t)( 0x06 )
#define INST_Reboot					(uint8_t)( 0x08 )
#define INST_Clear					(uint8_t)( 0x10 )
#define INST_Status					(uint8_t)( 0x55 )
#define INST_Sync_Read				(uint8_t)( 0x82 )
#define INST_Sync_Write				(uint8_t)( 0x83 )
#define INST_Bulk_Read				(uint8_t)( 0x92 )
#define INST_Bulk_Write				(uint8_t)( 0x93 )

/* Servo Error */
#define	RESULT_FAIL					(uint8_t)( 0x01 )	//Failed to process the sent Instruction pack
#define INSTRUCTION_ERROR			(uint8_t)( 0x02 )	//Undefined Instruction used || Action used without Reg Write
#define	CRC_ERROR					(uint8_t)( 0x03 )	//CRC of sent packet does not match
#define	DATA_RANGE_ERROR			(uint8_t)( 0x04 )	//Reg addr to write is out of range (mix-max)
#define	DATA_LENGTH_ERROR			(uint8_t)( 0x05 )	//Attempt to write data which is shorter than reg len
#define	DATA_LIMIT_ERROR			(uint8_t)( 0x06 )	//Data to write is out of reg value range
#define	ACCESS_ERROR				(uint8_t)( 0x07 )	//Attempt to write in a ReadOnly reg
														//||Attempt to read from a WriteOnly reg
														//||Attempt to write in EEPROM while servo Torque Enable is On														
/*-----------------------------------------------------------------------------------------------------*/

//byte info of Instruction packet 
enum EPackByte{  
	eBYTE_HEADER_1, 
	eBYTE_HEADER_2,  
	eBYTE_HRADER_3, 
	eBYTE_RESV, 
	eBYTE_ID, 
	eBYTE_LEN_L, 
	eBYTE_LEN_H, 
	eBYTE_INST,

	//instruction packet only
	eBYTE_PARA_1, 
	eBYTE_PARA_2, 
	eBYTE_PARA_3, 
	eBYTE_PARA_4,

	//status packet only
	eBYTE_ERR = 0x08,
};

/*Servo baudrate */
typedef enum{
	eBD9600,
	eBD57600,
	eBD115200,
	eBD1M,
	eBD2M,
	eBD3M,
	eBD4M,
	eBD4M5,
}EDxlBaudrate;

/*Servo status return level */
typedef enum{
	ePingInstOnly,		//不返回任何指令
	ePingReadInst,		//返回读操作指令
	eAllInst,			//返回所有操作指令
}EStatusReturnLv;

/*Buffer printing format */
typedef enum{
	eDec3,
	eDec4,
	eHex2,
	eHex02,
	eChar,
}EPrintFormat;

/*Dynamixel Servo message struct */
typedef struct _stServoMsg_{
	FunctionalState fWriteEnable;
	bool bDataReady;
	uint8_t* pDataBuf;
	uint8_t ucByteRecved;
	uint16_t usRegAddr;
	uint16_t usRegSize;
	bool err;
}ServoMsg;	
	

/* Global Vaiables-----------------------------------------------------------*/
extern float fServoStatusBuf[SERVO_STATUS_NUM][SERVO_NUM];
extern ServoMsg xServoMsg;

/* Functions Definition -----------------------------------------------------------*/
void DXL_ServoInit(EDxlBaudrate eBaudrate, FunctionalState fTorqueState);

void DXL_RegWrite(uint16_t usRegAddr, uint16_t usRegSize, 
					uint32_t ulData, uint8_t ucId);
void DXL_RegSyncWrite(uint16_t usRegAddr, uint16_t usRegSize, 
						uint8_t ucServoNum, uint32_t* ulDataBuf, uint8_t* ucIdBuf);
void DXL_RegRead(uint16_t usRegAddr, uint16_t usRegSize, uint8_t ucId);
void DXL_RegSyncRead(uint16_t usRegAddr, uint16_t usRegSize, 
						uint8_t ucServoNum, uint8_t* ucIdBuf);
void DXL_SetPacketReadEnable(ServoMsg* pServoMsg, FunctionalState fWriteEnable);
uint8_t DXL_GetPresentParam(ServoMsg* pServoMsg);

/* EEPROM Registers Write Functions Definition ----------------------------------------*/
void DXL_SetAllBaudrate(EDxlBaudrate eBaudrate);
void DXL_SetAllReturnLv(EStatusReturnLv eReturnLevel);

/* RAM Registers Write Functions Definition ----------------------------------------*/
void DXL_SetAllTorque(FunctionalState fTorqueState);
void DXL_SetAllGoalPos(uint32_t* ulGoalPosBuf);
void DXL_SetAllLedState(FunctionalState fLedState);

/* Servo Registers Read Functions Definition ----------------------------------------*/
void DXL_GetRegState(uint16_t usRegAddr, uint16_t usRegSize);
void DXL_GetAllLedState(void);

/* Dynamixel Servo Message Functions Definition ----------------------------------------*/
ServoMsg xCreateMsg(uint8_t* ucSrcBuf);
void xClearMsg(ServoMsg* pMsg);
bool isMsgWriteEnable(ServoMsg xMsg);
bool isMsgDataReady(ServoMsg xMsg);

/*Debug Functions Definition ------------------------------------------------*/
#ifdef IN_DEBUG_MODE
uint16_t DXL_CrcCheck(uint8_t* ucBuf, uint8_t ucLen);
void DXL_PrintStatusBuf(void);
void void DXL_PrintBuf(uint8_t* ucBuf, uint8_t ucLen, EPrintFormat ePrintFormat);											   
#endif 
/* ---------------------------------------------------------------------------*/
