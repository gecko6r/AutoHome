/**
  ***********************************UTF-8**************************************
  * @file    dunamixel.h
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于定义Dynamix舵机的寄存器地址以及通讯用的结构和函数等
  ******************************************************************************  
  */ 

#ifndef _HARDWARE_DYNAMIXEL_H
#define _HARDWARE_DYNAMIXEL_H


#include "dataComm.h"
#include "controller.h"
#include "sys.h"

#define LOW_BYTE(a)				    ((uint8_t)	(((uint16_t)(a)) &	0xff))
#define HIGH_BYTE(a)				((uint8_t)	(((uint16_t)(a)) >> 8))
#define LOW_16BIT(a)				((uint16_t)	(((uint32_t)(a)) &	0xffff))
#define HIGH_16BIT(a)				((uint16_t)	(((uint32_t)(a)) >> 16))
#define nthByteOf(data, n)			((uint8_t)	(((uint32_t)(data) >> (n*8)) & 0xff))
#define BYTE_ParamN(n)				((uint16_t) (7 + n))

#define SYNC_OPT_START_ADDR			((uint8_t) 12 )		//in syncRead(Write) mode, the address of para5
#define ID_BROADCAST				((uint8_t) 0xfe )	//the ID when operates multiple servos
#define dxlPing						0xff

#define PARAM_ADD_LEN				((uint16_t) 2 )
#define PACKET_ADD_LEN				((uint16_t) 3 )
#define PACKET_LEFT_LEN				((uint16_t) 7 )

#define REG_1_BYTE					((uint8_t) 1 )
#define REG_2_BYTE					((uint8_t) 2 )
#define REG_3_BYTE					((uint8_t) 3 )
#define REG_4_BYTE					((uint8_t) 4 )


/* Servo Status Matrix Buffer Definition -------------------------------------*/
#define dxlEnableStatusCRC			0
#define dxlMIN_STATUS_PACK_LEN		( ( uint8_t ) 11 )	//
#define dxlStatusPackIdByte			( ( uint8_t ) 4 )
#define dxlStatusPackLenLowByte		( ( uint8_t ) 5 )
#define dxlStatusPackLenHighByte	( ( uint8_t ) 6 )
#define dxlStatusPackInstByte		( ( uint8_t ) 7 )
#define dxlStatusPackErrorByte		( ( uint8_t ) 8 )
#define dxlStatusPackParamByte		( ( uint8_t ) 9 )

#define dxlStatusCount				( ( uint8_t ) 5 )
#define dxlCOL_Pos					( ( uint8_t ) 0 )
#define dxlCOL_Volt					( ( uint8_t ) 1 )
#define dxlCOL_Current				( ( uint8_t ) 2 )
#define dxlCOL_Velocity				( ( uint8_t ) 3 )
#define dxlCOL_Tick					( ( uint8_t ) 4 )


/* Servo Communication Macros Definition -------------------------------------*/

/*EEPROM Area*/
#define dxlREG_Mode_Number			( ( uint16_t ) 0 )
#define dxlREG_Model_Information	( ( uint16_t ) 2 )
#define dxlREG_Firmware_version		( ( uint16_t ) 6 )
#define dxlREG_ID					( ( uint16_t ) 7 )
#define dxlREG_Baudrate				( ( uint16_t ) 8 )
#define dxlREG_Return_Delay_Us		( ( uint16_t ) 9 )
#define dxlREG_Drive_Mode			( ( uint16_t ) 10 )
#define dxlREG_Operating_Mode		( ( uint16_t ) 11 )
#define dxlREG_SEC_ID				( ( uint16_t ) 12 )
#define dxlREG_Protocol_type		( ( uint16_t ) 13 )
#define dxlREG_Homing_Offset		( ( uint16_t ) 20 )
#define dxlREG_Moving_Threshold		( ( uint16_t ) 24 )
#define dxlREG_Temperature_Limit	( ( uint16_t ) 31 )
#define dxlREG_Max_Volt_Limit		( ( uint16_t ) 32 )
#define dxlREG_Min_Volt_Limit		( ( uint16_t ) 34 )
#define dxlREG_PWM_Limit			( ( uint16_t ) 36 )
#define dxlREG_Current_Limit		( ( uint16_t ) 38 )
#define dxlREG_Velocity_Limit		( ( uint16_t ) 44 )
#define dxlREG_Max_Pos_Limit		( ( uint16_t ) 48 )
#define dxlREG_Min_Pos_Limit		( ( uint16_t ) 52 )
#define dxlREG_Shutdown				( ( uint16_t ) 63 )

/*RAM Area*/
#define dxlREG_Torque_Enable		( ( uint16_t ) 64 )
#define dxlREG_LED					( ( uint16_t ) 65 )
#define dxlREG_Status_Return_Lv		( ( uint16_t ) 68 )
#define dxlREG_REG_Instruction		( ( uint16_t ) 69 )
#define dxlREG_Hardware_ERR_Status	( ( uint16_t ) 70 )
#define dxlREG_Vec_I_Gain			( ( uint16_t ) 76 )
#define dxlREG_Vec_P_Gain			( ( uint16_t ) 78 )
#define dxlREG_Pos_D_GAain			( ( uint16_t ) 80 )
#define dxlREG_Pos_I_Gain			( ( uint16_t ) 82 )
#define dxlREG_Pos_P_Gain			( ( uint16_t ) 84 )
#define dxlREG_Feed_FW_Sec_Gain		( ( uint16_t ) 88 )
#define dxlREG_Feed_FW_First_Gain	( ( uint16_t ) 90 )
#define dxlREG_Bus_Watchdog			( ( uint16_t ) 98 )
#define dxlREG_Goal_PWM				( ( uint16_t ) 100 )
#define dxlREG_Goal_Current			( ( uint16_t ) 102 )
#define dxlREG_Goal_Velocity		( ( uint16_t ) 104 )
#define dxlREG_Profile_Acceleration	( ( uint16_t ) 108 )
#define dxlREG_Profile_Velocity		( ( uint16_t ) 112 )
#define dxlREG_Goal_Position		( ( uint16_t ) 116 )
#define dxlREG_Realtime_Tick		( ( uint16_t ) 120 )
#define dxlREG_Moving				( ( uint16_t ) 122 )
#define dxlREG_Moving_Status		( ( uint16_t ) 123 )
#define dxlREG_Present_PWM			( ( uint16_t ) 124 )
#define dxlREG_Present_Current		( ( uint16_t ) 126 )
#define dxlREG_Present_Velocity		( ( uint16_t ) 128 )
#define dxlREG_Present_Position		( ( uint16_t ) 132 )
#define dxlREG_Velocity_Tracjectory	( ( uint16_t ) 136 )
#define dxlREG_Position_Tracjectory	( ( uint16_t ) 140 )
#define dxlREG_Present_Voltage		( ( uint16_t ) 144 )
#define dxlREG_Present_Temperature	( ( uint16_t ) 146 )

/* Instruction Definition */
#define dxlINST_Ping				( ( uint8_t ) 0x01 )
#define dxlINST_Read				( ( uint8_t ) 0x02 )
#define dxlINST_Write				( ( uint8_t ) 0x03 )
#define dxlINST_REG_Write			( ( uint8_t ) 0x04 )
#define dxlINST_Action				( ( uint8_t ) 0x05 )
#define dxlINST_Factory_Reset		( ( uint8_t ) 0x06 )
#define dxlINST_Reboot				( ( uint8_t ) 0x08 )
#define dxlINST_Clear				( ( uint8_t ) 0x10 )
#define dxlINST_Status				( ( uint8_t ) 0x55 )
#define dxlINST_Sync_Read			( ( uint8_t ) 0x82 )
#define dxlINST_Sync_Write			( ( uint8_t ) 0x83 )
#define dxlINST_Bulk_Read			( ( uint8_t ) 0x92 )
#define dxlINST_Bulk_Write			( ( uint8_t ) 0x93 )

/* Servo Error ---------------------------------------------------------------*/
/*Failed to process the sent Instruction pack*/
#define	dxlRESULT_FAIL					( ( uint8_t ) 0x01 )	
//Undefined Instruction used || Action used without Reg Write
#define dxlINSTRUCTION_ERROR			( ( uint8_t ) 0x02 )
//CRC of sent packet does not match
#define	dxldxlCRC_ERROR					( ( uint8_t ) 0x03 )	
//Reg addr to write is out of range (mix-max)
#define	dxlDATA_RANGE_ERROR			( ( uint8_t ) 0x04 )	
//Attempt to write data which is shorter than reg len
#define	dxlDATA_LENGTH_ERROR			( ( uint8_t ) 0x05 )	
//Data to write is out of reg value range
#define	dxlDATA_LIMIT_ERROR			( ( uint8_t ) 0x06 )	
//Attempt to write in a ReadOnly reg
//||Attempt to read from a WriteOnly reg
//||Attempt to write in EEPROM while servo Torque Enable is On
#define	dxlACCESS_ERROR				( ( uint8_t ) 0x07 )	
														
																												
/*----------------------------------------------------------------------------*/

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
	eBD9600 	= 0,
	eBD57600	= 1,
	eBD115200	= 2,
	eBD1M		= 3,
	eBD2M		= 4,
	eBD3M		= 5,
	eBD4M		= 6,
	eBD4M5		= 7,
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
	FunctionalState fReadEnable;
	volatile bool bDataReady;
	uint8_t* pDataBuf;
	volatile uint8_t ucByteRecved;
	uint8_t byteToRecv;
	uint16_t usRegAddr;
	uint16_t usRegSize;
	bool err;
}ServoMsg;	
	

/* 全局变量定义 */
extern uint8_t g_ucaServoTxBuffer[ HALF_BUF_SIZE ];
extern uint8_t g_ucaServoRxBuffer[ HALF_BUF_SIZE ];
extern double dServoStatusBuf[ dxlStatusCount ][ ctrlSERVO_NUM ];
extern ServoMsg xServoMsg;
extern uint8_t g_ucaDataRecved[ ctrlSERVO_NUM ];

/* Functions Definition ------------------------------------------------------*/
int DXL_ServoInit(EDxlBaudrate eBaudrate, FunctionalState fTorqueState);

void DXL_RegWrite(uint16_t usRegAddr, uint16_t usRegSize, 
					uint32_t ulData, uint8_t ucId);
void DXL_RegSyncWrite(uint16_t usRegAddr, uint16_t usRegSize, 
					uint8_t ucServoNum, uint32_t* ulDataBuf, uint8_t* ucIdBuf);
int DXL_RegRead(uint16_t usRegAddr, uint16_t usRegSize, uint8_t ucId, uint32_t *pDst);
void DXL_RegSyncRead(uint16_t usRegAddr, uint16_t usRegSize, 
						uint8_t ucServoNum, uint8_t* ucIdBuf);
void DXL_SetPacketReadEnable(ServoMsg* pServoMsg, FunctionalState fWriteEnable);
int DXL_GetPresentParam(ServoMsg* pServoMsg);
static int DXL_FindFirstPackhead( uint8_t * pucBuf, uint8_t len );

///* Servo communication buffer operating functions Definition ---------------*/
//void DXL_SetTxBuffer(uint8_t* pucBuf, uint16_t usRegAddr, uint16_t usRegSize, 
//					uint8_t ucServoNum, uint32_t* ulDataBuf, uint8_t* ucIdBuf);

//void DXL_PushBuffer(void);

/* EEPROM Registers Write Functions Definition -------------------------------*/
void DXL_SetAllBaudrate(EDxlBaudrate eBaudrate);
void DXL_SetAllReturnLv(EStatusReturnLv eReturnLevel);
void DXL_SetAllReturnDelay( uint16_t usReturnDelayUs );

/* RAM Registers Write Functions Definition ----------------------------------*/
void DXL_SetAllTorque(FunctionalState fTorqueState);
void DXL_SetAllGoalPos(uint32_t* ulGoalPosBuf);
void DXL_SetAllLedState(FunctionalState fLedState);

/* Servo Registers Read Functions Definition ---------------------------------*/
void DXL_GetRegState(uint16_t usRegAddr, uint16_t usRegSize);
void DXL_GetAllLedState(void);
int DXL_Ping( void );
int DXL_SetComBuf( COMM_MSG_E msgType );

/* Dynamixel Servo Message Functions Definition ------------------------------*/
ServoMsg xCreateMsg(uint8_t* ucSrcBuf);
void xClearMsg(ServoMsg* pMsg);
bool isMsgWriteEnable(ServoMsg xMsg);
bool isMsgDataReady(ServoMsg xMsg);

/*Debug Functions Definition ------------------------------------------------*/
void DXL_PrintStatusBuf(void);
void DXL_PrintBuf(uint8_t* ucBuf, uint8_t ucLen, EPrintFormat ePrintFormat);											   

/* ---------------------------------------------------------------------------*/
#endif
