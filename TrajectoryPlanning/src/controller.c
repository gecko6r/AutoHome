/**
  ***********************************UTF-8**************************************
  * @file    dataAcquisition.c
  * @author  Xiong
  * @version V1.0
  * @date    16-July-2020
  * @brief   此文件用于定义机器人运动控制函数，机器人的关节方向规定：关节上抬或
  *			 前伸或内收为正向
  ******************************************************************************  
  */ 
  
#include "controller.h"
#include "usart.h"
#include "math.h"
#include "dynamixel.h"

/* 机器人运动控制局部变量定义 ------------------------------------------------*/
uint32_t uxServoPosBuf[ctrlSERVO_NUM];


/* 机器人运动控制局部变量定义 ------------------------------------------------*/
/* 机器人类型及尺寸参数定义, 如果未定义，则定义为壁虎机器人*/
#if defined  GECKO_ROBOT
#undef HEXAPOD_ROBOT
Robot_t robotType = GeckoRobot;

/* 关节0距离身体中心的距离 mm */
static const double fJoint0_x = 38.75;
static const double fJoint0_y = 114.0;


/* 各关节连杆长度 mm */
static const double l1 = 48.0;
static const double l2 = 84.25;
static const double l3 = 29;
static const double l4 = 136;
static const double l5 = 29;
//static double stepLen;
//static double stepHeight;
//static double speed;

/* 脚掌初始位置数组（相对于身体中心,顺序左前右前左后右后，xyz，单位mm） */
static const double TipBasePosBuf[ ctrlLEG_COUNT ][ 4 ] = 
					{
						-l1 - l2 - l3 - fJoint0_x, 	l4 + fJoint0_y, 	-l5, 0,
						l1 + l2 + l3 + fJoint0_x, 	l4 + fJoint0_y, 	-l5, 0,
						-l1 - l2 - l3 - fJoint0_x, 	-l4 - fJoint0_y, 	-l5, 0,
						l1 + l2 + l3 + fJoint0_x, 	-l4 - fJoint0_y, 	-l5, 0,
						
					};

static int s_feetPosLimit[ 4 ][ 2 ] = 
					{
						2048, 4600,
						2048, 4000,
						2048, 4000,
						2048, 4000
					};

#elif defined HEXAPOD_ROBOT
RobotType_t RobotType = HexapodRobot;




#else
#error "No available robot type defined!"
#endif


/* 机器人运动控制局部变量定义 ------------------------------------------------*/
static TipPos_t* xTipPosBuf[ctrlLEG_COUNT];


//Set stall standing as initiate postion	
static const double ServoPosOffset[ctrlSERVO_NUM] = {
	2048, 2048, 2048, 512,
	2048, 2048, 2048, 512,
	2048, 2048, 2048, 512,
	2048, 2048, 2048, 512, 
};

static const double ServoPosDir[ctrlSERVO_NUM] = {
	1, 1, -1, 1,
	-1, -1, 1, 1,
	1, -1, 1, 1,
	-1, 1, -1, 1,
};

static const uint32_t ServoMaxPos = 4096;

/************************** 机器人运动控制全局变量声明 ************************/


/************************** 机器人运动控制全局函数定义 ************************/

/****
	* @brief	计算每条腿运动学逆解，输入的足端位置必须以身体中心为原点，坐标系
	*			为躯干坐标系，不同的机器人此函数需要被改写
	* @param  	xPointBuf：足端位置数组，按照腿的序号排列
	* @retval 	关节角度数组，（角度制）
	*/
uint8_t CTRL_WriteTipPosToBuf( TipPos_t* xTipPosBuf )
{
	LegAngle_t xLegAngleBuf[ctrlLEG_COUNT];
	double dAngleBuf[ctrlSERVO_NUM];
	
	/* 计算每条腿的运动学逆解 */
	CTRL_InverseKinemix( xTipPosBuf, xLegAngleBuf );
	/* 将关节角度结构体转储到浮点型数组中 */
	CTRL_LegAngTypeToDouble( xLegAngleBuf, ctrlLEG_COUNT, dAngleBuf );
	/* 根据每个关节的安装方向和中心位置，计算每个舵机的实际位置(0-360°->0-4096)*/
	CTRL_DoubleToPos( dAngleBuf, ctrlSERVO_NUM, uxServoPosBuf );
	
	return 0;
}
/* ---------------------------------------------------------------------------*/	

/****
	* @brief	移动足端到目标位置，设初始位置为0
	* @param  	xTipPosBuf：足端位置数组（大地坐标系）
	* @retval 	无
	*/
void CTRL_SetTipsPos( TipPos_t xTipPosBuf[ ctrlLEG_COUNT ] )
{
	int i;
	for( i = 0; i < ctrlLEG_COUNT; i++ )
	{
		xTipPosBuf[ i ].x += TipBasePosBuf[ i ][ 0 ];
		xTipPosBuf[ i ].y += TipBasePosBuf[ i ][ 1 ];
		xTipPosBuf[ i ].z += TipBasePosBuf[ i ][ 2 ];
	}
	
	CTRL_WriteTipPosToBuf( xTipPosBuf );
	DXL_SetAllGoalPos( uxServoPosBuf );

}
/* ---------------------------------------------------------------------------*/	

/****
	* @brief	设定脚掌卷起状态
	* @param  	pos: 0 - 255，表示卷起程度
	* @retval 	无
	*/
void CTRL_SetAdhesionPos( int pos )
{
	uint8_t idBuf[ 4 ] = { 4, 8, 12, 16 };
	uint32_t posBuf[ 4 ];
	uint8_t i;
	
	if( pos < 0 )
		pos = 0;
	if( pos > 255 )
		pos = 255;
	
	for( i = 0; i < 4; i++ )
	{
		posBuf[ i ] = ( pos * s_feetPosLimit[ i ][ 1 ] + 
							( 255 - pos ) * s_feetPosLimit[ i ][ 0 ] ) / 255;
	}
	
	DXL_RegSyncWrite( dxlREG_Goal_Position, 4, 4, posBuf, idBuf );
	
	
}

/****************************** 运动学逆解函数定义 ****************************/	
/****
	* @brief	计算每条腿运动学逆解，输入的足端位置必须以身体中心为原点，坐标系
	*			为躯干坐标系，不同的机器人此函数需要被改写
	* @param  	xPointBuf：足端位置数组，按照腿的序号排列
	* @retval 	关节角度数组，（角度制）
	*/
void CTRL_InverseKinemix( TipPos_t xTipPosBuf[ctrlLEG_COUNT], 
						  LegAngle_t* xDstBuf)
{
	uint8_t i = 0;
	Point3d xTmp;
	for( i=0; i<ctrlLEG_COUNT; i++ )
	{
		xTmp.x = pow( -1, i+1 ) * xTipPosBuf[i].x;
		xTmp.y = ((i >= (ctrlLEG_COUNT/2)) ? (-1) : 1) * xTipPosBuf[i].y;
		xTmp.z = xTipPosBuf[i].z;
		
		xDstBuf[i] = CTRL_SingleLegIK(xTmp);
	}
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	单腿运动学逆解计算，以右前腿为模型，由于每条腿的设计具有对称性且
	*			关节角度定义具有一致性，所以可以通过多次调用此函数完成所有腿的运
	*			动学逆解，但在调用时注意前后或左右的坐标抓换，都转换到右前腿模型
	*			此外，此函数会把牵线舵机的角度置为0
	* @param  	xPoint: 足端的三维坐标（按照身体坐标系,原点为机器人身体的中心, 
	*			单位mm）
	* @retval 	返回单腿三个关节的角度值（角度制）
	*/
LegAngle_t CTRL_SingleLegIK( TipPos_t xPoint )
{

	
	double x = xPoint.x - fJoint0_x;
	double y = xPoint.y - fJoint0_y;
	double z = xPoint.z;
	
	LegAngle_t xAngle;
	
	/* 运动学逆解计算 */
	double t3 = asin((-x*x - y*y - z*z - l1*l1 + 2.0*l1*sqrt(x*x+z*z-l5*l5) + l2*l2
		+ l3*l3 + l4*l4 + l5*l5) / (2.0*l2*sqrt(l3*l3+l4*l4))) + asin(l3 / sqrt(l3*l3+l4*l4));
	
	
	double t1 = acos(-l5 / sqrt((x*x + z*z))) + acos( -z / sqrt( x*x + z*z ));

    double exp = sqrt(pow((l4*cos(t3) + l3*sin(t3)), 2.0)
				+ pow((l2 + l3*cos(t3) - l4*sin(t3)), 2.0));
    double t2 = asin(y / exp) - asin((l4*cos(t3) + l3*sin(t3)) / exp);
	/* 弧度制转角度制 */
	xAngle.rootJoint 	= -180 + t1 * 180.0 / PI;
	xAngle.midJoint  	= t2 * 180.0 / PI;
	xAngle.endJoint 	= t3 * 180.0 / PI;
	xAngle.stringJoint 	= 0.0;
		
	return xAngle;
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	将腿部角度数组转换成双精度浮点数组
	* @param  	xBuf：腿部关节角度数组
	* @param	ucCount：腿部角度数组大小
	* @param	dDstBuf：目标存储数组
	* @retval 	无
	*/
void CTRL_LegAngTypeToDouble( LegAngle_t* xBuf, uint8_t ucCount, 
							  double* dDstBuf )
{
	uint8_t i = 0;
	for( i=0; i<ucCount; i++)
	{
		dDstBuf[i*ctrlJOINTS_PER_LEG + 0] = xBuf[i].rootJoint;
		dDstBuf[i*ctrlJOINTS_PER_LEG + 1] = xBuf[i].midJoint;
		dDstBuf[i*ctrlJOINTS_PER_LEG + 2] = xBuf[i].endJoint;
		dDstBuf[i*ctrlJOINTS_PER_LEG + 3] = xBuf[i].stringJoint;
	}
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	将各关节角度转换成舵机位置
	* @param  	dSrcBuf：浮点数组
	* @param	ucCount：数组大小
	* @param	uxDstBuf：目标数组
	* @retval 	无
	*/
void CTRL_DoubleToPos( double* dSrcBuf, uint8_t ucCount, uint32_t* uxDstBuf )
{
	uint8_t i = 0;
	
	for( i=0; i<ucCount; i++ ) 
		uxDstBuf[i] = ServoPosDir[i] * dSrcBuf[i] * ServoMaxPos / 360.0
						+ ServoPosOffset[i];
	
	
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	通过当前身体的RPY、身体中心位置和足端位置，计算运动学逆解，坐标
				系一律为接触表面坐标系。函数的计算过程是：在接触表面坐标系下，通
				过身体的中心以及身体的欧拉角计算出每条腿部根关节的位置，用对应足
				端位置减去每条腿根部关节的位置就可以得到足端相对于腿根部关节的位
				置，即前面CTRL_InverseKinemix的参数，计算此参数得到关节角度。
	* @param  	xBodyPose:机器人身体的欧拉角（角度制）
	* @param	xBodyCenter：身体中心位置（接触表面坐标系）
	* @param	xTipGroundPos：足端位置（接触表面坐标系）
	* @param	xLegAngleBuf： 保存计算结果的数组
	* @retval 	无
	*/
void CTRL_PoseBasedIK( BodyPose_t xBodyPose, Point3d xBodyCenter, 
					   Point3d* xTipGroundPos , LegAngle_t* xLegAngleBuf)
{
	
	
}
