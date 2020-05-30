//#include "servo.h"
//#include "usart.h"
//#include "delay.h"

//const char Servo_Err_Info[][255] = {
//		"No error!",
//		"Servo position is out of range!",
//		"Failed to initiate robot, please check user inputs.",
//	

//};

//ERROR_LIST _ERROR = 0;
//const u8 SERVO_NUM = 12;
//const double DGR2POS = 11.3777777777;
//const u8 INTER_LEG_DELAY_MS = 3;

//u8 ctrl_mode = MODE_CRAWLING;
//u16 pwm[12] = {1023};




///************************************************约定**************************************************/
////腿的编号：1号->左前腿, 2号->右前腿, 3号->左后腿, 4号->右后腿；
////关节编号：1号->肩关节, 2号->肘关节, 3号->腕关节；
////舵机方向：舵机顺时针为正，关节正向：关节1上抬，关节2前伸，关节3内合
////位置映射：90°<->1023, 0°<->0, 180°<->2047
///************************************************常量***************************************************/

////关节位置修正：用于修正舵机舵盘没有对中装配导致的误差(单位pos)
//const int SERVO_POS_CRC[12] = {0, 0, 0,			//左前腿[关节1， 关节2， 关节3]
//							   0, 0, 0, 		//右前腿[关节1， 关节2， 关节3]
//							   0, 0, 0, 		//左后腿[关节1， 关节2， 关节3]
//							   0, 0, 0};		//右后腿[关节1， 关节2， 关节3]

////关节位置死区：用于补偿传动轴配合间隙导致的误差()
//const u16 SERVO_DEAD_ZONE[12] = {0, 0, 0,		
//								 0, 0, 0, 		
//								 0, 0, 0, 		
//								 0, 0, 0};		

////关节位置限幅：用于避免舵机运动超过极限位置
//const u16 SERVO_POS_RANGE[12][2] = {250, 1776, 250, 1776, 250, 1776,
//								250, 1776, 250, 1776, 250, 1776,
//								250, 1776, 250, 1776, 250, 1776,
//								259, 1795, 258, 1794, 240, 1776};	
//								 
////关节方向
//const int SERVO_DIR[12] = {-1, -1, 1,		
//						   1, 1, -1, 		
//						   1, 1, -1, 		
//						   -1, -1, 1};		
//								   
///************************************************初始角度***************************************************/
//const double INIT_ANGLE[2][12] = {
//										{90.0, 90.0, 90.0,		
//										90.0, 90.0, 90.0, 		
//										90.0, 90.0, 90.0, 		
//										90.0, 90.0, 90.0},		
//										
//										{90.0, 60.0, 120.0,		
//										90.0, 60.0, 120.0, 		
//										90.0, 60.0, 120.0,		
//										90.0, 60.0, 120.0},
//};
///*******************************************************************************************************
//*功    能：初始化定时器及PWM
//*参    数：无
//*返    回：无
//*******************************************************************************************************/
//void pwm_init(void)
//{
//	/*
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	
//	
//	//RCC config
//	RCC_CONFIG_TIM1;
//	RCC_IO_CONFIG_TIM1;
//	
//	RCC_CONFIG_TIM2;
//	RCC_IO_CONFIG_TIM2;
//	
//	RCC_CONFIG_TIM3;
//	RCC_IO_CONFIG_TIM3;
//	
//	RCC_CONFIG_TIM4;
//	RCC_IO_CONFIG_TIM4;
//	
//	//GPIO alternative func config
//	PIN_AF_CONFIG_TIM1_CH1;
//	PIN_AF_CONFIG_TIM1_CH2;
//	PIN_AF_CONFIG_TIM1_CH3;
//	
//	PIN_AF_CONFIG_TIM2_CH1;
//	PIN_AF_CONFIG_TIM2_CH2;
//	PIN_AF_CONFIG_TIM2_CH3;
//	
//	PIN_AF_CONFIG_TIM3_CH1;
//	PIN_AF_CONFIG_TIM3_CH2;
//	PIN_AF_CONFIG_TIM3_CH3;
//	
//	PIN_AF_CONFIG_TIM4_CH1;
//	PIN_AF_CONFIG_TIM4_CH2;
//	PIN_AF_CONFIG_TIM4_CH3;
//	
//	//GPIO pin config
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
//	
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM1_CH1;              
//	GPIO_Init(GPIO_TIM1,&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM1_CH2;              
//	GPIO_Init(GPIO_TIM1,&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM1_CH3;              
//	GPIO_Init(GPIO_TIM1,&GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM2_CH1;              
//	GPIO_Init(GPIO_TIM2,&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM2_CH2;              
//	GPIO_Init(GPIO_TIM2,&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM2_CH3;              
//	GPIO_Init(GPIO_TIM2,&GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM3_CH1;              
//	GPIO_Init(GPIO_TIM3,&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM3_CH2;              
//	GPIO_Init(GPIO_TIM3,&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM3_CH3;              
//	GPIO_Init(GPIO_TIM3,&GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM4_CH1;              
//	GPIO_Init(GPIO_TIM4,&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM4_CH2;              
//	GPIO_Init(GPIO_TIM4,&GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = PIN_TIM4_CH3;              
//	GPIO_Init(GPIO_TIM4,&GPIO_InitStructure);
//	
//	//Timer init config
//	TIM_TimeBaseStructure.TIM_Prescaler=83;
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period=19999;
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
//	
//	TIM_TimeBaseStructure.TIM_Prescaler=167;
//	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//	
//	//Timer OC config
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//	
//	
//	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
//	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
//	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
//	
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
//	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
//	TIM_OC2Init(TIM4, &TIM_OCInitStructure);

//	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
//	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
//	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
//	
//	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
//	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	
//	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	
//	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
//	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
//	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	
//	//Timer enable
//	TIM_ARRPreloadConfig(TIM1,ENABLE);
//	TIM_Cmd(TIM1, ENABLE);
//	
//	TIM_ARRPreloadConfig(TIM2,ENABLE);
//	TIM_Cmd(TIM2, ENABLE);
//	
//	TIM_ARRPreloadConfig(TIM3,ENABLE);
//	TIM_Cmd(TIM3, ENABLE);
//	
//	TIM_ARRPreloadConfig(TIM4,ENABLE);
//	TIM_Cmd(TIM4, ENABLE);
//	
//	//Advanced timer enable
//	TIM_CtrlPWMOutputs(TIM1, ENABLE);
//	
//	*/
//}
//				
///*******************************************************************************************************
//*功    能：初始化定时器及PWM，使机器人运动到初始状态
//*参    数：mode：初始状态选择
//*返    回：无
//*******************************************************************************************************/
//void servo_init(LOCO_MODE mode)
//{
//	double _buf[12] = {0.0};
//	u8 i;
//	ctrl_mode = mode;
//	pwm_init();
//	for(i=0; i<12; i++)
//		_buf[i] = 0.0;
//	servo_ctrl_degree(_buf);
//}

///*******************************************************************************************************
//*功    能：舵机控制，直接作用于舵机的唯一函数
//*参    数：_pos：12个关节的位置数组，范围（0， 2047）
//*返    回：0：没有错误；others：有错误，调用errhandle（）来查看错误
//*******************************************************************************************************/
//int servo_ctrl(u16* _pos)
//{
//	
//	u16 _pwm[12] = {0};

//	u8 i = 0;
//	CLR_ERR;
//	

//	
//	if(is_over_range(_pos))
//	{
//		_ERROR = ERR_OUT_OF_RANGE;
//		err_handle(_ERROR);
//		return _ERROR;
//	}
//	
//	for(i=0; i<12; i++)
//	{
//		_pwm[i] = pos2pwm(_pos[i]);
//		
//	}
//	
//	
//	TIM3->CCR1 = _pwm[0];
//	TIM3->CCR2 = _pwm[1];
//	TIM3->CCR3 = _pwm[2];
//	TIM4->CCR1 = _pwm[3];
//	TIM4->CCR2 = _pwm[4];
//	TIM4->CCR3 = _pwm[5];
//	TIM1->CCR1 = _pwm[6];
//	TIM1->CCR2 = _pwm[7];
//	TIM1->CCR3 = _pwm[8];
//	TIM2->CCR1 = _pwm[9];
//	TIM2->CCR2 = _pwm[10];
//	TIM2->CCR3 = _pwm[11];
//	return 0;
//}




///*******************************************************************************************************
//*功    能：以角度的方式控制舵机
//*参    数：_buf：12个关节角度数组
//*返    回：0：没有错误；others：有错误，调用errhandle（）来查看错误
//*******************************************************************************************************/
//int servo_ctrl_degree(double* _buf)
//{
//	u8 i = 0;
//	u8 res = 0;
//	u16 pos[12] = {1023};
//	for(i=0; i<12; i++)
//	{
//		_buf[i] += INIT_ANGLE[ctrl_mode][i];
//		pos[i] = (u16)(DGR2POS * ((SERVO_DIR[i]<0)? (180.0-_buf[i]) : (_buf[i])));
//	}
//	
////	for(i=0; i<6; i++)
////	{
////		printf("%d ", pos[i]);
////	}
////	printf("\r\n");
//	
//	
//	res = servo_ctrl(pos);
//	return res;
//}

///*******************************************************************************************************
//*功    能：关节位置转换成pwm
//*参    数：_pos：12个关节的位置数组，范围（0， 2047）
//*返    回：无
//*******************************************************************************************************/
//u16 pos2pwm(u16 pos)
//{
//	u32 _pos = pos;
//	return (u16)(2000 * _pos / 2047 + 500);
//}

///*******************************************************************************************************
//*功    能：判断设定位置是否超出舵机运动范围
//*参    数：12个关节位置
//*返    回：true：超出范围；false：未超出范围
//*******************************************************************************************************/
//bool is_over_range(u16* _p)
//{
//	u8 i = 0;
//	for(i=0; i<12; i++)
//	{
//		if((_p[i] <  SERVO_POS_RANGE[i][0]) || (_p[i] >= SERVO_POS_RANGE[i][1]))
//			return true;
//	}
//	
//	return false;
//}

///*******************************************************************************************************
//*功    能：打印错误信息并重置错误标志
//*参    数：错误序号
//*返    回：无
//*******************************************************************************************************/
//void err_handle(ERROR_LIST _err)
//{
//	const char* s = Servo_Err_Info[_err];
//	printf("--------------------------------error message--------------------------------\r\n");
//	printf("ERROR! Error info: %s\r\n\r\n", s);
//	printf("--------------------------------error message--------------------------------\r\n\r\n");
//	
//	clear_err_msg();
//}
///*******************************************************************************************************
//*功    能：重置错误标志
//*参    数：无
//*返    回：无
//*******************************************************************************************************/
//void clear_err_msg(void)
//{
//	_ERROR = 0;
//}
