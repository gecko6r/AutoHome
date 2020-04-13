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




///************************************************Լ��**************************************************/
////�ȵı�ţ�1��->��ǰ��, 2��->��ǰ��, 3��->�����, 4��->�Һ��ȣ�
////�ؽڱ�ţ�1��->��ؽ�, 2��->��ؽ�, 3��->��ؽڣ�
////������򣺶��˳ʱ��Ϊ�����ؽ����򣺹ؽ�1��̧���ؽ�2ǰ�죬�ؽ�3�ں�
////λ��ӳ�䣺90��<->1023, 0��<->0, 180��<->2047
///************************************************����***************************************************/

////�ؽ�λ�����������������������û�ж���װ�䵼�µ����(��λpos)
//const int SERVO_POS_CRC[12] = {0, 0, 0,			//��ǰ��[�ؽ�1�� �ؽ�2�� �ؽ�3]
//							   0, 0, 0, 		//��ǰ��[�ؽ�1�� �ؽ�2�� �ؽ�3]
//							   0, 0, 0, 		//�����[�ؽ�1�� �ؽ�2�� �ؽ�3]
//							   0, 0, 0};		//�Һ���[�ؽ�1�� �ؽ�2�� �ؽ�3]

////�ؽ�λ�����������ڲ�����������ϼ�϶���µ����()
//const u16 SERVO_DEAD_ZONE[12] = {0, 0, 0,		
//								 0, 0, 0, 		
//								 0, 0, 0, 		
//								 0, 0, 0};		

////�ؽ�λ���޷������ڱ������˶���������λ��
//const u16 SERVO_POS_RANGE[12][2] = {250, 1776, 250, 1776, 250, 1776,
//								250, 1776, 250, 1776, 250, 1776,
//								250, 1776, 250, 1776, 250, 1776,
//								259, 1795, 258, 1794, 240, 1776};	
//								 
////�ؽڷ���
//const int SERVO_DIR[12] = {-1, -1, 1,		
//						   1, 1, -1, 		
//						   1, 1, -1, 		
//						   -1, -1, 1};		
//								   
///************************************************��ʼ�Ƕ�***************************************************/
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
//*��    �ܣ���ʼ����ʱ����PWM
//*��    ������
//*��    �أ���
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
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
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
//*��    �ܣ���ʼ����ʱ����PWM��ʹ�������˶�����ʼ״̬
//*��    ����mode����ʼ״̬ѡ��
//*��    �أ���
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
//*��    �ܣ�������ƣ�ֱ�������ڶ����Ψһ����
//*��    ����_pos��12���ؽڵ�λ�����飬��Χ��0�� 2047��
//*��    �أ�0��û�д���others���д��󣬵���errhandle�������鿴����
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
//*��    �ܣ��ԽǶȵķ�ʽ���ƶ��
//*��    ����_buf��12���ؽڽǶ�����
//*��    �أ�0��û�д���others���д��󣬵���errhandle�������鿴����
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
//*��    �ܣ��ؽ�λ��ת����pwm
//*��    ����_pos��12���ؽڵ�λ�����飬��Χ��0�� 2047��
//*��    �أ���
//*******************************************************************************************************/
//u16 pos2pwm(u16 pos)
//{
//	u32 _pos = pos;
//	return (u16)(2000 * _pos / 2047 + 500);
//}

///*******************************************************************************************************
//*��    �ܣ��ж��趨λ���Ƿ񳬳�����˶���Χ
//*��    ����12���ؽ�λ��
//*��    �أ�true��������Χ��false��δ������Χ
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
//*��    �ܣ���ӡ������Ϣ�����ô����־
//*��    �����������
//*��    �أ���
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
//*��    �ܣ����ô����־
//*��    ������
//*��    �أ���
//*******************************************************************************************************/
//void clear_err_msg(void)
//{
//	_ERROR = 0;
//}
