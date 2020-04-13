#include "stdio.h"
#include "stm32f4xx_conf.h"


#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					 
#endif


#include "usart.h"	
#include "dma.h"
#include "dynamixel.h"
#include "led.h"

//static u8 ucUsart1TxBuf[MAX_BUF_SIZE] = {0U};
//static u8 ucUsart1RxBuf[MAX_BUF_SIZE] = {0U}; 

	


/*----------------------------------------------------------------*/
#pragma import(__use_no_semihosting)             
//            
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
// 
void _sys_exit(int x) 
{ 
	x = x; 
} 
//
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//
	USART1->DR = (u8) ch;      
	return ch;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	初始化USART1
    * @param  	ulBaudrate：波特率
    * @retval 	None
    */
void USART1_Init(uint32_t ulBaudrate){
	
   
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_9 | GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType 		= GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	USART_InitStruct.USART_BaudRate 			= ulBaudrate;//
	USART_InitStruct.USART_WordLength 			= USART_WordLength_8b;//
	USART_InitStruct.USART_StopBits 			= USART_StopBits_1;//
	USART_InitStruct.USART_Parity 				= USART_Parity_No;//
	USART_InitStruct.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;//
	USART_InitStruct.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;	//
	USART_Init(USART1, &USART_InitStruct); 						//

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);							//
	NVIC_InitStructure.NVIC_IRQChannel 						= USART1_IRQn;	//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 7;				//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;				//
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;		//
	NVIC_Init(&NVIC_InitStructure);											//

	
	USART_Cmd(USART1, ENABLE);  											//
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	初始化USART2
    * @param  	ulBaudrate：波特率
    * @retval 	None
    */
void USART2_Init(u32 ulBaudrate)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 

	GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF;		
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType 		= GPIO_OType_PP; 	
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;	 	
	GPIO_Init(GPIOA,&GPIO_InitStructure); 					
	
	GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_IN;		
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 					

	USART_InitStruct.USART_BaudRate 	= ulBaudrate;			
	USART_InitStruct.USART_WordLength 	= USART_WordLength_8b;	
	USART_InitStruct.USART_StopBits 	= USART_StopBits_1;		
	USART_InitStruct.USART_Parity 		= USART_Parity_No;		
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;			
	USART_InitStruct.USART_Mode 		= USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(USART2, &USART_InitStruct); 						
	
	NVIC_InitStructure.NVIC_IRQChannel 						= USART2_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 2;				
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 1;				
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;		
	NVIC_Init(&NVIC_InitStructure);				

	USART_HalfDuplexCmd(USART2, ENABLE);

	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);		
	USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);		
//	
	
	USART_Cmd(USART2, ENABLE);  						
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);		
//	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);		

}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	串口1 ISR
    * @param  	None
    * @retval 	None
    */
void USART1_IRQHandler(void) 
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	} 
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	串口2 ISR
    * @param  	None
    * @retval 	None
    */ 
void USART2_IRQHandler(void) 
{	
	static u8 data = 0, i = 0;

	//串口2接收中断标志置位
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		//将接受到的字节放入接收数组
		ucDmaUsart2RxBuf[xServoMsg.ucByteRecved++] = USART_ReceiveData(USART2);
		//点亮LED
		LED = 0;
		//清除串口2接收中断标志位
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);

	}
	
	//串口2空闲中断标志置位
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		//消息结构体中的数据就绪状态为true
		if(xServoMsg.ucByteRecved > 10)
			xServoMsg.bDataReady = true;
		//这里是读一次接收寄存器以清除IDLE标志（详见手册）
		data  = USART_ReceiveData(USART2);
		//关闭LED
		LED = 1;
		
	}
}

	
