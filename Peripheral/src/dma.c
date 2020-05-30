#include "FreeRTOS.h"

#include "dma.h"
#include "led.h"
#include "usart.h"
#include "dynamixel.h"
#include "dataComm.h"
#include "delay.h"


/* Static variables ----------------------------------------------------------*/




/* Public functions ----------------------------------------------------------*/

uint32_t finish_tick;

/**
* @brief 	初始化串口1 Tx DMA，高优先级，开启FIFO设定阈值为满，16节拍一次突发
  * @param  None
  * @retval None
  */
void DMA_ServoTxInit( uint32_t ulPeriphAddr, uint32_t ulMemAddr, uint16_t usBufferSize )
{
	DMA_InitTypeDef DMA_InitStruct;						
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);					//使能DMA时钟

	DMA_DeInit(dmaServoTxStream);   										//重置DMA
	
	DMA_InitStruct.DMA_Channel				= DMA_Channel_4;				//DMA通道4
	DMA_InitStruct.DMA_PeripheralBaseAddr 	= ulPeriphAddr;  				//外设基地址
	DMA_InitStruct.DMA_Memory0BaseAddr 		= ulMemAddr;  					//内存基地址
	DMA_InitStruct.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;  	//数据方向：内存到外设
	DMA_InitStruct.DMA_BufferSize 			= usBufferSize;  				//数组大小
	DMA_InitStruct.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;  	//外设地址自增：关
	DMA_InitStruct.DMA_MemoryInc 			= DMA_MemoryInc_Enable;  		//内存地址自增：开
	DMA_InitStruct.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;  //外设数据大小：字节
	DMA_InitStruct.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte; 		//内存字节大小：字节
	DMA_InitStruct.DMA_Mode 				= DMA_Mode_Normal;  			//DMA模式（普通或循环发送）：普通
	DMA_InitStruct.DMA_Priority 			= DMA_Priority_High; 			//DMA优先级：高
	
	DMA_InitStruct.DMA_FIFOMode 			= DMA_FIFOMode_Enable;			//FIFO模式：开启
	DMA_InitStruct.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;		//FIFO阈值：满（4字）
	DMA_InitStruct.DMA_MemoryBurst			= DMA_MemoryBurst_INC16;		//16节拍一次突发
	DMA_InitStruct.DMA_PeripheralBurst		= DMA_PeripheralBurst_Single;	//外设突发节拍：1
	DMA_Init(dmaServoTxStream, &DMA_InitStruct);  
	  	
	NVIC_InitStructure.NVIC_IRQChannel 						= dmaServoTxIRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(dmaServoTxStream, DMA_IT_TC, ENABLE);			//使能dma发送完成中断
	
	DMA_Cmd(dmaServoTxStream, DISABLE);							//初始化完成，先关闭发送DMA
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	初始化串口2接收DMA
  * @param  None
  * @retval None
  */
void DMA_ServoRxInit( uint32_t ulPeriphAddr, uint32_t ulMemAddr, uint16_t usBufferSize )
{
	DMA_InitTypeDef DMA_InitStruct;
	DMA_Stream_TypeDef* DMA_Stream = dmaServoRxStream;							
	NVIC_InitTypeDef NVIC_InitStructure;							//最大传送数据量	
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);					//使能DMA时钟

	DMA_DeInit(DMA_Stream);   												//重置DMA
	
	DMA_InitStruct.DMA_Channel				= DMA_Channel_4;				//DMA通道4
	DMA_InitStruct.DMA_PeripheralBaseAddr 	= ulPeriphAddr;  				//外设基地址
	DMA_InitStruct.DMA_Memory0BaseAddr 		= ulMemAddr;  					//内存基地址
	DMA_InitStruct.DMA_DIR 					= DMA_DIR_PeripheralToMemory ;  //数据方向：外设到内存
	DMA_InitStruct.DMA_BufferSize 			= usBufferSize;  				//数组大小
	DMA_InitStruct.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;  	//外设地址自增：关
	DMA_InitStruct.DMA_MemoryInc 			= DMA_MemoryInc_Enable;  		//内存地址自增：开
	DMA_InitStruct.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;  //外设数据大小：字节
	DMA_InitStruct.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte; 		//内存字节大小：字节
	DMA_InitStruct.DMA_Mode 				= DMA_Mode_Normal;  			//DMA模式（普通或循环发送）：普通
	DMA_InitStruct.DMA_Priority 			= DMA_Priority_Medium; 			//DMA优先级：中等
	
	DMA_InitStruct.DMA_FIFOMode 			= DMA_FIFOMode_Disable;			//FIFO模式：
	DMA_InitStruct.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;		//FIFO阈值：满（4字）
	DMA_InitStruct.DMA_MemoryBurst			= DMA_MemoryBurst_Single;		//一个节拍一次突发
	DMA_InitStruct.DMA_PeripheralBurst		= DMA_PeripheralBurst_Single;	//
	
	DMA_Init(DMA_Stream, &DMA_InitStruct);  
	
	NVIC_InitStructure.NVIC_IRQChannel 						= dmaServoRxIRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig( dmaServoRxStream, DMA_IT_TC, ENABLE);			//使能dma发送完成中断
	
	DMA_Cmd( dmaServoRxStream, DISABLE);											//先关闭串口接收DMA
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	串口2发送DMA中断服务函数
  * @param  None
  * @retval None
  */
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(dmaServoTxStream, dmaServoTxTCIF) != RESET) 
	{
		finish_tick = SysTick->VAL;
		//清除标志位
		DMA_ClearFlag(dmaServoTxStream, dmaServoTxTCIF);
		//关闭DMA
		DMA_Cmd(dmaServoTxStream, DISABLE);
		
		//发送完毕，使能串口接收中断和空闲中断
		if( xServoMsg.fReadEnable )
		{
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
		}
		usart2Busy = false;
	}
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	串口1接收DMA中断服务函数
  * @param  None
  * @retval None
  */
void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus( dmaServoRxStream, dmaServoRxTCIF ) != RESET) 
	{
		LED4 = 0;
		//清除标志位
		DMA_ClearFlag( dmaServoRxStream, dmaServoRxTCIF );
		//关闭DMA
		DMA_Cmd( dmaServoRxStream, DISABLE );
	}
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	使能单次DMA发送
  * @param  usLen：发送的数据长度
  * @retval None
  */
void DMA_SendData( DMA_Stream_TypeDef *DMAy_Streamx, uint16_t usLen )
{
	g_idle_wait_time = 0;
	while( usart2Busy )
		g_idle_wait_time++;
	usart2Busy = true;
	
	DMA_Cmd(DMAy_Streamx, DISABLE );  //
 	DMA_SetCurrDataCounter(DMAy_Streamx, usLen);//
	
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
	
 	DMA_Cmd(DMAy_Streamx, ENABLE);  //
	
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	使能单次DMA发送
  * @param  usLen：发送的数据长度
  * @retval None
  */
void DMA_RecvData( DMA_Stream_TypeDef *DMAy_Streamx, uint16_t usLen )
{
	g_idle_wait_time = 0;
	while( usart2Busy )
		g_idle_wait_time++;
	usart2Busy = true;
	
	DMA_Cmd(DMAy_Streamx, DISABLE );  //
 	DMA_SetCurrDataCounter(DMAy_Streamx, usLen);//
 	DMA_Cmd(DMAy_Streamx, ENABLE);  //
}
