#include "dma.h"
#include "led.h"
#include "usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
uint8_t ucDmaUsart2TxBuf[MAX_BUF_SIZE] = {0U};
uint8_t ucDmaUsart2RxBuf[MAX_BUF_SIZE] = {0U};

/* Static variables ---------------------------------------------------------*/
static DMA_Stream_TypeDef* dmaUsart2Tx = DMA1_Stream6;
static DMA_Stream_TypeDef* dmaUsart2Rx = DMA1_Stream5;

/* Public functions -----------------------------------------------*/


/* ---------------------------------------------------------------------------*/



/* Public functions -----------------------------------------------*/


/**
* @brief 	初始化串口2Tx DMA，高优先级，开启FIFO设定阈值为满，16节拍一次突发
  * @param  None
  * @retval None
  */
void DMA_Usart2_Tx_Init(void)
{
	DMA_InitTypeDef DMA_InitStruct;						
	NVIC_InitTypeDef NVIC_InitStructure;
	
	u32 ulPeriphAddr = (u32)&(USART2->DR);									//外设地址：串口2数据寄存器
	u32 ulMemAddr = (u32) ucDmaUsart2TxBuf;									//存储器地址；舵机发送数据数组
	u32 ulBufferSize = MAX_BUF_SIZE;										//最大传送数据量
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);					//使能DMA时钟

	DMA_DeInit(dmaUsart2Tx);   												//重置DMA
	
	DMA_InitStruct.DMA_Channel				= DMA_Channel_4;				//DMA通道4
	DMA_InitStruct.DMA_PeripheralBaseAddr 	= ulPeriphAddr;  				//外设基地址
	DMA_InitStruct.DMA_Memory0BaseAddr 		= ulMemAddr;  					//内存基地址
	DMA_InitStruct.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;  	//数据方向：内存到外设
	DMA_InitStruct.DMA_BufferSize 			= ulBufferSize;  				//数组大小
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
	DMA_Init(dmaUsart2Tx, &DMA_InitStruct);  
	  	
	NVIC_InitStructure.NVIC_IRQChannel 						= DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(dmaUsart2Tx, DMA_IT_TC, ENABLE);			//使能dma发送完成中断
	
	DMA_Cmd(dmaUsart2Tx, DISABLE);							//初始化完成，先关闭发送DMA
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	初始化串口2接收DMA（暂未启用）
  * @param  None
  * @retval None
  */
void DMA_Usart2_Rx_Init(void)
{
	DMA_InitTypeDef DMA_InitStruct;
	DMA_Stream_TypeDef* DMA_Stream = dmaUsart2Rx;							
	
	u32 ulPeriphAddr = (u32)&(USART2->DR);									//外设地址：串口2数据寄存器
	u32 ulMemAddr = (u32) ucDmaUsart2RxBuf;									//存储器地址；舵机返回数据数组
	u32 ulBufferSize = MAX_BUF_SIZE;										//最大传送数据量	
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);					//使能DMA时钟

	DMA_DeInit(DMA_Stream);   												//重置DMA
	
	DMA_InitStruct.DMA_Channel				= DMA_Channel_4;				//DMA通道4
	DMA_InitStruct.DMA_PeripheralBaseAddr 	= ulPeriphAddr;  				//外设基地址
	DMA_InitStruct.DMA_Memory0BaseAddr 		= ulMemAddr;  					//内存基地址
	DMA_InitStruct.DMA_DIR 					= DMA_DIR_PeripheralToMemory ;  //数据方向：外设到内存
	DMA_InitStruct.DMA_BufferSize 			= ulBufferSize;  				//数组大小
	DMA_InitStruct.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;  	//外设地址自增：关
	DMA_InitStruct.DMA_MemoryInc 			= DMA_MemoryInc_Enable;  		//内存地址自增：开
	DMA_InitStruct.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;  //外设数据大小：字节
	DMA_InitStruct.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte; 		//内存字节大小：字节
	DMA_InitStruct.DMA_Mode 				= DMA_Mode_Normal;  			//DMA模式（普通或循环发送）：普通
	DMA_InitStruct.DMA_Priority 			= DMA_Priority_High; 			//DMA优先级：中等
	
	DMA_InitStruct.DMA_FIFOMode 			= DMA_FIFOMode_Disable;			//FIFO模式：
	DMA_InitStruct.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;		//FIFO阈值：满（4字）
	DMA_InitStruct.DMA_MemoryBurst			= DMA_MemoryBurst_Single;		//一个节拍一次突发
	DMA_InitStruct.DMA_MemoryBurst			= DMA_PeripheralBurst_Single;		//
	
	DMA_Init(DMA_Stream, &DMA_InitStruct);  
	
	DMA_Cmd(dmaUsart2Rx, DISABLE);											//先关闭串口接收DMA
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	串口2发送DMA中断服务函数
  * @param  None
  * @retval None
  */
void DMA1_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET) 
	{
		//清除标志位
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
		//关闭DMA
		DMA_Cmd(DMA1_Stream6, DISABLE);
		
		//发送完毕，使能串口接收中断和空闲中断
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
		
		//打开发送完成中断,发送最后两个字节
//		USART_ITConfig(USART1,USART_IT_TC,ENABLE);
	}
}
/* ---------------------------------------------------------------------------*/

/**
  * @brief 	使能单次DMA发送
  * @param  usLen：发送的数据长度
  * @retval None
  */
void DMA_SendData(uint16_t usLen)
{

	DMA_Cmd(DMA1_Stream6, DISABLE );  //
 	DMA_SetCurrDataCounter(DMA1_Stream6, usLen);//
	
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);
	
 	DMA_Cmd(DMA1_Stream6, ENABLE);  //
	
}


