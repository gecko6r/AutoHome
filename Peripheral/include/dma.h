#ifndef _PERIPHERAL_DMA_H
#define _PERIPHERAL_DMA_H

#include "sys.h"
#include "usart.h"

#define dmaServoTxPeriphRCC			RCC_AHB1Periph_DMA2
#define dmaServoRxPeriphRCC			RCC_AHB1Periph_DMA2
#define dmaServoTxStream			DMA2_Stream7
#define dmaServoRxStream			DMA2_Stream2
#define dmaServoTxChn				DMA_Channel_4
#define dmaServoRxChn				DMA_Channel_4
#define dmaServoTxIRQn				DMA2_Stream7_IRQn
#define dmaServoRxIRQn				DMA2_Stream2_IRQn
#define dmaServoTxTCIF				DMA_IT_TCIF7
#define dmaServoRxTCIF				DMA_IT_TCIF2

#define dmaServoTxPeriphAddr		( uint32_t )&USART1->DR
#define dmaServoRxPeriphAddr		( uint32_t )&USART1->DR

extern uint32_t finish_tick;

void xDMA_Init(void);
void DMA_ServoTxInit( uint32_t ulPeriphAddr, uint32_t ulMemAddr, uint16_t usBufSize );
void DMA_ServoRxInit( uint32_t ulPeriphAddr, uint32_t ulMemAddr, uint16_t usBufSize );
void DMA_SendData( DMA_Stream_TypeDef *DMAy_Streamx, uint16_t usLen );
void DMA_RecvData( DMA_Stream_TypeDef *DMAy_Streamx, uint16_t usLen );


#endif
