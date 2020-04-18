#ifndef _PERIPHERAL_DMA_H
#define _PERIPHERAL_DMA_H

#include "sys.h"
#include "dataManager.h"


void xDMA_Init(void);
void DMA_Usart2_Tx_Init(void);
void DMA_Usart2_Rx_Init(void);
void DMA_SendData(uint8_t* pucSrc, uint16_t usLen);
int DMA_RecvData(uint16_t usLen);
void DMA_Usart1_Init(void);

#endif
