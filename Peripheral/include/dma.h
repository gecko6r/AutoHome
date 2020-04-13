#ifndef _PERIPHERAL_DMA_H
#define _PERIPHERAL_DMA_H

#include "sys.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t ucDmaUsart2TxBuf[MAX_BUF_SIZE];
extern uint8_t ucDmaUsart2RxBuf[MAX_BUF_SIZE];

extern uint8_t ucDmaUsart1TxBuf[100];
/* Private function prototypes -----------------------------------------------*/



void xDMA_Init(void);
/* Private functions ---------------------------------------------------------*/
void DMA_Usart2_Tx_Init(void);
void DMA_Usart2_Rx_Init(void);
void DMA_SendData(uint16_t usLen);
int DMA_RecvData(uint16_t usLen);
void DMA_Usart1_Init(void);

#endif
