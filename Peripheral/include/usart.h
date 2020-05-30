#ifndef __USART_H
#define __USART_H

#include "sys.h" 
#include "stdio.h"

#define usartServoIOCtrlGPIO		GPIOA
#define usartServoIOCtrlRCC			RCC_AHB1Periph_GPIOA
#define usartServoIOCtrlPin			GPIO_Pin_11	

void USART1_Init(uint32_t ulBaudrate);
void USART2_Init(uint32_t ulBaudrate);
void USART_DirIO_Init( void );
void USART_DataOut( void );
void USART_DataIn( void );
extern volatile bool usart2Busy;

extern volatile uint32_t g_idle_wait_time;
extern volatile uint32_t idle_buf[ 50 ];
extern volatile uint8_t idle_count;

#endif


