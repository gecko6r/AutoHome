#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>	
#include "FreeRTOS.h"
 
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
TickType_t getCurrSysTick(void);
TickType_t usCalc(uint32_t ulStartTime, uint32_t ulEndTime);
#endif





























