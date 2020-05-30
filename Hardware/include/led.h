#ifndef _LED_H
#define _LED_H

#include "sys.h"


#define LED0 PEout( 2 )	
#define LED1 PDout( 11 )
#define LED2 PDout( 12 )
#define LED3 PDout( 13 )
#define LED4 PDout( 14 )

void LED_Init(void);	
void blink(uint8_t led_num, uint8_t ucTimes, uint16_t usDelayMs);

#endif
