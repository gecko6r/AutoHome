#ifndef _HARDWARE_LED_H
#define _HARDWARE_LED_H
#include "sys.h"

//LED¶Ë¿Ú¶¨Òå
#define LED PEout(2)	// DS0 

void LED_Init(void);	
void blink(u8 ucTimes, u16 usDelayMs);
#endif
