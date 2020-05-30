#ifndef _BEEP_H
#define _BEEP_H

#include "sys.h"

#define beepRCC		RCC_AHB1Periph_GPIOA
#define beepGPIO	GPIOA
#define beepPin		GPIO_Pin_15

void beepInit( void );

void beep(  uint16_t beepHz, uint16_t times, uint16_t interval_ms );



#endif
