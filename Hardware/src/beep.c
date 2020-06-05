#include "beep.h"
#include "delay.h"

void beepInit( void  )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd( beepRCC, ENABLE );
	
	GPIO_InitStructure.GPIO_Pin 		= beepPin;
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_OUT;		
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType 		= GPIO_OType_PP; 	
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_DOWN;	 	
	GPIO_Init( beepGPIO, &GPIO_InitStructure ); 			

}


void beep( uint16_t beepHz, uint16_t times, uint16_t interval_ms )
{
	uint32_t interval_us;
	uint16_t t;
	uint8_t i;
	assert_param( beepHz );
	interval_us = 1000000 / beepHz;
	
	
	while( times-- )
	{
		t = interval_ms * 1000 / interval_us;
		while( t-- )
		{
			GPIO_SetBits( beepGPIO, beepPin );
			delay_us( interval_us );
			GPIO_ResetBits( beepGPIO, beepPin );
			delay_us( interval_us );
		}
		
		delay_us( interval_ms * 1000 );
	}
		
	
}