#include "led.h" 
#include "delay.h"

/**
  * @brief  初始化LED
  * @param  None
  * @retval None
  */

void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE, GPIO_Pin_2);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_11);
	GPIO_SetBits(GPIOD, GPIO_Pin_12);
	GPIO_SetBits(GPIOD, GPIO_Pin_13);
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
}

/**
  * @brief  LED闪烁
  * @param  None
  * @retval None
  */
void blink(uint8_t ledIndex, uint8_t ucTimes, uint16_t usDelayMs)
{
	volatile unsigned long *led;
	
	if( ledIndex == 0 )
		led = &LED0;
	
	if( ledIndex == 1 )
		led = &LED1;
	
	if( ledIndex == 2 )
		led = &LED2;
	
	if( ledIndex == 3 )
		led = &LED3;
	
	if( ledIndex == 4 )
		led = &LED4;
		
	while(ucTimes--)
	{
		*led = 0;
		delay_ms( usDelayMs );
		*led = 1;
		delay_ms( usDelayMs );
	}
	
}



