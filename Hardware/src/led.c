#include "led.h" 
#include "delay.h"


/**
  * @brief  初始化板载应用LED
  * @param  None
  * @retval None
  */

void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE, GPIO_Pin_2);
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void blink(u8 ucTimes, u16 usDelayMs)
{
	while(ucTimes--)
	{
		LED = 0;
		delay_ms(usDelayMs);
		LED = 1;
		delay_ms(usDelayMs);
	}
	
}



