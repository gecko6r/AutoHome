#include "delay.h"
#include "sys.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用OS，则包括下面的头文件即可
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//֧支持OS时使用
#include "task.h"
#endif


static u8  fac_us=0;							//us延时系数		   
static u16 fac_ms=0;							//ms延时系数，支持OS时，代表每个节拍的ms数
	

/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	Systick中断函数
	* @param  	None
	* @retval 	None
	*/
void SysTick_Handler(void)
{	
	if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)					//OS开始跑了，才执行
	{
		xPortSysTickHandler();      	 				//
	}
}

/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化延时函数，当使用OS时，此函数会初始化OS的时钟节拍
	* @brief	Systick的时钟频率固定为AHB时钟的1/8
	* @param  	SYSCLK：系统时钟频率，单位MHz
	* @retval 	None
	*/
void delay_init(u8 SYSCLK)
{

	u32 reload;
	
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
	fac_us=SYSCLK/8;							//us延时系数
	
	reload=SYSCLK/8;							//每秒钟的计数次数，单位MHz   
	reload*=1000000/configTICK_RATE_HZ;			//重装载值（实际为间隔，单位us）
											
	fac_ms=1000/configTICK_RATE_HZ;				//毫秒延时系数
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//
	SysTick->LOAD=reload; 						//
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; 	//使能Systick  

}								    

/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	μs延时函数
	* @param  	nus：延时时间（μs）
	* @retval 	None
	*/							   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//读取当前重装载值    	 
	ticks=nus*fac_us; 						//时钟数 = 微秒数*微秒tick数
	told=SysTick->VAL;        				//初始化told为当前Systick值（注意，VAL是不断减小的）
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//已经过去的时间
			else tcnt+=reload-tnow+told;	//已经过去的时间（如果经历了一次重装载）
			told=tnow;						//更新told
			if(tcnt>=ticks)break;			//时间到
		}  
	};										    
}  
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	毫秒延时函数
	* @param  	nms：延时时间（ms）
	* @retval 	None
	*/
void delay_ms(u16 nms)
{	
	if(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)//如果已开始任务（OS已初始化）    
	{		 
		if(nms>=fac_ms)										//
		{ 
   			vTaskDelay(nms/fac_ms);							//延时的毫秒数/每毫秒的tick数，实际参数为tick数
		}
		nms%=fac_ms;										//如果有余数
	}
	delay_us((u32)(nms*1000));								//
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	得到当前SysTick的VAL寄存器值
	* @param  	None
	* @retval 	当前SysTick VAL寄存器值
	*/
TickType_t getCurrSysTick(void)
{
	return SysTick->VAL;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	计算时间间隔(单位是微秒）
	* @param  	ulStartTime：开始时的SysTick VAL
	* @param  	ulEndTime：结束时的SysTick VAL
	* @retval 	时间间隔（us）
	*/
TickType_t usCalc(uint32_t ulStartTime, uint32_t ulEndTime)
{
	TickType_t ulReload = SysTick->LOAD;
	
	if(ulStartTime > ulEndTime) return ((ulStartTime - ulEndTime) / fac_us);
	else return ((ulStartTime + ulReload - ulEndTime) / fac_us);
}
































