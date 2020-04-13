#include "stdio.h"

#include "delay.h"
#include "usart.h"
#include "led.h"
#include "i2c.h"

#include "dynamixel.h"
 
#define START_TASK_PRIO			1
#define START_STK_SIZE			120
void start_task(void * pvParameters);  //
TaskHandle_t StartTask_Handler;		//
 
 
#define TASK1_TASK_PRIO			2
#define TASK1_STK_SIZE			120
void task1_task(void * pvParameters);
TaskHandle_t Task1Task_Handler;		// 
 
#define TASK2_TASK_PRIO			3
#define TASK2_STK_SIZE			120 
void task2_task(void * pvParameters);
TaskHandle_t Task2Task_Handler;		//


uint8_t flag = 0, count = 0, ret = 0;
uint8_t i = 0;
uint8_t ucBuf[20] = {0};
I2cErrType_t err = I2C_ERR_NoError;
uint8_t dataBuf[2] = {0x00, 0x00};
uint8_t saveBuf[2] = {0x00, 0x00};
int main(void)
{	
	
	
	uint32_t buf[16] = {0};
	
	for(i=0; i<16; i++)
	{
		buf[i] = 2047;
	}
	
	delay_init(168);
	
	DXL_ServoInit(eBD2M, ENABLE);
	USART1_Init(921600);
	IIC_Init();
	
	LED_Init();
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	LED = 0;
	delay_ms(1000);
	DXL_SetAllGoalPos(buf);
	delay_ms(100);
	LED = 1;
	xServoMsg.err = false;
		

	xTaskCreate((TaskFunction_t	) start_task,
				(char*			) "start_task",
				(uint16_t		) START_STK_SIZE,
				(void * 		) NULL,
				(UBaseType_t	) START_TASK_PRIO,
				(TaskHandle_t*	) &StartTask_Handler);
    vTaskStartScheduler();          
}
 
void start_task(void * pvParameters)
{
	//创建Task1
	xTaskCreate((TaskFunction_t	) task1_task,
				(char*			) "task1_task",
				(uint16_t		) TASK1_STK_SIZE,
				(void * 		) NULL,
				(UBaseType_t	) TASK1_TASK_PRIO,
				(TaskHandle_t*	) &Task1Task_Handler);
				
	//创建Task2
	xTaskCreate((TaskFunction_t	) task2_task,
				(char*			) "task2_task",
				(uint16_t		) TASK2_STK_SIZE,
				(void * 		) NULL,
				(UBaseType_t	) TASK2_TASK_PRIO,
				(TaskHandle_t*	) &Task2Task_Handler);
	vTaskDelete(StartTask_Handler); //NULL
				

}
 

void task1_task(void * pvParameters)
{
	char task1_num=0;
	
	while(1)
	{

		
//		LED = ~LED;
//		
		vTaskDelay(300 / portTICK_RATE_MS);
	}
}

void task2_task(void * pvParameters)
{
//	uint8_t bytes = 0;
	
	while(1)
	{

//		if(!flag)
//		{
//			DXL_GetRegState(ADDR_Present_Current, REG_2_BYTE);
//			flag = 1;
//		}

//		if(isMsgDataReady(xServoMsg))
//		{
//			for(count=0; count<xServoMsg.ucByteRecved; count++)
//			ret = DXL_GetPresentParam(&xServoMsg);
//			
//			if(!ret)
//			{
//				
//				
//				printf("%6.2f\r\n", fServoStatusBuf[COL_Current][0]);
//			}
//		}
//		
//		flag = 0;

		

		count++;
//		I2C_MultiRead(I2C2, 0xa0, 0x34, 12, ucBuf, &err);
//		printf("count: %3d, err: %d\r\n", count, err);
//		for(i=0; i<12; i++)
//			printf("%02X ", ucBuf[i]);
//		printf("\r\n\r\n");

		I2C_MultiWrite(I2C2, 0xa0, 0x1b, 2, dataBuf, &err);
		printf("count: %3d, err: %d\r\n", count, err);
		
		I2C_MultiWrite(I2C2, 0xa0, 0x00, 2, saveBuf, &err);
		printf("count: %3d, err: %d\r\n", count, err);
		
		I2C_MultiRead(I2C2, 0xa0, 0x1b, 1, ucBuf, &err);
		printf("count: %3d, err: %d\r\n", count, err);
		printf("data_L: %d, data_H: %d\r\n", ucBuf[0], ucBuf[1]);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
		
}
 
