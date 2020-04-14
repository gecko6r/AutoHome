#include "stdio.h"

#include "delay.h"
#include "usart.h"
#include "led.h"
#include "gyro.h"

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



int main(void)
{	
	
	

	
	delay_init(168);
	
	DXL_ServoInit(eBD2M, ENABLE);
	USART1_Init(921600);
	Gyro_Init();
	LED_Init();
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

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

		
		LED = ~LED;
//		
		vTaskDelay(300 / portTICK_RATE_MS);
	}
}

void task2_task(void * pvParameters)
{
	GyroAccType_t xGyroAcc;
	GyroAngleType_t xGyroAngle;
	GyroErrType_t xGyroErr;
	
	while(1)
	{

		xGyroAcc = Gyro_GetCurrAcc(&xGyroErr);
		
		printf("acc_x: %03.3f, acc_y: %03.3f, acc_z: %03.3f\r\n", 
				xGyroAcc.x, xGyroAcc.y, xGyroAcc.z);
		printf("err info: %02X\r\n", xGyroErr);

		xGyroErr = 0;
		
		xGyroAngle = Gyro_GetCurrAng(&xGyroErr);
		printf("roll: %03.3f, pitch: %03.3f, yaw: %03.3f\r\n", 
				xGyroAngle.roll, xGyroAngle.pitch, xGyroAngle.yaw);
//				printf("roll: %d, pitch: %d, yaw: %d\r\n", 
//				xGyroAngle.roll, xGyroAngle.pitch, xGyroAngle.yaw);
		printf("err info: %02X\r\n\r\n", xGyroErr);

		vTaskDelay(50 / portTICK_RATE_MS);
	}
		
}
 
