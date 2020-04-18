#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"

#include "delay.h"
#include "usart.h"
#include "led.h"
#include "gyro.h"
#include "nrf24l01p.h"
#include "dynamixel.h"
 
 
 
TaskHandle_t readTask_Handler;		//
TaskHandle_t ctrlTask_Handler;		// 
TaskHandle_t ledTask_Handler;		//

GyroAccType_t xAcc;
GyroAngleType_t xAngle;
GyroErrType_t gyroErr = I2C_ERR_NoError;

uint32_t buf1[3] = { 1024, 1024, 1024};
uint32_t buf2[3] = { 3072, 3072, 3072};
static void vSensorReadTask(void)
{
	while(1)
	{
		DXL_GetRegState(dxlREG_Present_Current, 2);
		xAngle = Gyro_GetCurrAng(&gyroErr);
		xAcc = Gyro_GetCurrAcc(&gyroErr);
		
		if(xServoMsg.bDataReady != RESET)
		{
			DXL_GetPresentParam(&xServoMsg);
		}

		printf("acc_x: %3.2f, acc_x: %3.2f, acc_x: %3.2f\r\n",
										xAcc.x, xAcc.y, xAcc.z);
		printf("roll: %3.2f, pitch: %3.2f, yaw: %3.2f\r\n",
										xAngle.roll, xAngle.pitch, xAngle.yaw);
		printf("%3.2f\r\n", fServoStatusBuf[COL_Current][0]);
		
		vTaskDelay(50);
	}
	
}

static void vServoCtrlTask(void)
{
	uint32_t pos1 = 1024, pos2 = 3072;
	uint8_t flag = 0;
	
	while(1)
	{
		taskENTER_CRITICAL();
		if(flag == 0)
		{
			DXL_SetAllGoalPos(buf1);
			flag = 1;
		}
		
		else if(flag == 1)
		{
			DXL_SetAllGoalPos(buf2);
			flag = 0;
		}
		
		taskEXIT_CRITICAL();
		


		vTaskDelay(1000);
	}
	
}

static void vLedTask(void)
{
	while(1)
	{
		LED = ~LED;
		
		vTaskDelay(300);
	}
}


void vApplicationMallocFailedHook( void )
{
    while(1) blink(2, 100);
}



int main(void)
{	
	
	SpiErr_t xSpiErr = 0;
	NrfStatus_t xNrfStatus = 0;
	uint8_t val = 0;
	uint8_t status = 0;
	
	delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	DXL_ServoInit(eBD2M, ENABLE);
	USART1_Init(921600);
	Gyro_Init();
	LED_Init();
	Nrf_GPIO_Init();
	SPI1_Init();
	
	blink(10, 100);
	
	
	

//	xTaskCreate( ( TaskFunction_t ) vSensorReadTask, "sensorRead", 1024, 
//								( void * ) NULL, 3, &readTask_Handler);
//	xTaskCreate( ( TaskFunction_t ) vServoCtrlTask, "servoCtrl", 200, 
//								( void * ) NULL, 4, &readTask_Handler);
//	xTaskCreate( ( TaskFunction_t ) vLedTask, "blink", 20, 
//								( void * ) NULL, 5, &readTask_Handler);
//	
//    vTaskStartScheduler();   


	/* 	正常情况下，系统不会到达这里，如果到达这里，可能原因是任务的栈空间太小，
		或者系统分配的堆大小不足*/
				
	/*	快速闪烁LED以指示错误*/
	
	while(1)
	{
//		xNrfStatus = Nrf_RegSingleRead(nrfREG_RF_CH, &val, &xSpiErr);
//		
//		printf("status: %02X, spi_err: %02X\r\n", xNrfStatus, xSpiErr);
//		printf("val: %d\r\n\r\n", val);
//		
//		xSpiErr = 0;
//		delay_ms(500);
		
//		Nrf_SetCSN_Low();
//		
//		while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET){}//ֈսע̍ȸࠕ  
//		SPI_SendData(SPI1, 0x05); //ͨڽ΢ʨSPIxע̍һٶbyte  ˽ߝ

//		while (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET){} //ֈսޓ˕Ϊһٶbyte  
//		status = SPI_ReceiveData(SPI1); //׵ܘͨڽSPIxخ޼ޓ˕ք˽ߝ	
////		SPI_ClearFlag(SPI1, SPI_FLAG_RXNE);
//			
////		while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET){}//ֈսע̍ȸࠕ  
//		SPI_SendData(SPI1, 0x05); //ͨڽ΢ʨSPIxע̍һٶbyte  ˽ߝ

//		while (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET){} //ֈսޓ˕Ϊһٶbyte  
//		val = ( uint8_t ) SPI_ReceiveData(SPI1); //׵ܘͨڽSPIxخ޼ޓ˕ք˽ߝ	
// 		 
//		Nrf_SetCSN_High();

		status = Nrf_RegSingleRead(nrfREG_RF_CH, &val, &xSpiErr);
			
		printf("status: %d, value: %d\r\n", status, val);
		delay_ms(500);
	
		
	}
	while(1) blink(10, 100);
}
 

 
