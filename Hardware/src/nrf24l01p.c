/**
  ***********************************UTF-8**************************************
  * @file    nrf24l01p.c
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于实现Nrf24L01+的收发，模块型号为EBYTE-ML01DP5
  ******************************************************************************  
  */ 
  
  /*-------------STM32F407VET6 与 EBYTE-ML01DP5的接线---------------------------
				   _______________________________
				  |               |               |
				  | STM32F407VET6 | EBYTE-ML01DP5 |
				  |_______________|_______________|
				  |      PA4      |      CSN      |
				  |---------------|---------------|
				  |      PA5      |      SCK      |
				  |---------------|---------------|
				  |      PA6      |      MISO     |
				  |---------------|---------------|
				  |      PA7      |      MOSI     |
				  |---------------|---------------|
				  |      PC4      |      CE       |
				  |---------------|---------------|
				  |      PB0      |      IRQ      |
				  |_______________|_______________|
  ----------------------------------------------------------------------------*/
  
#include "nrf24l01p.h"


/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化nrf24l01+ 相连引脚
	* @param  	None
	* @retval 	None
	*/
static void Nrf_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(nrfRCC, ENABLE);
	
	/* 配置nrf24l01+模块CSN引脚 */
	GPIO_InitStructure.GPIO_Pin 	= nrfIO_CSN_Pin;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(nrfIO_CSN_GPIO, &GPIO_InitStructure);
	
	/* 配置nrf24l01+模块CE引脚 */
	GPIO_InitStructure.GPIO_Pin 	= nrfIO_CE_Pin;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(nrfIO_CE_GPIO, &GPIO_InitStructure);
	
	/* 配置nrf24l01+模块IRQ引脚 */
	GPIO_InitStructure.GPIO_Pin 	= nrfIO_IRQ_Pin;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_Init(nrfIO_IRQ_GPIO, &GPIO_InitStructure);
	
	/* 配置PC5引脚，正常不需要配置 */
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化nrf24l01+ 外部中断
	* @param  	None
	* @retval 	None
	*/
static void Nrf_IRQ_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = nrfEXTI_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt to the highest priority */
    NVIC_InitStructure.NVIC_IRQChannel = nrfEXTI_Line;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    EXTI_ClearITPendingBit(nrfEXTI_Line);

}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 外部中断服务函数
	* @param  	None
	* @retval 	None
	*/
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		
		
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	初始化nrf24l01+ 的相关引脚和外部中断，以及SPI1
	* @param  	NrfInitStruct：nrf24l01+初始化结构体
	* @retval 	nrf24l01+状态寄存器值
	*/
NrfStatusType_t Nrf_Init(NRF_InitTypeDef* NrfInitStruct)
{
	uint8_t ucConfig = 0;
	SpiErrType_t xErr = 0;
	NrfStatusType_t xStatus = 0;
	/*初始化nrf24l01+相连引脚*/
	Nrf_GPIO_Init();
	/*初始化nrf24l01+ 外部中断*/
	Nrf_IRQ_Init();
	/*初始化nrf24l01+ SPI*/
	SPI1_Init();
	
	ucConfig =   NrfInitStruct->NRF_Mask_RX_DR			|\
				 NrfInitStruct->NRF_Mask_TX_DS			|\
				 NrfInitStruct->NRF_Mask_MAX_RT			|\
				 NrfInitStruct->NRF_EN_CRC				|\
				 NrfInitStruct->NRF_CRC_Coding_Bytes	|\
				 NrfInitStruct->NRF_PWR_Manage			|\
				 NrfInitStruct->NRF_PRIM_RX;
	
	xStatus = Nrf_RegSingleWrite(nrfREG_CONFIG, ucConfig, &xErr);
	
	return xStatus;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	使能nrf24l01+ CE引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_SetCE_HIGH(void)
{
	GPIO_SetBits(nrfIO_CE_GPIO, nrfIO_CE_Pin);
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	失能nrf24l01+ CE引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_SetCE_LOW(void)
{
	GPIO_ResetBits(nrfIO_CE_GPIO, nrfIO_CE_Pin);
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	使能nrf24l01+ CSN引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_EnableCSN(void)
{
	GPIO_SetBits(nrfIO_CSN_GPIO, nrfIO_CSN_Pin);
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	失能nrf24l01+ CSN引脚
	* @param  	None
	* @retval 	None
	*/
void Nrf_DisableCSN(void)
{
	GPIO_ResetBits(nrfIO_CSN_GPIO, nrfIO_CSN_Pin);
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 单个寄存器写入,如果写入过程无误，返回值最高为应为0，
				否则为1
	* @param  	ucRegAddr：寄存器地址
	* @param  	ucSrc：待写入的值
	* @param  	pErr：存储错误信息的指针
	* @retval 	nrf24l01+ 状态寄存器值
	*/
NrfStatusType_t Nrf_RegSingleWrite(uint8_t ucRegAddr, uint8_t ucSrc, 
									SpiErrType_t* pErr)
{
	NrfStatusType_t xNrfStatus = 0;
	
	Nrf_SetCSN_LOW();
	
	/*发送写寄存器操作和寄存器地址*/
	SPI_Write(nrfSPI, nrfCMD_W_REG|ucRegAddr, pErr);	
	/*读nrf24l01+ status值*/
	xNrfStatus |= SPI_Read(nrfSPI, pErr);
	/*发送新的寄存器值*/
	SPI_Write(nrfSPI, ucSrc, pErr);	
	
	Nrf_SetCSN_HIGH();
	
	return xNrfStatus;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 单个寄存器读取,如果写入过程无误，返回值最高为应为0，
				否则为1
	* @param  	ucRegAddr：寄存器地址
	* @param  	ucSrc：待写入的值
	* @param  	pErr：存储错误信息的指针
	* @retval 	nrf24l01+ 状态寄存器值
	*/
NrfStatusType_t Nrf_RegSingleRead(uint8_t ucRegAddr, uint8_t* pucDst, 
									SpiErrType_t* pErr)
{
	NrfStatusType_t xNrfStatus = 0;
	Nrf_SetCSN_LOW();
	
	/*发送读寄存器操作和寄存器地址*/
	SPI_Write(nrfSPI, nrfCMD_R_REG|ucRegAddr, pErr);	
	/*读nrf24l01+ status值*/
	xNrfStatus |= SPI_Read(nrfSPI, pErr);
	/*读取寄存器值*/
	*pucDst = SPI_Read(nrfSPI, pErr);
		
	Nrf_SetCE_HIGH();
	
	return xNrfStatus;
}
/* ---------------------------------------------------------------------------*/		
											   
/****
	* @brief	nrf24l01+ 多个寄存器写入
				否则为1
	* @param  	ucRegAddr：起始寄存器地址
	* @param  	ucSrcBuf：待写入的值数组
	* @param  	ucLen：数组长度
	* @param  	pErr：存储错误信息的指针
	* @retval 	nrf24l01+ 状态寄存器值
	*/
NrfStatusType_t Nrf_RegMultiWrite(uint8_t ucRegAddr, uint8_t* ucSrcBuf, 
									uint8_t ucLen, SpiErrType_t* pErr)	
{
	
}