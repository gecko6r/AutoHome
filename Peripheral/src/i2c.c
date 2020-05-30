/**
  ***********************************UTF-8**************************************
  * @file    i2c.c
  * @author  Xiong
  * @version V1.0
  * @date    02-July-2020
  * @brief   此文件用于定义STM32的IIC操作函数
  ******************************************************************************  
  */ 
  
#include "i2c.h"
#include "usart.h"

static uint32_t usTimeCount;			//用于检测线路超时
static uint16_t IIC_status;

/* ---------------------------------------------------------------------------*/

/****
	* @brief	初始化I2C2
    * @param  	None
    * @retval 	None
    */
void IIC_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    /* GPIO Peripheral clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    /*I2C2 configuration*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2); //
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

    //PB10: I2C2_SCL  PB11: I2C2_SDA
    GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType 	= GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  	= GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* I2C Struct Initialize */
    I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_ClockSpeed 			= 350000;						//I2C时钟速度（低于400kHz）
    I2C_InitStructure.I2C_Mode 					= I2C_Mode_I2C;					//I2C_Config!式
    I2C_InitStructure.I2C_DutyCycle				= I2C_DutyCycle_2;				//
    I2C_InitStructure.I2C_OwnAddress1 			= 0x50;							//
    I2C_InitStructure.I2C_Ack 					= I2C_Ack_Enable;				//
    I2C_InitStructure.I2C_AcknowledgedAddress 	= I2C_AcknowledgedAddress_7bit;	//
    I2C_Init(I2C2, &I2C_InitStructure);
	

    /* I2C Initialize */
    I2C_Cmd(I2C2, ENABLE);
	I2C_StretchClockCmd(I2C2, DISABLE);
	

}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	I2C发送一个字节
    * @param  	I2Cx：I2C外设
    * @param  	ucSlaveAddr：从设备地址	
    * @param	ucRegAddr：寄存器地址
    * @param  	ucData：数据
	* @retval 	0：无错误，非0：有错误
    */
uint8_t I2C_ByteWrite(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,\
					uint8_t ucData )
{
	
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))  //等待I2C
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
		}
	}
	
	I2C_GenerateSTART(I2Cx, ENABLE);  //产生起始信号
	usTimeCount = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_StartTimeOut;
		}
	}
	
	I2C_Send7bitAddress(I2Cx, ucSlaveAddr, I2C_Direction_Transmitter);  //发送设备地址
	usTimeCount = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		usTimeCount++;
		if (usTimeCount > 1000)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendSlaveAddrTimeOut;
		}
	}
	//
	//读取SR2状态寄存器
	//
	IIC_status = I2Cx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	
	I2C_SendData(I2Cx, ucRegAddr);  //发送存储地址
	usTimeCount = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendRegAddrTimeOut;
		}
	}
	I2C_SendData(I2Cx, ucData);  //发送数据
	usTimeCount = 0;
	//
	//EV8_2
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendDataTimeOut;
		}
	}
	I2C_GenerateSTOP(I2Cx, ENABLE);  //产生停止信号
	
	IIC_status = 0;
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	I2C接收一个字节，*err为0时，数据才有效
    * @param  	I2Cx：I2C外设
    * @param  	ucSlaveAddr：从设备地址左移1	
    * @param	ucRegAddr：寄存器地址
    * @param  	err：错误信息存储地址
	* @retval 	0：无错误，非0：有错误
    */
uint8_t I2C_ByteRead(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr,
							uint8_t ucRegAddr, uint8_t *pcDst )
{
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))  //等待I2C
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_Busy;
		}
	}
	I2C_GenerateSTART(I2Cx, ENABLE);  //发送起始信号
	usTimeCount = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_StartTimeOut;
		}
	}
	I2C_Send7bitAddress(I2Cx, ucSlaveAddr, I2C_Direction_Transmitter);  //发送设备地址
	usTimeCount = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendSlaveAddrTimeOut;
		}
	}
	
	IIC_status = I2Cx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	
	I2C_SendData(I2Cx, ucRegAddr);  //发送存储地址
	usTimeCount = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendRegAddrTimeOut;
		}
	}
	I2C_GenerateSTART(I2Cx, ENABLE);  //重启信号
	usTimeCount = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_StartTimeOut;
		}
	}
	I2C_Send7bitAddress(I2Cx, ucSlaveAddr, I2C_Direction_Receiver);  //读取命令
	usTimeCount = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendSlaveAddrTimeOut;
		}
	}
	IIC_status = I2Cx->SR2;
	
	I2C_AcknowledgeConfig(I2Cx, DISABLE);  //发送NACK
	I2C_GenerateSTOP(I2Cx, ENABLE);
	usTimeCount = 0;
	//
	//EV7
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_ReadTimeOut;
		}
	}
	*pcDst = I2C_ReceiveData(I2Cx);
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
	IIC_status = 0;
	return 0;
}

/* ---------------------------------------------------------------------------*/

/****
	* @brief	I2C发送多个字节
    * @param  	I2Cx：I2C外设
    * @param  	ucSlaveAddr：从设备地址	
    * @param	ucRegAddr：寄存器地址
	* @param  	ucNumToWrite：写入数据长度
    * @param	pucBuffer：数据首地址
    * @param  	err：错误信息存储地址
	* @retval 	0：无错误，非0：有错误
    */
uint8_t I2C_MultiWrite(I2C_TypeDef * I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,
							uint8_t ucNumToWrite, uint8_t* pucBuffer )
{
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))  //等待I2C
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_Busy;
		}
	}
	I2C_GenerateSTART(I2Cx, ENABLE);  //产生起始信号
	usTimeCount = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_StartTimeOut;
		}
	}
	I2C_Send7bitAddress(I2Cx, ucSlaveAddr, I2C_Direction_Transmitter);  //发送设备地址
	usTimeCount = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendSlaveAddrTimeOut;
		}
	}
	//
	//读取SR2状态寄存器
	//
	IIC_status = I2Cx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	I2C_SendData(I2Cx, ucRegAddr);  //发送存储地址
	usTimeCount = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendRegAddrTimeOut;
		}
	}
	//
	//循环发送数据
	//
	while (ucNumToWrite--)
	{
		I2C_SendData(I2Cx, *(pucBuffer++));  //发送数据
		usTimeCount = 0;
		//
		//EV8_2
		//
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			usTimeCount++;
			if (usTimeCount > I2C_TIME_OUT)
			{
				I2C_GenerateSTOP(I2Cx, ENABLE); 
				return I2C_ERR_SendDataTimeOut;
			}
		}
	}
	
	I2C_GenerateSTOP(I2Cx, ENABLE);  //产生停止信号
	
	IIC_status = 0;
	return 0;
}
/* ---------------------------------------------------------------------------*/

/****
	* @brief	I2C接收一个字节，*err为0时，数据才有效
    * @param  	I2Cx：I2C外设
    * @param  	ucSlaveAddr：从设备地址	
    * @param	ucRegAddr：寄存器地址
    * @param  	err：错误信息存储地址
	* @retval 	0：无错误，非0：有错误
    */
uint8_t I2C_MultiRead(I2C_TypeDef* I2Cx, uint8_t ucSlaveAddr, uint8_t ucRegAddr,\
					uint8_t ucNumToRead, uint8_t* pucBuffer )
{

	usTimeCount = 0;
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))  //等待I2C
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_Busy;
		}
	}
	I2C_GenerateSTART(I2Cx, ENABLE);  //发送起始信号
	usTimeCount = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_StartTimeOut;
		}
	}
	I2C_Send7bitAddress(I2Cx, ucSlaveAddr, I2C_Direction_Transmitter);  //发送设备地址
	usTimeCount = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			printf("write timeout\r\n");
			return I2C_ERR_SendSlaveAddrTimeOut;
		}
	}
	IIC_status = I2Cx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！
	
	I2C_SendData(I2Cx, ucRegAddr);  //发送存储地址
	usTimeCount = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_SendRegAddrTimeOut;
		}
	}
	I2C_GenerateSTART(I2Cx, ENABLE);  //重启信号
	usTimeCount = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE); 
			return I2C_ERR_StartTimeOut;
		}
	}
	I2C_Send7bitAddress(I2Cx, ucSlaveAddr, I2C_Direction_Receiver);  //读取命令
	usTimeCount = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		usTimeCount++;
		if (usTimeCount > I2C_TIME_OUT)
		{
			I2C_GenerateSTOP(I2Cx, ENABLE);
			printf("send read timeout");	
			return I2C_ERR_SendSlaveAddrTimeOut;
		}
	}
	
	
	
	
	
	IIC_status = I2Cx->SR2;  //软件读取SR1寄存器后,对SR2寄存器的读操作将清除ADDR位，不可少！！！！！！！！！

	while (ucNumToRead)
	{
		
		if (ucNumToRead == 1)  //最后一个数据了，不发送应答信号
		{
			I2C_AcknowledgeConfig(I2Cx, DISABLE);  //发送NACK
			I2C_GenerateSTOP(I2Cx, ENABLE);
		}
		//
		//EV7
		//
		usTimeCount = 0;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			usTimeCount++;
			if (usTimeCount > I2C_TIME_OUT)
			{
				I2C_GenerateSTOP(I2Cx, ENABLE); 
				return I2C_ERR_ReadTimeOut;
			}
		}
		*(pucBuffer++) = I2C_ReceiveData(I2Cx);
		
		ucNumToRead--;
	}
	
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
	IIC_status = 0;
	return 0;
}


