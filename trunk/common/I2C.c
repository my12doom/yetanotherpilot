#include "I2C.h"
#include <stm32f10x_i2c.h>
#include <stdio.h>
#include "timer.h"
#include "i2c_sw.h"
#include "printf.h"

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

int I2C_Reset()
{
	// change GPIO into Out Push-pull mode
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);       

	// disable I2C2 and bit banging reset
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, DISABLE);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	delayus(10);
	GPIO_ResetBits(GPIOB, GPIO_Pin_11);
	delayus(10);
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
	delayus(10);
	GPIO_SetBits(GPIOB, GPIO_Pin_11);
	delayus(10);

	// Change GPIO back and re-enable I2C2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);       
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
		
  RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
	
	return 0;
}

int waitFlag_and_reset(I2C_TypeDef* I2Cx, int flag)
{
	int timeout = 5000;
	
	while (I2C_GetFlagStatus(I2Cx, flag))
	{
		timeout--;
		if (timeout == 0)
		{
			TRACE("I2C Flag Error, resetting\r\n");
			I2C_Reset();

			return -1;
		}
	}

	return 0;
}
int checkEvent_and_reset(I2C_TypeDef* I2Cx, int event)
{
	int timeout = 5000;
	
	while (!I2C_CheckEvent(I2Cx, event))
	{
		timeout--;
		if (timeout == 0)
		{
			TRACE("I2C Error, resetting\r\n");
			I2C_Reset();

			return -1;			
		}
	}

	return 0;
}

int I2C_init(u8 OwnAddress1)
{
#ifndef SW_I2C
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* PB10,11 SCL and SDA */  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);       

	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = OwnAddress1;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_Cmd(I2C2, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_AcknowledgeConfig(I2C2, ENABLE);

	return I2C_Reset();
#else
	I2C2_SW_Configuration();
	return 0;
#endif
}

#ifdef SW_I2C
int I2C_ReadReg(u8 SlaveAddress, u8 startRegister, u8*out, int count)
{
	return I2C_SW_ReadReg(SlaveAddress, startRegister, out, count);
}

int I2C_WriteReg(u8 SlaveAddress, u8 Register, u8 data)
{
	return I2C_SW_WriteByte(SlaveAddress, Register, data);
}
int I2C_WriteRegs(u8 SlaveAddress, u8 startRegister, const u8*data, int count)
{
	return I2C_SW_WriteReg(SlaveAddress, startRegister, data, count);
}
#else
int I2C_ReadReg(u8 SlaveAddress, u8 startRegister, u8*out, int count)
{
	int i;

	// wait for I2C bus becomes free
	if(waitFlag_and_reset(I2C2, I2C_FLAG_BUSY) < 0)
		return -1; 

	I2C_AcknowledgeConfig(I2C2, ENABLE); 

	I2C_GenerateSTART(I2C2, ENABLE); 

	if (checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_MODE_SELECT)<0)
		return -1;

	I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Transmitter); 

	if (checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)<0)
		return -1;

	I2C_SendData(I2C2, startRegister);
	if (checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)<0)
		return -1;

	I2C_GenerateSTART(I2C2, ENABLE);
	if (checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_MODE_SELECT)<0)
		return -1; 

	I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Receiver);
	if (checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)<0)
		return -1; 

	for(i=0; i<count; i++)
	{
		// disable auto ack for last byte, and send a nack instead
		if (i==count-1)
		{
			I2C_AcknowledgeConfig(I2C2, DISABLE);
			I2C_GenerateSTOP(I2C2, ENABLE);
		}

		if (checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
			return -1;
		out[i] = I2C_ReceiveData(I2C2);
	}

	// reenable auto ack
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	
	return 0;
}


int I2C_WriteReg(u8 SlaveAddress, u8 Register, u8 data)
{
	// wait for bus
	if (waitFlag_and_reset(I2C2, I2C_FLAG_BUSY)<0)
		return -1;
	
	// start
	I2C_GenerateSTART(I2C2, ENABLE);
	if (!checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_MODE_SELECT)<0)
		return -1; 

	// send slave address
	I2C_Send7bitAddress(I2C2, SlaveAddress, I2C_Direction_Transmitter);
	if (!checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)<0)
		return -1;

	// send Register address
	I2C_SendData(I2C2, Register);
	if (!checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)<0)
		return -1;


	// send data
	I2C_SendData(I2C2, data); 
	if (!checkEvent_and_reset(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)<0)
		return -1;


	// stop
	I2C_GenerateSTOP(I2C2, ENABLE);	
	return 0;
}
#endif
