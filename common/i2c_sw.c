#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include "i2c_sw.h"
#include "i2c_sw_priv.h"


volatile int SCL_PIN = DEFAULT_SCL_PIN;
GPIO_TypeDef * volatile SCL_PORT = DEFAULT_SCL_PORT;
volatile int SDA_PIN = DEFAULT_SDA_PIN;
GPIO_TypeDef * volatile SDA_PORT = DEFAULT_SDA_PORT;


#define SCL_HI     (SCL_PORT->BSRR = SCL_PIN)
#define SCL_LO     (SCL_PORT->BRR  = SCL_PIN)
#define SDA_HI     (SDA_PORT->BSRR = SDA_PIN)
#define SDA_LO     (SDA_PORT->BRR  = SDA_PIN)
#define SDA_STATE  (SDA_PORT->IDR  & SDA_PIN)

void I2C2_SW_Configuration(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* GPIO configuration */
    /* Configure sEE_I2C pins: SCL & SDA*/
    GPIO_InitStructure.GPIO_Pin = SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(SCL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = SDA_PIN;
    GPIO_Init(SDA_PORT, &GPIO_InitStructure);
	
    I2C_Stop();
}

void I2C_SW_WriteByte(u8 deviceAddr, u8 writeReg, u8 writeValue)
{
    if (!I2C_Start()) {
        return;
    }

    I2C_SendByte(deviceAddr&0xFE);

    if (!I2C_WaitAck()) {
        I2C_Stop();
        return;
    }

    I2C_SendByte(writeReg&0xFF);
    I2C_WaitAck();

    I2C_SendByte(writeValue&0xFF);
    I2C_WaitAck();

    I2C_Stop();
}

int I2C_SW_WriteReg(u8 SlaveAddress, u8 startRegister, const u8*data, int count)
{
    int i;
    if (!I2C_Start()) {
        return -1;
    }

    I2C_SendByte(SlaveAddress&0xFE);

    if (!I2C_WaitAck()) {
        I2C_Stop();
        return -1;
    }

    I2C_SendByte(startRegister&0xFF);
    I2C_WaitAck();

    for(i=0; i<count; i++)
    {
        I2C_SendByte(data[i]&0xFF);
        
        if (!I2C_WaitAck())
        {
            I2C_Stop();
            return -1;
        }
    }

    I2C_Stop();

    return 0;
}

u8 I2C_SW_ReadByte(u8 deviceAddr, u8 readReg)
{
    u8 readValue = 0xFF;

    if (!I2C_Start()) {
        return 0xFF;
    }

    I2C_SendByte(deviceAddr&0xFE);

    if (!I2C_WaitAck()) {
        I2C_Stop();
        return 0xFF;
    }

    I2C_SendByte(readReg&0xFF);
    I2C_WaitAck();

    I2C_Start();
    I2C_SendByte((deviceAddr&0xFE)|0x01);
    I2C_WaitAck();

    readValue = I2C_ReceiveByte();
    I2C_SendNoAck();

    I2C_Stop();
    return readValue;
}


int I2C_SW_ReadReg(u8 SlaveAddress, u8 startRegister, u8*out, int count)
{	
	int i;

	if (!I2C_Start()) {
			return 0xFF;
	}

	I2C_SendByte(SlaveAddress&0xFE);

	if (!I2C_WaitAck()) {
			I2C_Stop();
			return 0xFF;
	}

	I2C_SendByte(startRegister);
	I2C_WaitAck();

	I2C_Start();
	I2C_SendByte((SlaveAddress&0xFE)|0x01);
	I2C_WaitAck();

	for(i=0; i<count; i++)
	{
		out[i] = I2C_ReceiveByte();
		if (i==count-1)
			I2C_SendNoAck();
		else
			I2C_SendAck();
	}

	I2C_Stop();
	
	return 0;
}


static void I2C_Delay(void)
{
    int speedTick = 5;
    while (speedTick) {
       speedTick--;
    }
}

static u8 I2C_Start(void)
{
    SDA_HI;
    SCL_HI;
    I2C_Delay();
    if (!SDA_STATE) {
        //DB_Print("I2C_Start:BUSY!\n");
        return 0;
    }
    SDA_LO;
    I2C_Delay();
    if (SDA_STATE) {
        //DB_Print("I2C_Start:BUS ERROR!\n");
        return 0;
    }
    SDA_LO;
    I2C_Delay();
    return 1;
}

static void I2C_Stop(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_LO;
    I2C_Delay();
    SCL_HI;
    I2C_Delay();
    SDA_HI;
    I2C_Delay();
}

static void I2C_SendAck(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_LO;
    I2C_Delay();
    SCL_HI;
    I2C_Delay();
    SCL_LO;
    I2C_Delay();
}

static void I2C_SendNoAck(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_HI;
    I2C_Delay();
    SCL_HI;
    I2C_Delay();
    SCL_LO;
    I2C_Delay();
}

static u8 I2C_WaitAck(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_HI;
    I2C_Delay();
    SCL_HI;
    I2C_Delay();
    if (SDA_STATE) {
      SCL_LO;
      return 0;
    }
    SCL_LO;
    return 1;
}

static void I2C_SendByte(u8 Data)
{
    u8 i = 8;
    while (i--) {
        SCL_LO;
        I2C_Delay();
        if (Data&0x80) {
            SDA_HI;
        }
        else {
            SDA_LO;
        }
        Data <<= 1;
        I2C_Delay();
        SCL_HI;
        I2C_Delay();
    }
    SCL_LO;
}

static u8 I2C_ReceiveByte(void)
{
    u8 i = 8;
    u8 Data = 0;
    SDA_HI;
    while (i--) {
        Data <<= 1;
        SCL_LO;
        I2C_Delay();
        SCL_HI;
        I2C_Delay();
        if (SDA_STATE) {
            Data |= 0x01;
        }
    }
    SCL_LO;
    return Data;
}
