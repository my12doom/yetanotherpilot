#include "../mcu.h"
#include "i2c_sw.h"
#include "i2c_sw_priv.h"


volatile int SCL_PIN = DEFAULT_SCL_PIN;
GPIO_TypeDef * volatile SCL_PORT = DEFAULT_SCL_PORT;
volatile int SDA_PIN = DEFAULT_SDA_PIN;
GPIO_TypeDef * volatile SDA_PORT = DEFAULT_SDA_PORT;

#ifdef STM32F1
#define SCL_HI     (SCL_PORT->BSRR = SCL_PIN)
#define SCL_LO     (SCL_PORT->BRR  = SCL_PIN)
#define SDA_HI     (SDA_PORT->BSRR = SDA_PIN)
#define SDA_LO     (SDA_PORT->BRR  = SDA_PIN)
#define SDA_STATE  (SDA_PORT->IDR  & SDA_PIN)
#define SCL_STATE  (SCL_PORT->IDR  & SCL_PIN)
#endif

#ifdef STM32F4
#define SCL_HI     (SCL_PORT->BSRRL = SCL_PIN)
#define SCL_LO     (SCL_PORT->BSRRH  = SCL_PIN)
#define SDA_HI     (SDA_PORT->BSRRL = SDA_PIN)
#define SDA_LO     (SDA_PORT->BSRRH  = SDA_PIN)
int SDA_STATE2()
{
	volatile int v;
SDA_PORT->MODER|=GPIO_Mode_IN<<28;
v = SDA_PORT->IDR  & SDA_PIN;
SDA_PORT->MODER|=GPIO_Mode_OUT<<28;
	return v;
}
int SCL_STATE2()
{
	volatile int v;
	SCL_PORT->MODER|=GPIO_Mode_IN<<26;
	v = SCL_PORT->IDR  & SCL_PIN;
	SCL_PORT->MODER|=GPIO_Mode_OUT<<26;
	return v;
}
#define SCL_STATE SCL_STATE2()
#define SDA_STATE SDA_STATE2()
#endif

static uint8_t I2C_SCLHigh(void);

void I2C2_SW_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure = {0};

    /* sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
#ifdef STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
#endif
#ifdef STM32F4
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
#endif

    /* GPIO configuration */
    /* Configure sEE_I2C pins: SCL & SDA*/
    GPIO_InitStructure.GPIO_Pin = SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef STM32F1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#endif
    GPIO_Init(SCL_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = SDA_PIN;
    GPIO_Init(SDA_PORT, &GPIO_InitStructure);
	
    I2C_Stop();
}

int I2C_SW_WriteByte(uint8_t deviceAddr, uint8_t writeReg, uint8_t writeValue)
{
    if (!I2C_Start()) {
        return -1;
    }

    I2C_SendByte(deviceAddr&0xFE);

    if (!I2C_WaitAck()) {
        I2C_Stop();
        return -1;
    }

    I2C_SendByte(writeReg&0xFF);
    I2C_WaitAck();

    I2C_SendByte(writeValue&0xFF);
    I2C_WaitAck();

    I2C_Stop();

	return 0;
}

int I2C_SW_WriteReg(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count)
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

uint8_t I2C_SW_ReadByte(uint8_t deviceAddr, uint8_t readReg)
{
    uint8_t readValue = 0xFF;

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


int I2C_SW_ReadReg(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count)
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
	#ifdef STM32F1
    volatile int speedTick = 5;
	#endif
	#ifdef STM32F4
    volatile int speedTick = 25;
	#endif
    while (speedTick) {
       speedTick--;
    }
}

static uint8_t I2C_Start(void)
{
    SDA_HI;
    I2C_SCLHigh();
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
    I2C_SCLHigh();
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
    I2C_SCLHigh();
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
    I2C_SCLHigh();
    I2C_Delay();
    SCL_LO;
    I2C_Delay();
}

static uint8_t I2C_WaitAck(void)
{
    SCL_LO;
    I2C_Delay();
    SDA_HI;
    I2C_Delay();
    I2C_SCLHigh();
    I2C_Delay();
    if (SDA_STATE) {
      SCL_LO;
      return 0;
    }
    SCL_LO;
    return 1;
}

static uint8_t I2C_SCLHigh(void)
{
	static int retry = 25;
	SCL_HI;
	while(retry && SCL_STATE)
		retry --;

	return retry > 0;
}

static void I2C_SendByte(uint8_t Data)
{
    uint8_t i = 8;
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
        I2C_SCLHigh();
        I2C_Delay();
    }
    SCL_LO;
}

static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t Data = 0;
    SDA_HI;
    while (i--) {
        Data <<= 1;
        SCL_LO;
        I2C_Delay();
        I2C_SCLHigh();
        I2C_Delay();
        if (SDA_STATE) {
            Data |= 0x01;
        }
    }
    SCL_LO;
    return Data;
}
