#include <stm32f10x.h>

#define SCL_HI     GPIOB->BSRR = GPIO_Pin_10
#define SCL_LO     GPIOB->BRR  = GPIO_Pin_10
#define SDA_HI     GPIOB->BSRR = GPIO_Pin_11
#define SDA_LO     GPIOB->BRR  = GPIO_Pin_11
#define SDA_STATE  GPIOB->IDR  & GPIO_Pin_11

static void I2C_Delay(void);

static u8 I2C_Start(void);

static void I2C_Stop(void);

static void I2C_SendAck(void);

static void I2C_SendNoAck(void);

static u8 I2C_WaitAck(void);

static void I2C_SendByte(u8 Data);

static u8 I2C_ReceiveByte(void);
