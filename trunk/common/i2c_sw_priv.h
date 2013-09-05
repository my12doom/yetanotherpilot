#include <stm32f10x.h>

#define SCL_HI     GPIOA->BSRR = GPIO_Pin_12
#define SCL_LO     GPIOA->BRR  = GPIO_Pin_12
#define SDA_HI     GPIOA->BSRR = GPIO_Pin_15
#define SDA_LO     GPIOA->BRR  = GPIO_Pin_15
#define SDA_STATE  GPIOA->IDR  & GPIO_Pin_15

static void I2C_Delay(void);

static u8 I2C_Start(void);

static void I2C_Stop(void);

static void I2C_SendAck(void);

static void I2C_SendNoAck(void);

static u8 I2C_WaitAck(void);

static void I2C_SendByte(u8 Data);

static u8 I2C_ReceiveByte(void);
