#include "mcu.h"

#if 1


#ifdef LITE
#define DEFAULT_SDA_PORT GPIOB
#define DEFAULT_SCL_PORT GPIOB
#define DEFAULT_SDA_PIN GPIO_Pin_8
#define DEFAULT_SCL_PIN GPIO_Pin_9

#else
#define DEFAULT_SDA_PORT GPIOC
#define DEFAULT_SCL_PORT GPIOC
#define DEFAULT_SDA_PIN GPIO_Pin_14
#define DEFAULT_SCL_PIN GPIO_Pin_13


#endif

#else

#define DEFAULT_SDA_PORT GPIOA
#define DEFAULT_SCL_PORT GPIOA
#define DEFAULT_SDA_PIN GPIO_Pin_3
#define DEFAULT_SCL_PIN GPIO_Pin_2

#endif

static void I2C_Delay(void);

static uint8_t I2C_Start(void);

static void I2C_Stop(void);

static void I2C_SendAck(void);

static void I2C_SendNoAck(void);

static uint8_t I2C_WaitAck(void);

static void I2C_SendByte(uint8_t Data);

static uint8_t I2C_ReceiveByte(void);
