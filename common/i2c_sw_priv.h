#include "mcu.h"

#define DEFAULT_SDA_PORT GPIOC
#define DEFAULT_SCL_PORT GPIOC
#define DEFAULT_SDA_PIN GPIO_Pin_14
#define DEFAULT_SCL_PIN GPIO_Pin_13

static void I2C_Delay(void);

static uint8_t I2C_Start(void);

static void I2C_Stop(void);

static void I2C_SendAck(void);

static void I2C_SendNoAck(void);

static uint8_t I2C_WaitAck(void);

static void I2C_SendByte(uint8_t Data);

static uint8_t I2C_ReceiveByte(void);
