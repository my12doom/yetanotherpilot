#include <stm32f10x.h>

#if 1

#define DEFAULT_SDA_PORT GPIOC
#define DEFAULT_SCL_PORT GPIOC
#define DEFAULT_SDA_PIN GPIO_Pin_14
#define DEFAULT_SCL_PIN GPIO_Pin_13

#else

#define DEFAULT_SDA_PORT GPIOA
#define DEFAULT_SCL_PORT GPIOA
#define DEFAULT_SDA_PIN GPIO_Pin_3
#define DEFAULT_SCL_PIN GPIO_Pin_2

#endif

static void I2C_Delay(void);

static u8 I2C_Start(void);

static void I2C_Stop(void);

static void I2C_SendAck(void);

static void I2C_SendNoAck(void);

static u8 I2C_WaitAck(void);

static void I2C_SendByte(u8 Data);

static u8 I2C_ReceiveByte(void);
