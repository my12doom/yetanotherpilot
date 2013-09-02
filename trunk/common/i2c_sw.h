#include <stm32f10x.h>

void I2C2_SW_Configuration(void);

void I2C_SW_WriteByte(u8 deviceAddr, u8 writeReg, u8 writeValue);

u8 I2C_SW_ReadByte(u8 deviceAddr, u8 readReg);

int I2C_SW_ReadReg(u8 SlaveAddress, u8 startRegister, u8*out, int count);
