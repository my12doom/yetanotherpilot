#ifndef __I2C_SW_H__
#define __I2C_SW_H__
#include <stm32f10x.h>

#ifdef __cplusplus
extern "C" {
#endif

void I2C2_SW_Configuration(void);
int I2C_SW_WriteByte(u8 deviceAddr, u8 writeReg, u8 writeValue);
u8 I2C_SW_ReadByte(u8 deviceAddr, u8 readReg);
int I2C_SW_ReadReg(u8 SlaveAddress, u8 startRegister, u8*out, int count);
int I2C_SW_WriteReg(u8 SlaveAddress, u8 startRegister, const u8*data, int count);

#ifdef __cplusplus
}
#endif

#endif
