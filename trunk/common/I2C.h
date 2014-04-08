#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f10x.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// return 0 on success, -1 on error
int I2C_init(u8 OwnAddress1);
int I2C_ReadReg(u8 SlaveAddress, u8 startRegister, u8*out, int count);
int I2C_WriteReg(u8 SlaveAddress, u8 Register, u8 data);
int I2C_WriteRegs(u8 SlaveAddress, u8 startRegister, const u8*data, int count);

#ifdef __cplusplus
}
#endif

#endif
