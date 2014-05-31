#ifndef __I2C_H__
#define __I2C_H__

#include "../mcu.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// return 0 on success, -1 on error
int I2C_init(uint8_t OwnAddress1);
int I2C_ReadReg(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count);
int I2C_WriteReg(uint8_t SlaveAddress, uint8_t Register, uint8_t data);
int I2C_WriteRegs(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count);

#ifdef __cplusplus
}
#endif

#endif
