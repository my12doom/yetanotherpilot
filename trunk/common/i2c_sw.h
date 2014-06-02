#ifndef __I2C_SW_H__
#define __I2C_SW_H__
#include "../mcu.h"

#ifdef __cplusplus
extern "C" {
#endif

void I2C2_SW_Configuration(void);
int I2C_SW_WriteByte(uint8_t deviceAddr, uint8_t writeReg, uint8_t writeValue);
uint8_t I2C_SW_ReadByte(uint8_t deviceAddr, uint8_t readReg);
int I2C_SW_ReadReg(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count);
int I2C_SW_WriteReg(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count);

#ifdef __cplusplus
}
#endif

#endif
