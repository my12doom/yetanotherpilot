#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif
void ADC1_Init(void);
int ADC1_Read(void);
#ifdef __cplusplus
}
#endif

#endif
