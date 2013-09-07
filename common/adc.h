#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"

extern __IO uint16_t ADC_ConvertedValue;

#ifdef __cplusplus
extern "C" {
#endif
void ADC1_Init(void);
#ifdef __cplusplus
}
#endif

#endif
