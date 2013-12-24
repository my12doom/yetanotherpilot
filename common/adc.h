#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif
void ADC1_Init(void);
void ADC1_SelectPin(uint16_t GPIO_Pin);		//only GPIOA are supported, so we use only one parameter
int ADC1_Read(void);
#ifdef __cplusplus
}
#endif

#endif
