#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif
void ADC1_Init(void);
int ADC1_SelectChannel(uint16_t adc_channel);			//  returns 1 on success, 0 on error
													// only ADC channel 0~9 are supported, GPIO A0~A7 B0 B1
int ADC1_Read(void);
#ifdef __cplusplus
}
#endif

#endif
