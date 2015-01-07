#ifndef __USART1_H
#define	__USART1_H

#include <stdio.h>
#include "common.h"
#include "build.h"

#ifndef WIN32
#include "../mcu.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void printf_init(void);


#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

#define gcc_printf(...)\
{\
	int iiii;\
	int cccc = sprintf(gcc_printf_buffer, __VA_ARGS__);\
	for(iiii=0; iiii<cccc; iiii++)\
{\
	USART_SendData(USART1, (unsigned char) gcc_printf_buffer[iiii]);\
	while (!(USART1->SR & USART_FLAG_TXE));\
	if (DEMCR & TRCENA) \
{\
	while (ITM_Port32(0) == 0);\
	ITM_Port8(0) = gcc_printf_buffer[iiii];\
}\
}\
}

#if _TRACE == 1 && __GNUC__ > 0
#define TRACE gcc_printf
#elif _TRACE == 1
#define TRACE printf
#else
#define TRACE(...)
#endif

#if __GNUC__ > 0
extern char gcc_printf_buffer[256];
#define LOGE gcc_printf
#else
#define LOGE printf
#endif

#ifdef __cplusplus
}
#endif

#endif /* __USART1_H */
