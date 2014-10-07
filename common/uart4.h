#ifndef __UART4_H
#define	__UART4_H

#include "../mcu.h"

#ifdef __cplusplus
extern "C" {
#endif
void UART4_Init(uint32_t baud_rate);
int UART4_ParseBuffer(void);			// return number of valid sentences parsed.
int UART4_Send(void *buf, int size);

#ifdef __cplusplus
}
#endif

#endif
