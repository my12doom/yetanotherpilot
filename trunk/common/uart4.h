#ifndef __UART4_H
#define	__UART4_H

#include "../mcu.h"

#ifdef __cplusplus
extern "C" {
#endif
void UART4_Init(uint32_t baud_rate, int PC10_11);
int UART4_ReadPacket(void *out, int max_size);			// return size of packet read, 
														// -1 when not enough data, -2 if packet exceeded buffer size, -3 on other errors.
int UART4_SendPacket(const void *buf, int size);

#ifdef __cplusplus
}
#endif

#endif
