#ifndef __GPS_H
#define	__GPS_H

#include "../mcu.h"
#include "../nmea/nmea.h"

#ifdef __cplusplus
extern "C" {
#endif
void GPS_Init(uint32_t baud_rate);
int GPS_ParseBuffer(void);			// return number of valid sentences parsed.
nmeaINFO* GPS_GetInfo(void); 

#ifdef __cplusplus
}
#endif

#endif
