#ifndef __MS5611_H__
#define __MS5611_H__

#ifdef __cplusplus
extern "C" {
#endif

int init_MS5611(void);
int read_MS5611(int *data);

#ifdef __cplusplus
}
#endif

#endif
