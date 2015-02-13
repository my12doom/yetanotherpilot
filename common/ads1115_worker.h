#pragma once

#include "mcu.h"
#include "ads1115.h"

#define MAX_ADS1115_WORKS 8

typedef struct
{
	ads1115_speed speed;
	ads1115_channel channel;
	ads1115_gain gain;
	int16_t *out;
}ads1115_work;


#ifdef __cplusplus
extern "C" {
#endif

int ads1115_new_work(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, int16_t *out);
int ads1115_go_on(void);

#ifdef __cplusplus
}

#endif
