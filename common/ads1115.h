#pragma once

#include "I2C.h"

typedef enum
{
	ads1115_speed_8sps = 0,
	ads1115_speed_16sps = 1,
	ads1115_speed_32sps = 2,
	ads1115_speed_64sps = 3,
	ads1115_speed_128sps = 4,
	ads1115_speed_250sps = 5,
	ads1115_speed_475sps = 6,
	ads1115_speed_860sps = 7,

} ads1115_speed;

typedef enum
{
	ads1115_channnel_AIN0_AIN1 = 0,		// differential
	ads1115_channnel_AIN0_AIN3 = 1,
	ads1115_channnel_AIN1_AIN3 = 2,
	ads1115_channnel_AIN2_AIN3 = 3,
	ads1115_channnel_AIN0 = 4,			// single ends
	ads1115_channnel_AIN1 = 5,
	ads1115_channnel_AIN2 = 6,
	ads1115_channnel_AIN3 = 7,
} ads1115_channel;

typedef enum
{
	ads1115_gain_6V = 0,	// +-6.144V
	ads1115_gain_4V = 1,	// +-4.096V
	ads1115_gain_2V = 2,	// +-2.048V
	ads1115_gain_1V = 3,	// +-1.024V
	ads1115_gain_512 = 4,	// +-0.512V
	ads1115_gain_256 = 5,	// +-0.256V
	ads1115_gain_256_2 = 6,	// +-0.256V
	ads1115_gain_256_3 = 7,	// +-0.256V
} ads1115_gain;

typedef enum
{
	ads1115_mode_continuous = 0,
	ads1115_mode_singleshot = 1,
} ads1115_mode;

#ifdef __cplusplus
extern "C" {
#endif

int ads1115_init(void);
int ads1115_config(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, ads1115_mode mode);
int ads1115_startconvert(void);
int ads1115_getresult(short *result);		// return -1 if still converting, 0 if conversion completed or continuous mode, further calls return the last conversion result.
short ads1115_convert(void);				// a simplfied version which start a new conversion ,wait it to complete and returns the result directly

#ifdef __cplusplus
}
#endif
