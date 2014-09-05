#ifndef __PPM_H__
#define __PPM_H__

#include "../mcu.h"


// constants and enums
extern float g_ppm_input[6];
extern int64_t g_ppm_input_update[6];
extern uint16_t g_ppm_output[8];
extern float ppm_static[8][2];

enum PPM_OUTPUT_CHANNEL
{
	PPM_OUTPUT_CHANNEL0 = 0x1,
	PPM_OUTPUT_CHANNEL1 = 0x2,
	PPM_OUTPUT_CHANNEL2 = 0x4,
	PPM_OUTPUT_CHANNEL3 = 0x8,
	PPM_OUTPUT_CHANNEL4 = 0x10,
	PPM_OUTPUT_CHANNEL5 = 0x20,
	PPM_OUTPUT_CHANNEL6 = 0x40,
	PPM_OUTPUT_CHANNEL7 = 0x80,
	PPM_OUTPUT_CHANNEL_ALL = 0xff,
};

#ifdef __cplusplus
extern "C" {
#endif
	
// configure PPM input and output
// PPM output is 100Hz
// it always configure TIM3 with REMAP  = 10, output pin = PB0, PB1, PB4, PB5
// if input disabled, it configure TIM4 with REMAP  = 00, output pin = PB6~9
// if input enabled, those pins are connected to EXTI6~9, and TIM4 is used as input timer
// ppm time units are 1/1000 ms
void PPM_init();

// update the specified output channel
void PPM_update_output_channel(int channel_to_update);
	

// reset ppm input statics, for RC range calibration usage
void PPM_reset_static(void);

#ifdef __cplusplus
}
#endif

#endif
