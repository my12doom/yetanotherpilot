#ifndef __CONFIG_H__
#define __CONFIG_H__

// printf debug configuration
#define _TRACE 0
#define ITM_DBG
#define USART1_DBG
#define SW_I2C
#define GPS_BUFFER_BLOCK 512
#define PCB_VERSION 3
//#define EXTERNAL_HMC5883			// different I2C pins
#define EXTERNAL_HMC5883_2			// same I2C pins

#ifndef PCB_VERSION
#define PCB_VERSION 1
#endif

#ifndef STATION
#undef USART1_DBG
#endif

#ifdef BLUETOOTH
#undef USART1_DBG
#define ITM_DBG
#endif

// pilot configuration
#define QUADCOPTER 1
#define PI 3.14159265f
#define cycle_time (3000)

#define RC_TIMEOUT 1000000				// 1 seconds
#define RC_RANGE 400
#define RC_DEAD_ZONE 15
#define RC_CENTER 1520
#define BARO_OSS 50
#define hall_sensor_sensitivity	0.0666		// unit: mV / mA
#define VOLTAGE_DIVIDER_BASE 6		// uncalibrated voltage divider ratio
#define MAX_GYRO_BIAS_DRIFT 30
#define THROTTLE_STOP (max((int)(rc_setting[2][0]-20),1000))
#define THROTTLE_MAX (min((int)(rc_setting[2][2]-20),2000))
#define THROTTLE_CRUISE 0.45f

#define ACRO_ROLL_RATE (PI*3/2)				// 270 degree/s
#define ACRO_PITCH_RATE (PI)			// 180 degree/s
#define ACRO_YAW_RATE (PI/2)			// 90 degree/s

#define ACCELEROMETER_THRESHOLD 0.3f
#define MAG_THRESHOLD 0.3f

#define CRUISING_SPEED 125				// 125 pascal
#define LOG_NRF 1
#define LOG_SDCARD 2
#define LOG_USART1 4
#define LOG_USART2 8
extern int LOG_LEVEL;



#if QUADCOPTER == 1
#define ACRO_MANUAL_FACTOR (0.0)

#else
#define ACRO_MANUAL_FACTOR (0.3)		// final output in acrobatic mode, 70% pid, 30% rc

#endif

#define QUADCOPTER_THROTTLE_RESERVE 0.15




static float ACRO_MAX_OFFSET[3] =
{
(PI/8),		// 22.5 degree, max roll advance before airframe can response in acrobatic mode
(PI/8),	// 22.5 degree, max pitch advance before airframe can response in acrobatic mode
(PI/8),		// 22.5 degree, max yaw advance before airframe can response in acrobatic mode
};

static float FLY_BY_WIRE_MAX_OFFSET[3] = 
{
(PI/4),		// 45 degree, max roll angle
(PI/8),		// 22.5 degree, max pitch angle
(PI/8),		// 22.5 degree, max yaw (what?)
};

static float QUADCOPTER_MAX_YAW_OFFSET = PI/4;
#define QUADCOPTER_ACRO_YAW_RATE (PI)			// 180 degree/s

static int rc_reverse[3] = 								// -1 = reverse, 1 = normal, 0 = disable, won't affect mannual mode
{
	1,			// roll
	1,			// pitch
	1,			// yaw
};

static int sensor_reverse[3] = 						// -1 = reverse, 1 = normal, 0 = disable, won't affect mannual mode
{
	1,			// roll
	1,			// pitch
	-1,			// yaw
};



// station configuration

// EEPROM configuration
#define EEPROM_MAG_ZERO 0 			// start 0 size 12
#define EEPROM_VOLTAGE_DIVIDER 12 	// start 12 size 4
#define EEPROM_MAG_ZERO2 16 		// start 16 size 12
#define EEPROM_MAG_GAIN 28 		// start 28 size 12
#define EEPROM_CONFIG 40		// start 40 size 128*8

#endif
