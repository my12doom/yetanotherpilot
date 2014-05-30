#ifndef __CONFIG_H__
#define __CONFIG_H__

// printf debug configuration
#define _TRACE 0
#define ITM_DBG
#define USART1_DBG
#define SW_I2C
#define GPS_BUFFER_BLOCK 512
#define PCB_VERSION 3
//#define EXTERNAL_HMC5883
#define EXTERNAL_HMC5883_2
#define HEADFREE

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
#define PI 3.14159265
#define cycle_time (3000)

#define RC_TIMEOUT 1000000				// 1 seconds
#define RC_RANGE 400
#define RC_DEAD_ZONE 5
#define RC_CENTER 1520
#define BARO_OSS 50
#define hall_sensor_sensitivity	0.0666		// unit: mV / mA
#define VOLTAGE_DIVIDER_BASE 6		// uncalibrated voltage divider ratio
#define MAX_GYRO_BIAS_DRIFT 30
#define THROTTLE_STOP 1110
#define THROTTLE_IDLE (THROTTLE_STOP+60)
#define THROTTLE_MAX 1888
#define THROTTLE_CRUISE 1550

#define ACRO_ROLL_RATE (PI*3/2)				// 270 degree/s
#define ACRO_PITCH_RATE (PI)			// 180 degree/s
#define ACRO_YAW_RATE (PI/2)			// 90 degree/s

#define ACCELEROMETER_THRESHOLD 0.3
#define MAG_THRESHOLD 0.3

#define CRUISING_SPEED 125				// 125 pascal
#define LOG_NRF 1
#define LOG_SDCARD 2
#define LOG_USART1 4
#define LOG_USART2 8
extern int LOG_LEVEL;

extern float pid_limit[3][3]; 				// pid_limit[roll,pitch,yaw][p max offset, I limit, d dummy]
extern float pid_factor[3][3];			// pid_factor[roll,pitch,yaw][p,i,d]
extern float pid_factor2[3][4];			// another pid factor
extern float stablize_Kp;
extern float quadcopter_trim[3];
static float quadcopter_range[3] = 
{
	PI/8,			// roll target on RC full deflection
	PI/8,			// pitch
	PI/8,			// yaw
};

static int quadcopter_mixing_matrix[4][3] = // the motor mixing matrix, [motor number] [roll, pitch, yaw]
{
	{0, +1, -1},			// rear
	{-1, 0, +1},			// right
	{0, -1, -1},			// front
	{+1, 0, +1},			// left
};

#if QUADCOPTER == 1
#define ACRO_MANUAL_FACTOR (0.0)

#else
#define ACRO_MANUAL_FACTOR (0.3)		// final output in acrobatic mode, 70% pid, 30% rc

#endif

#define QUADCOPTER_MAX_DELTA 100




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

extern float pid_quad_altitude[4];	// P, I, D, IMAX, 
									// unit: 1/second, 1/seconds^2, 1, meter*second
									// convert altitude error(meter) to target climb rate(meter/second)
extern float pid_quad_alt_rate[4];	// P, I, D, IMAX
									// unit: 1/second, 1/seconds^2, 1, meter*second
									// convert climb rate error(meter/second) to target acceleration(meter/second^2)
extern float pid_quad_accel[4];		// P, I, D, IMAX
									// unit:
									// convert acceleration error(meter/second^2) to motor output

#define quadcopter_max_climb_rate 5
#define quadcopter_max_descend_rate 2
#define quadcopter_max_acceleration 4.5

// station configuration

// EEPROM configuration
#define EEPROM_MAG_ZERO 0 			// start 0 size 12
#define EEPROM_VOLTAGE_DIVIDER 12 	// start 12 size 4
#define EEPROM_MAG_ZERO2 16 		// start 16 size 12
#define EEPROM_MAG_GAIN 28 		// start 16 size 12

#endif
