#include <stdio.h>
#include "mcu.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "RFData.h"
#include "common/adc.h"
#include "common/printf.h"
#include "common/I2C.h"
#include "common/PPM.h"
#include "common/common.h"
#include "common/vector.h"
#include "common/build.h"
#include "sensors/HMC5883.h"
#include "sensors/MPU6050.h"
#include "sensors/mag_offset.h"
#include "common/matrix.h"
#include "common/param.h"
#include "common/space.h"

#ifndef LITE
#include "common/gps.h"
#include "common/ads1115.h"
#include "common/ads1256.h"
#include "sensors/sonar.h"
#include "common/NRF24L01.h"
#include "fat/ff.h"
#include "osd/MAX7456.h"
#include "sensors/MS5611.h"
#include "sensors/hp203b.h"
#else
#include "sensors/BMP085.h"
#endif


extern "C"
{
#include "fat/diskio.h"

//#include "osd/osdcore.h"

#ifdef STM32F1
#ifndef LITE
	#include "usb_mass_storage/hw_config.h"
	#include "usb_mass_storage/usb_init.h"
	#include "usb_mass_storage/mass_mal.h"
#else
	#include "usb_com/hw_config.h"
	#include "usb_mass_storage/usb_init.h"
#endif
#endif

#ifdef STM32F4
	#include "usb_comF4/cdc/usbd_cdc_core.h"
	#include "usb_comF4/core/usbd_usr.h"
	#include "usb_comF4/usb_conf/usbd_desc.h"
	#include "usb_comF4/usb_conf/usb_conf.h"

	#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
	#if defined ( __ICCARM__ ) /*!< IAR Compiler */
	#pragma data_alignment=4
	#endif
	#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

	__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_dev  __ALIGN_END ;
#endif
}

#ifndef LITE
typedef struct
{
	ads1115_speed speed;
	ads1115_channel channel;
	ads1115_gain gain;
	int16_t *out;
}ads1115_work;
#define MAX_ADS1115_WORKS 8
int ads1115_work_pos = 0;
int ads1115_work_count = 0;
ads1115_work ads1115_works[MAX_ADS1115_WORKS];
int ads1115_new_work(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, int16_t *out);
int ads1115_go_on();
#endif

enum
{
	error_gyro = 1,
	error_accelerometer = 2,
	error_magnet = 4,
	error_baro = 8,
	error_RC = 16,
	error_MAX,
} critical_error;

int critical_errors = 0;

static param crash_protect("prot", 0);		// crash protection
#define CRASH_TILT_IMMEDIATE	1
#define CRASH_COLLISION_IMMEDIATE	2


static param pid_factor[3][4] = 			// pid_factor[roll,pitch,yaw][p,i,d,i_limit]
{
	{param("rP1",0.50), param("rI1",0.375), param("rD1",0.05),param("rM1",PI)},
	{param("rP2",0.50), param("rI2",0.375), param("rD2",0.05),param("rM2",PI)},
	{param("rP3",1.75), param("rI3",0.25), param("rD3",0.01),param("rM3",PI)},
};
static param pid_factor2[3][4] = 			// pid_factor2[roll,pitch,yaw][p,i,d,i_limit]
{
	{param("sP1", 6), param("sI1", 0), param("sD1", 0.12),param("sM1", PI/45)},
	{param("sP2", 6), param("sI2", 0), param("sD2", 0.12),param("sM2", PI/45)},
	{param("sP3", 8), param("sI3", 0), param("sD3", 0.23),param("sM3", PI/45)},
};
static param quadcopter_max_climb_rate("maxC",5);
static param quadcopter_max_descend_rate("maxD", 2);
static param quadcopter_max_acceleration("maxA", 4.5);

static param pid_quad_altitude[4] = 	// P, I, D, IMAX, 
										// unit: 1/second, 1/seconds^2, 1, meter*second
										// convert altitude error(meter) to target climb rate(meter/second)
{
	param("altP", 1.0f),
	param("altI", 0.0f),
	param("altD", 0.0f),
	param("altM", 0.0f),
};

static param pid_quad_alt_rate[4] = 	// P, I, D, IMAX
										// unit: 1/second, 1/seconds^2, 1, meter*second
										// convert climb rate error(meter/second) to target acceleration(meter/second^2)
{
	param("cliP", 6.0f),
	param("cliI", 0.0f),
	param("cliD", 0.0f),
	param("cliM", 0.0f),
};
static param pid_quad_accel[4] =		// P, I, D, IMAX
										// unit:
										// convert acceleration error(meter/second^2) to motor output
										// In ardupilot, default P = 0.75 converts 1 cm/s^2 into 0.75 * 0.1% of full throttle
										// In yetanotherpilot implementation, default P=0.075 converts 1 m/s^2 into 0.075 of full throttle
										// the max accel error in default value is around +- 6.66 m/s^2, but should not use that much
{
	param("accP", 0.075f),
	param("accI", 0.150f),
	param("accD", 0.0f),
	param("accM", 2.0f),
};

static param quadcopter_trim[3] = 
{
// 	param("trmR", -2.36 * PI / 18),			// zewu roll
// 	param("trmP", 2.72 * PI / 180),			// zewy pitch
	param("trmR", 0 * PI / 18),				// roll
	param("trmP", 0 * PI / 180),			// pitch
	param("trmY", 0.0f),					// yaw
};
static param quadcopter_range[3] = 
{
	param("rngR", PI / 7),			// roll
	param("rngP", PI / 7),			// pitch
	param("rngY", PI / 8),			// yaw
};

static param gyro_bias[2][4] =	//[p1,p2][temperature,g0,g1,g2]
{
	{param("gbt1", NAN), param("gb11", 0), param("gb21", 0), param("gb31", 0),},
	{param("gbt2", NAN), param("gb12", 0), param("gb22", 0), param("gb32", 0),},
};

static param power_factor("pfac", 1.0f);

static param rc_setting[8][4] = 
{
	{param("rc00", 1000), param("rc01", 1520), param("rc02", 2000), param("rc03", 0),},
	{param("rc10", 1000), param("rc11", 1520), param("rc12", 2000), param("rc13", 0),},
	{param("rc20", 1000), param("rc21", 1520), param("rc22", 2000), param("rc23", 0),},
	{param("rc30", 1000), param("rc31", 1520), param("rc32", 2000), param("rc33", 0),},
	{param("rc40", 1000), param("rc41", 1520), param("rc42", 2000), param("rc43", 0),},
	{param("rc50", 1000), param("rc51", 1520), param("rc52", 2000), param("rc53", 0),},
	{param("rc60", 1000), param("rc61", 1520), param("rc62", 2000), param("rc63", 0),},
	{param("rc70", 1000), param("rc71", 1520), param("rc72", 2000), param("rc73", 0),},
};

static param motor_matrix("mat", 0);
static param THROTTLE_IDLE("idle", 1200);
#define MAX_MOTOR_COUNT 8
static int quadcopter_mixing_matrix[2][MAX_MOTOR_COUNT][3] = // the motor mixing matrix, [motor number] [roll, pitch, yaw]
{
	{							// + mode
		{0, +1, -1},			// rear, CCW
		{-1, 0, +1},			// right, CW
		{0, -1, -1},			// front, CCW
		{+1, 0, +1},			// left, CW
	},
	{							// X mode
		{-1,+1,-1},				//REAR_R, CCW
		{-1,-1,+1},				//FRONT_R, CW
		{+1,-1,-1},				//FRONT_L, CCW
		{+1,+1,+1},				//REAR_L, CW
	}
};

/*
*/
#if PCB_VERSION == 2
#define CURRENT_PIN 2
#define VOLTAGE_PIN 4
#elif PCB_VERSION == 1
#define CURRENT_PIN 0
#define VOLTAGE_PIN 4
#elif PCB_VERSION == 3
#define CURRENT_PIN 9
#define VOLTAGE_PIN 9
#endif

#ifdef LITE
#define read_baro read_BMP085
#else
#define read_baro read_MS5611
#endif

float PI180 = 180/PI;

static void SysClockInit(void);

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}
static int16_t max(int16_t a, int16_t b)
{
	return a<b?b:a;
}
static float f_min(float a, float b)
{
	return a > b ? b : a;
}
static float f_max(float a, float b)
{
	return a > b ? a : b;
}

// a helper
bool calculate_roll_pitch(vector *accel, vector *mag, vector *accel_target, vector *mag_target, float *roll_pitch);



// states
#ifndef LITE
bool sd_ok = false;
bool nrf_ok = false;
FIL *file = NULL;
FRESULT res;
FATFS fs;
#endif
uint64_t last_log_flush_time = 0;
bool launched = false;
float mpu6050_temperature = 0;
float angle_pos[3] = {0};
float angle_posD[3] = {0};
float angle_target_unrotated[3] = {0};	// for quadcopter only currently, for fixed-wing, pos is also angle_pos
float angle_target[3] = {0};	// for quadcopter only currently, for fixed-wing, pos is also angle_pos
float angle_error[3] = {0};
float angle_errorD[3] = {0};
float angle_errorI[3] = {0};
float pos[3] = {0};
float target[3] = {0};		// target [roll, pitch, yaw] (pid controller target, can be angle or angle rate)
int cycle_counter = 0;
float ground_pressure = 0;
float ground_temperature = 0;
float rc[8] = {0};			// ailerron : full left -1, elevator : full down -1, throttle: full down 0, rudder, full left -1
// float climb_rate = 0;
// float climb_rate_lowpass = 0;
// float climb_rate_filter[7] = {0};			// 7 point Derivative Filter(copied from ArduPilot), see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/#noiserobust_2
// float climb_rate_filter_time[7] = {0};
float accelz = 0;
bool airborne = false;
bool nearground = false;
float takeoff_ground_altitude = 0;
int mode = initializing;
int64_t collision_detected = 0;	// remember to clear it before arming
int64_t tilt_us = 0;	// remember to clear it before arming
uint8_t data[32];
static sensor_data *p = (sensor_data*)data;
vector gyro;
vector mag;
vector accel;
vector estAccGyro = {0};			// for roll & pitch
vector estMagGyro = {0};			// for yaw
vector estGyro = {0};				// for gyro only yaw, yaw lock on this
vector groundA;						// ground accerometer vector
vector groundM;						// ground magnet vector
float mag_radius = -999;
vector mag_avg = {0};
vector gyro_zero = {0};
vector accel_avg = {0};
vector mag_zero = {0};
vector mag_gain = {0.7924,0.8354,0.8658};
vector accel_earth_frame;
param voltage_divider_factor("vfac",6);
int ms5611[2];
int ms5611_result = -1;
float adc_2_5_V = -1;
float VCC_3_3V = -1;
float VCC_5V = -1;
float VCC_motor = -1;
float airspeed_voltage = -1;
long last_baro_time = 0;
int baro_counter = 0;
char climb_rate_string[10];
int64_t time;
int last_mode = initializing;
float rc_zero[] = {1520, 1520, 1520, 1520, 1520, 1520};
float error_pid[3][3] = {0};		// error_pid[roll, pitch, yaw][p,i,d]
int64_t last_tick = getus();
int64_t last_gps_tick = 0;
float airspeed_sensor_data;
int adc_voltage = 0;
int adc_current = 0;
vector gyroI = {0};	// attitude by gyro only
vector targetVA;		// target accelerate vector
vector targetVM;		// target magnet vector
float airspeed_bias = 0;
bool has_airspeed = false;
float accel_1g = 0;
float interval = 0;

int64_t last_rc_work = 0;
float roll;
float pitch;
float yaw_est;
float yaw_gyro;
float yaw_launch;
float pid_result[3] = {0}; // total pid for roll, pitch, yaw

float a_raw_pressure = 0;
float a_raw_temperature = 0;
float a_raw_altitude = 0;
float a_altitude = NAN;
float a_raw_climb = 0;
float a_climb = 0;
float a_climb2_tick = 0;
float a_climb_rate_filter[7] = {0};			// 7 point Derivative Filter(copied from ArduPilot), see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/#noiserobust_2
float a_climb_rate_filter_time[7] = {0};


float _time_constant_z = 5.0f;
float _k1_z = 3 / _time_constant_z;
float _k2_z = 3 / (_time_constant_z*_time_constant_z);
float _k3_z = 1 / (_time_constant_z*_time_constant_z*_time_constant_z);
float _position_error = 0;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
float _position_base = 0;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float _position_correction = 0;       // sum of corrections to _position_base from delayed 1st order samples in cm
float _velocity_base = 0;             // latest velocity estimate (integrated from accelerometer values) in cm/s
float _velocity_correction = 0;       // latest velocity estimate (integrated from accelerometer values) in cm/s
float _position = 0;                  // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float _velocity = 0;				  // latest velocity estimate (integrated from accelerometer values) in cm/s
float _accel_correction_ef = 0;		  // accelerometer corrections

float target_altitude = 0;
float ground_altitude = NAN;
float target_climb_rate = 0;
float target_accel = 0;
float altitude_error_pid[3] = {0};
float climb_rate_error_pid[3] = {0};
float accel_error_pid[3] = {0};
float throttle_result = 0;
float throttle_real = 0;
float throttle_real_crusing = THROTTLE_CRUISE;

int16_t ads1115_2_5V = 0;
int16_t ads1115_airspeed = 0;
int16_t ads1115_voltage = 0;
int16_t ads1115_current = 0;
float mah_consumed = 0;
float wh_consumed = 0;

float sonar_distance = NAN;
int64_t last_sonar_time = getus();
bool has_5th_channel = true;


vector gyro_temp_k = {0};		// gyro temperature compensating curve (linear)
vector gyro_temp_a = {0};
float temperature0 = 0;

void matrix_error(const char*msg)
{
	ERROR(msg);
	while(true)
		;
}

void matrix_mov(float *dst, const float *src, int row, int column)
{
	memcpy(dst, src, row*column*4);
}

void matrix_add(float *op1, float *op2, int row, int column)
{
	int count = row * column;
	for(int i=0; i<count; i++)
		op1[i] += op2[i];
}
void matrix_sub(float *op1, float *op2, int row, int column)
{
	int count = row * column;
	for(int i=0; i<count; i++)
		op1[i] -= op2[i];
}

int matrix_mul(float *out, const float *m1, int row1, int column1, const float *m2, int row2, int column2)
{
	if (column1 != row2)
		matrix_error("invalid matrix_mul");

	for(int x1 = 0; x1<column2; x1++)
	{
		for(int y1 = 0; y1<row1; y1++)
		{
			out[y1*column2+x1] = 0;
			for(int k = 0; k<column1; k++)
				out[y1*column2+x1] += m1[y1*column1+k] * m2[k*column2+x1];
		}
	}

	return 0;
}

int inverse_matrix2x2(float *m)
{
	float det = m[0] * m[3] - m[1] * m[2];
	if (det == 0)
		return -1;
	float t[4] = {m[0], m[1], m[2], m[3]};

	m[0] = t[3]/det;
	m[1] = -t[1]/det;
	m[2] = -t[2]/det;
	m[3] = t[0]/det;

	return 0;
}
// kalman test
float state[4] = {0};	// 4x1 matrix, altitude, climb, accel, accel_bias

#if 1
float P[16] = 
{
	200, 0, 0, 0,
	0, 200, 0, 0,
	0, 0, 200, 0,
	0, 0, 0, 200
};	// 4x4 matrix, covariance
float Q[16] = 
{
	4e-6, 0, 0, 0,
	0, 1e-6, 0, 0,
	0, 0, 1e-6, 0,
	0, 0, 0, 1e-7,
};
float R[4] = 
{
	4.8, 0,
	0, 0.0063,
};

float R2[4] = 
{
	48, 0,
	0, 0.00230,
};
#else
float P[16] = 
{
	2000, 0, 0, 0,
	0, 2000, 0, 0,
	0, 0, 2000, 0,
	0, 0, 0, 2000
};	// 4x4 matrix, covariance
float Q[16] = 
{
	0.4, 0, 0, 0,
	0, 0.1, 0, 0,
	0, 0, 0.1, 0,
	0, 0, 0, 0.01,
};

float R[4] = 
{
	480000, 0,
	0, 230,
};


float R2[4] = 
{
	4800000, 0,
	0, 230,
};
#endif

int kalman()
{
	if (interval > 0.2f)
		return -1;

	// near ground, reject ground effected baro data
	bool invalid_baro_data = mode == quadcopter && (!airborne || (!isnan(sonar_distance) && sonar_distance < 1.0f) || fabs(state[0] - ground_altitude) < 1.0f);
	bool bias_baro_data = invalid_baro_data;
	invalid_baro_data = ms5611_result == 0;


	float dt = interval;
	float dtsq = dt*dt;
	float dtsq2 = dtsq/2;
	float F[16] = 
	{
		1, dt, dtsq2, dtsq2,
		0, 1, dt, dt,
		0, 0, 1, 0,
		0, 0, 0, 1,
	};
	float FT[16] = 
	{
		1, 0, 0, 0,
		dt, 1, 0, 0,
		dtsq2, dt, 1, 0,
		dtsq2, dt, 0, 1,
	};
	
	static float Hab[2*4] = 
	{
		1, 0, 0, 0,
		0, 0, 1, 0,
	};
	static float HabT[4*2] = 
	{
		1, 0,
		0, 0,
		0, 1,
		0, 0,
	};

	// accelerometer only
	static float Ha[2*4] = 
	{
		0, 0, 1, 0,
	};
	static float HaT[4*2] = 
	{
		0,
		0,
		1,
		0,
	};

	float zk_ab[2] = {a_raw_altitude, accelz};
	float zk_a[2] = {accelz};

	float *H = invalid_baro_data ? Ha : Hab;
	float *HT = invalid_baro_data ? HaT : HabT;
	float *zk = invalid_baro_data ? zk_a : zk_ab;
	int observation_count = invalid_baro_data ? 1 : 2;

	float state1[4];
	float P1[16];
	float tmp[16];
	float tmp2[16];
	float tmp3[16];
	float kg[8];


	// predict
	matrix_mul(state1, F, 4, 4, state, 4, 1);
	matrix_mul(tmp, P, 4, 4, FT, 4, 4);
	matrix_mul(P1, F, 4, 4, tmp, 4, 4);

	// covariance
	matrix_add(P1, Q, 4, 4);

	// controll vector
	//state1[2] = state1[2] * 0.8f * 0.2f * (target_accel);
// 	if (invalid_baro_data)
// 		state1[1] *= 0.995f;

	// update

	// kg
	matrix_mul(tmp, P1, 4, 4, HT, 4, observation_count);
	matrix_mul(tmp2, H, observation_count, 4, P1, 4, 4);
	matrix_mul(tmp3, tmp2, observation_count, 4, HT, 4, observation_count);
	if (observation_count == 2)
	{
		matrix_add(tmp3, bias_baro_data ? R2 : R, observation_count, observation_count);
		inverse_matrix2x2(tmp3);
	}
	else
	{
		tmp3[0] += R[3];
		tmp3[0] = 1.0f / tmp3[0];
	}
	matrix_mul(kg, tmp, 4, observation_count, tmp3, observation_count, observation_count);


	// update state
	// residual
	matrix_mul(tmp, H, observation_count, 4, state1, 4, 1);
	matrix_sub(zk, tmp, observation_count, 1);

	matrix_mul(tmp, kg, 4, observation_count, zk, observation_count, 1);
	matrix_mov(state, state1, 4, 1);
	matrix_add(state, tmp, 4, 1);

	// update P
	float I[16] = 
	{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	};
	matrix_mul(tmp, kg, 4, observation_count, H, observation_count, 4);
	matrix_sub(I, tmp, 4, 4);
	matrix_mul(P, I, 4, 4, P1, 4, 4);
	
	TRACE("\rtime=%.3f,state:%.2f,%.2f,%.2f,%.2f, ref=%.2f/%.2f/%.2f, throttle:%d", getus()/1000000.0f, state[0], state[1], state[2], state[3], _position, a_raw_altitude, _velocity, throttle_result);

	return 0;
}

int auto_throttle(float user_climb_rate)
{
	// new target altitude
	if (fabs(user_climb_rate) < 0.001f && airborne)
	{
		if (isnan(target_altitude))
			target_altitude = state[0];
	}
	else
	{
		target_altitude = NAN;
	}

	// new altitude error
	if (isnan(target_altitude))
	{
		target_climb_rate = user_climb_rate;
	}
	else
	{
		altitude_error_pid[0] = target_altitude - state[0];
		altitude_error_pid[0] = limit(altitude_error_pid[0], -2.5f, 2.5f);

		// new target rate, directly use linear approach since we use very tight limit 
		// TODO: use sqrt approach on large errors (see get_throttle_althold() in Attitude.pde)
		// TODO: apply pid instead of P only
		target_climb_rate = pid_quad_altitude[0] * altitude_error_pid[0];
		target_climb_rate = limit(target_climb_rate, -quadcopter_max_descend_rate, quadcopter_max_climb_rate);
	}

	TRACE("\rtarget_climb_rate=%.2f, out=%d", target_climb_rate, throttle_result);


	// new climb rate error
	float climb_rate_error = target_climb_rate - state[1];

	// apply a 2Hz LPF to rate error
	const float RC = 1.0f/(2*3.1415926 * 2.0f);
	float alpha = interval / (interval + RC);
	// 5Hz LPF filter
	const float RC5 = 1.0f/(2*3.1415926 * 2.0f);
	float alpha5 = interval / (interval + RC5);
	// 30Hz LPF for derivative factor
	const float RC30 = 1.0f/(2*3.1415926 * 30.0f);
	float alpha30 = interval / (interval + RC30);

	// TODO: apply pid instead of P only, and add feed forward
	// reference: get_throttle_rate()
	bool ground_ops = !airborne && climb_rate_error_pid[0]<0;
	float accel_factor_ground = throttle_real_crusing*1.1/(quadcopter_max_descend_rate)/pid_quad_accel[0];

	climb_rate_error_pid[0] = climb_rate_error_pid[0] * (1-alpha) + alpha * climb_rate_error;
	target_accel = climb_rate_error_pid[0] * (ground_ops ? accel_factor_ground : pid_quad_alt_rate[0]);
	target_accel = limit(target_accel,  airborne ? -quadcopter_max_acceleration : -2 * quadcopter_max_acceleration, quadcopter_max_acceleration);

	
	// new accel error, +2Hz LPF
	float accel_error = target_accel - (accelz + state[3]);
	accel_error = accel_error_pid[0] * (1-alpha) + alpha * accel_error;

	// core pid
	if (airborne)
	{
		accel_error_pid[1] += accel_error_pid[0] * interval;
		accel_error_pid[1] = limit(accel_error_pid[1], -pid_quad_accel[3], pid_quad_accel[3]);
	}
	accel_error_pid[2] = accel_error_pid[2] * (1-alpha30) + alpha30 * (accel_error - accel_error_pid[0])/interval;
	accel_error_pid[0] = accel_error;


	float output = 0;
	output += accel_error_pid[0] * pid_quad_accel[0];
	output += accel_error_pid[1] * pid_quad_accel[1];
	output += accel_error_pid[2] * pid_quad_accel[2];

	throttle_result  = output + throttle_real_crusing;
	float angle_boost_factor = limit(1/ cos(pitch) / cos(roll), 1.0f, 1.5f);
	throttle_result = throttle_result * angle_boost_factor;
	
	throttle_result = limit(throttle_result, airborne ? QUADCOPTER_THROTTLE_RESERVE : 0, 1 - QUADCOPTER_THROTTLE_RESERVE);

	TRACE("\rthrottle=%.3f, altitude = %.2f/%.2f, pid=%.2f,%.2f,%.2f", throttle_result, state[0], target_altitude,
		accel_error_pid[0], accel_error_pid[1], accel_error_pid[2]);

	// update throttle_real_crusing if we're in near level state and no violent climbing/descending action
	if (airborne && throttle_real>0 && fabs(state[1]) < 0.5f && fabs(state[3] + accelz)<0.5f && fabs(roll)<5*PI/180 && fabs(pitch)<5*PI/180
		&& fabs(user_climb_rate) < 0.001f)
	{
		// 0.2Hz low pass filter
		const float RC02 = 1.0f/(2*3.1415926 * 0.2f);
		float alpha02 = interval / (interval + RC02);

		//throttle_real_crusing = throttle_real_crusing * (1-alpha02) + alpha02 * throttle_real;
		// TODO: estimate throttle cursing correctly
	}

	return 0;
}

int altitude_estimation_baro()
{
	// delta time
// 	float time = getus()/1000000.0f;
// 	float delta_time = a_climb2_tick == 0 ? 0 : (time - a_climb2_tick);
// 	a_climb2_tick = time;

	// raw altitude
	double scaling = (double)a_raw_pressure / ground_pressure;
	float temp = ((float)ground_temperature) + 273.15f;
	a_raw_altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));
	if (fabs(a_raw_altitude) < 5.0f)
		ground_temperature = a_raw_temperature;

 	_position_error = a_raw_altitude - (_position_base + _position_correction);


	return 0;
}

int altitude_estimation_inertial()
{
	float &dt = interval;

	_accel_correction_ef += _position_error * _k3_z  * dt;
	_velocity_correction += _position_error * _k2_z  * dt;
	_position_correction += _position_error * _k1_z  * dt;

	const float velocity_increase = (accelz + _accel_correction_ef) * dt;
	_position_base += (_velocity_base + _velocity_correction + velocity_increase*0.5f) * dt;
	_velocity_base += velocity_increase;

	_position = _position_base + _position_correction;
	_velocity = _velocity_base + _velocity_correction;

	kalman();
	
// 	state[0] = _position;
// 	state[1] = _velocity;
// 	state[3] = _accel_correction_ef;
	
	return 0;
}

#ifndef LITE
int sdcard_speed_test()
{
	FIL f;
	res = f_open(&f, "test.bin", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	if (sd_ok)
	{
		UINT done;
		int64_t ttt = getus();
		char blk[512*16] = {0};
// 		for(int i=0; i<4096; i++)
// 			f_write(&f, blk, 512, &done);
		ttt = getus() - ttt;

		ERROR("SDCARD 2Mbyte write cost %dus\r\n", int(ttt));
		f_lseek(&f, 0);
		ttt = getus();
// 		for(int i=0; i<4096; i++)
// 			f_read(&f, blk, 512, &done);
		ttt = getus() - ttt;
		ERROR("SDCARD 2Mbyte read cost %dus\r\n", int(ttt));

		ttt = getus();
//		for(int i=0; i<4096; i++)
//			SD_ReadBlock(((int64_t)i) << 9 ,(uint32_t*)blk, 512);
		ttt = getus() - ttt;
		ERROR("SDCARD 2Mbyte raw read cost %dus\r\n", int(ttt));

		ttt = getus();
//		for(int i=0; i<4096; i+=8)
//			SD_ReadMultiBlocks(((int64_t)i+8192) << 9 ,(uint32_t*)blk, 512,8);
		ttt = getus() - ttt;
		ERROR("SDCARD 2Mbyte raw read cost %dus\r\n", int(ttt));
	}
	f_close(&f);
	return 0;
}

int sdcard_init()
{	
	ERROR("sdcard init...");
	FIL f;
	res = disk_initialize(0) == RES_OK ? FR_OK : FR_DISK_ERR;
	res = f_mount(&fs, "", 0);
	res = f_open(&f, "test.bin", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	sd_ok = res == FR_OK;
	f_close(&f);
	ERROR("%s\r\n", sd_ok ? "OK" : "FAIL");
	return 0;
}
#endif

float errorV[2] = {0};
float rc_d[3] = {0};

int calculate_position()
{
#if QUADCOPTER == 1
	float new_angle_pos[3] = {roll, pitch, gyroI.array[2]};

	// the quadcopter's main pid lock on angle rate
	for(int i=0; i<3; i++)
	{
		pos[i] = angle_posD[i] = (new_angle_pos[i] - angle_pos[i])/interval;
		angle_pos[i] = new_angle_pos[i];
	}
#else

	for(int i=0; i<3; i++)
		pos[i] = gyroI.array[i];

#endif

	return 0;
}
int calculate_target()
{
	// calculate new target
	float rc_dv[3] = {0};
	float rate[3] = {ACRO_ROLL_RATE * interval / RC_RANGE, 
		ACRO_PITCH_RATE * interval / RC_RANGE,
		ACRO_YAW_RATE * interval / RC_RANGE};

	switch (mode)
	{
	case acrobatic:
		{
			for(int i=0; i<3; i++)
			{
				float rc = g_ppm_input[i==2?3:i] - rc_zero[i==2?3:i];
				if (abs(rc) < RC_DEAD_ZONE)
					rc = 0;
				else
					rc *= rate[i];

				rc_d[i] = -rc * rc_reverse[i] * sensor_reverse[i];

				float new_target = radian_add(target[i], rc_d[i]);
				float new_error = abs(radian_sub(pos[i], new_target));
				if (new_error > ACRO_MAX_OFFSET[i] && new_error > abs(error_pid[i][0]))
					rc_d[i] = 0;
				else
					target[i] = new_target;
			}
		}
		break;

	case rc_fail:
		{
#if QUADCOPTER == 1
			// TODO
#else
			g_ppm_output[2] = (getus() - last_rc_work > 10000000) ? 1178 : 1350;		// 1350 should be enough to maintain altitude for my plane, 1178 should harm nobody
			float delta[3] = {(getus() - last_rc_work > 10000000) ? PI/36*sensor_reverse[1] : 0, -PI/18*sensor_reverse[0], 0};						//, level flight for 10seconds, then 10 degree bank, 5 degree pitch down

			targetVA = groundA;
			targetVM = groundM;

			vector_rotate(&targetVA, delta);
			vector_rotate(&targetVM, delta);
#endif
		}
		break;

	case fly_by_wire:
		{
			float delta[3] = {0, 0, 0};
			for(int i=0; i<2; i++)
			{
				delta[i] = -(g_ppm_input[i==2?3:i] - rc_zero[i==2?3:i])  * rc_reverse[i] * sensor_reverse[i];
				if (abs(delta[i]) < RC_DEAD_ZONE)
					delta[i] = 0;
				else
					delta[i] *= FLY_BY_WIRE_MAX_OFFSET[i] / RC_RANGE;
			}

			targetVA = groundA;
			targetVM = groundM;

			vector_rotate(&targetVA, delta);
			vector_rotate(&targetVM, delta);
		}
		break;

	case acrobaticV:
		{
			float current_error[2];
			vector acc = estAccGyro;
			vector mag = estMagGyro;
			vector_normalize(&acc);
			vector_normalize(&mag);
			calculate_roll_pitch(&acc, &mag, &targetVA, &targetVM, current_error);

			rc_dv[0] = rc_dv[1] = rc_dv[2] = 0;
			for(int i=0; i<2; i++)
			{
				float rc = g_ppm_input[i==2?3:i] - rc_zero[i==2?3:i];
				if (abs(rc) < RC_DEAD_ZONE)
					rc = 0;
				else
					rc *= rate[i];

				rc_dv[i] = -rc * rc_reverse[i] * sensor_reverse[i];

				vector new_targetVA = targetVA;
				vector new_targetVM = targetVM;
				vector_rotate(&new_targetVA, rc_dv);
				vector_rotate(&new_targetVM, rc_dv);

				float new_error[3];
				calculate_roll_pitch(&acc, &mag, &new_targetVA, &new_targetVM, new_error);

				if (abs(new_error[i]) > ACRO_MAX_OFFSET[i] && abs(new_error[i]) > abs(current_error[i]))
					rc_dv[i] = 0;
			}


			vector_rotate(&targetVA, rc_dv);
			vector_rotate(&targetVM, rc_dv);
		}

		break;

#if QUADCOPTER == 1
	case quadcopter:
		{
			// auto throttle
			float v = rc[2] - 0.5;
			float user_rate;
			if (fabs(v)<0.05f)
				user_rate = 0;
			else if (v>= 0.05f)
				user_rate = (v-0.05f)/0.45f * quadcopter_max_climb_rate;
			else
				user_rate = (v+0.05f)/0.45f * quadcopter_max_descend_rate;
			auto_throttle(user_rate);


			// first, calculate target angle
			// roll & pitch, RC trim is accepted.
			for(int i=0; i<2; i++)
			{
				float limit_l = angle_target_unrotated[i] - PI * interval;
				float limit_r = angle_target_unrotated[i] + PI * interval;
				angle_target_unrotated[i] = rc[i] * quadcopter_range[i] * (i==1?-1:1);	// pitch stick and coordinate are reversed 
				angle_target_unrotated[i] = limit(angle_target_unrotated[i], limit_l, limit_r);
				angle_target[i] = angle_target_unrotated[i];
			}

#ifdef HEADFREE
			float diff = yaw_launch - yaw_est;
			float cosdiff = cos(diff);
			float sindiff = sin(diff);
			angle_target[0] = angle_target_unrotated[0] * cosdiff - angle_target_unrotated[1] * sindiff;
			angle_target[1] = angle_target_unrotated[0] * sindiff + angle_target_unrotated[1] * cosdiff;
#endif

			// yaw:
			//target[2] = limit((g_ppm_input[3] - RC_CENTER) * rc_reverse[2] / RC_RANGE, -1, 1) * quadcopter_range[2] + yaw_gyro + quadcopter_trim[2];
			if (airborne || rc[2] > 0.2f)	// airborne or armed and throttle up
			{
				rc_d[2] = ((fabs(rc[3]) < (float)RC_DEAD_ZONE/RC_RANGE) ? 0 : -rc[3]) * interval * QUADCOPTER_ACRO_YAW_RATE;

				float trimmed_pos = radian_add(angle_pos[2], quadcopter_trim[2]);
				float new_target = radian_add(angle_target[2], rc_d[2]);
				float new_error = abs(radian_sub(trimmed_pos, new_target));
				if (new_error > (airborne?QUADCOPTER_MAX_YAW_OFFSET:(QUADCOPTER_MAX_YAW_OFFSET/5)) && new_error > abs(angle_error[2]))
					rc_d[2] = 0;
				else
					angle_target[2] = new_target;
			}
			else
			{
				angle_target[2] = angle_pos[2];
			}


			// now calculate target angle rate
			// based on a PID stablizer
			for(int i=0; i<3; i++)
			{
				float new_angle_error = radian_sub(angle_pos[i], angle_target[i]);	// use radian_sub mainly for yaw


				// 5hz low pass filter for D, you won't be that crazy, right?
				static const float lpf_RC = 1.0f/(2*PI * 20.0f);
				float alpha = interval / (interval + lpf_RC);

				if (airborne)
				{
					angle_errorI[i] += new_angle_error * interval;
					angle_errorI[i] = limit(angle_errorI[i], -pid_factor2[i][3], pid_factor2[i][3]);
				}
				angle_errorD[i] = (1-alpha) * angle_errorD[i] + alpha * (new_angle_error - angle_error[i]) / interval;
				angle_error[i] = new_angle_error;

				// apply angle pid
				target[i] = - (angle_error[i] * pid_factor2[i][0] + angle_errorI[i] * pid_factor2[i][1] + angle_errorD[i] * pid_factor2[i][2]);

				// max target rate: 180 degree/second
				target[i] = limit(target[i], -PI, PI);
			}
			TRACE(",roll=%f,%f", angle_pos[0] * PI180, angle_target[0] * PI180, airborne ? "true" : "false");
			TRACE("angle pos,target=%f,%f, air=%s\r\n", angle_pos[1] * PI180, angle_target[1] * PI180, airborne ? "true" : "false");

			// check takeoff
			float active_throttle = (g_ppm_input[5] > RC_CENTER) ? throttle_result : rc[2];
			if ( (state[0] > takeoff_ground_altitude + 1.0f) ||
				(state[0] > takeoff_ground_altitude && active_throttle > throttle_real_crusing) ||
				(active_throttle > throttle_real_crusing + QUADCOPTER_THROTTLE_RESERVE))
			{
				airborne = true;
			}
		}
		break;
#endif
	}

	return 0;
}

int pid()
{

	// calculate new pid & apply pid controll & output
	if (mode == acrobaticV || mode ==rc_fail || mode == fly_by_wire)
	{
		vector acc = estAccGyro;
		vector mag = estMagGyro;
		vector VA = targetVA;
		vector VM = targetVM;
		vector_normalize(&acc);
		vector_normalize(&mag);
		vector_normalize(&VA);
		vector_normalize(&VM);
		calculate_roll_pitch(&acc, &mag, &VA, &VM, errorV);
	}
	float airspeed_factor = has_airspeed ? sqrt(airspeed_sensor_data>0?CRUISING_SPEED/1000.0f/airspeed_sensor_data:2.0f) : 1.0f;
	airspeed_factor = limit(airspeed_factor, 0.5f, 2.0f);
#if QUADCOPTER == 1
	airspeed_factor = 1.0f;
#endif
	for(int i=0; i<3; i++)
	{
		float new_p;

		if (mode == acrobaticV || mode ==rc_fail || mode == fly_by_wire)
			new_p = (i<2 ? errorV[i] : 0) * sensor_reverse[i];
		else if (mode == quadcopter)
			new_p = (pos[i]-target[i]) * sensor_reverse[i];
		else
			new_p = radian_sub(pos[i], target[i]) * sensor_reverse[i];

		if (i ==0 && QUADCOPTER == 1)
			new_p = -new_p;

		TRACE("p[%d]=%f", i, new_p*PI180);


		// I
#if QUADCOPTER == 1
		if (airborne)		// only integrate after takeoff
#endif
		error_pid[i][1] += new_p * interval;
		error_pid[i][1] = limit(error_pid[i][1], -pid_factor[i][3], pid_factor[i][3]);

		// D, with 30hz low pass filter
		static const float lpf_RC = 1.0f/(2*PI * 30.0f);
		float alpha = interval / (interval + lpf_RC);
		error_pid[i][2] = error_pid[i][2] * (1-alpha) + alpha * (new_p - error_pid[i][0] + rc_d[i]* sensor_reverse[i])/interval;

		// P
		error_pid[i][0] = new_p;

		if (mode == fly_by_wire)		// D disabled for fly by wire for now
			error_pid[i][2] = 0;

		// sum
		pid_result[i] = 0;
		float p_rc = limit(rc[5]+1, 0, 2);
		for(int j=0; j<3; j++)
		{
#if QUADCOPTER == 1
			pid_result[i] += error_pid[i][j] * pid_factor[i][j] * power_factor;
#else
			pid_result[i] += limit(limit(error_pid[i][j],-pid_limit[i][j],+pid_limit[i][j]) / pid_limit[i][j], -1, 1) * pid_factor[i][j] * p_rc * airspeed_factor;

#endif
		}
	}
	TRACE(", pid=%.2f, %.2f, %.2f\n", pid_result[0], pid_result[1], pid_result[2]);

	return 0;
}

int output()
{

	for(int i=0; i<3; i++)
	{
		float sum = pid_result[i] * (1-ACRO_MANUAL_FACTOR);

		int rc = rc_reverse[i]*(g_ppm_input[i==2?3:i] - rc_zero[i==2?3:i]);

		sum += rc * ACRO_MANUAL_FACTOR / RC_RANGE;

		g_ppm_output[i==2?3:i] = limit(rc_zero[i==2?3:i] + sum*RC_RANGE, 1000, 2000);

	}


	// RC pass through for channel 5 & 6
	for(int i=4; i<6; i++)
		g_ppm_output[i] = floor(g_ppm_input[i]+0.5f);

	if (mode != rc_fail)
	{
		// throttle pass through
		g_ppm_output[2] = floor(g_ppm_input[2]+0.5f);
		last_rc_work = getus();

#if QUADCOPTER == 0
		if (g_ppm_input[2] > (THROTTLE_STOP + THROTTLE_MAX)/2)
			launched = true;
#endif
	}


#if QUADCOPTER == 1
	if (mode == quadcopter || (mode == rc_fail) )
	{
		//pid[2] = -pid[2];
		throttle_real = 0;
		int matrix = (float)motor_matrix;

		int motor_count = MAX_MOTOR_COUNT;

		for(int i=0; i<MAX_MOTOR_COUNT; i++)
		{
			if (quadcopter_mixing_matrix[matrix][i][0] == quadcopter_mixing_matrix[matrix][i][1] && 
				quadcopter_mixing_matrix[matrix][i][1] == quadcopter_mixing_matrix[matrix][i][2] && 
				fabs((float)quadcopter_mixing_matrix[matrix][i][2]) < 0.01f)
			{
				motor_count = i;
				break;
			}

			if (mode == rc_fail)
			{
				g_ppm_output[i] = THROTTLE_STOP;
			}
			else
			{
				float mix = g_ppm_input[5] > RC_CENTER ? throttle_result : (rc[2] * 0.7f);

				for(int j=0; j<3; j++)
					mix += quadcopter_mixing_matrix[matrix][i][j] * pid_result[j] * QUADCOPTER_THROTTLE_RESERVE;

				g_ppm_output[i] = limit(THROTTLE_IDLE + mix*(THROTTLE_MAX-THROTTLE_IDLE), THROTTLE_IDLE, THROTTLE_MAX);

				TRACE("\rpid[x] = %f, %f, %f", pid[0], pid[1], pid[2]);
				throttle_real += mix;
			}
		}
		throttle_real /= motor_count;
	}
	else
#endif

		// yaw pass through for acrobatic
		g_ppm_output[3] = g_ppm_input[3];


	// manual flight pass through
	if (mode == manual)
	{
		for(int i=0; i<6; i++)
		{
			g_ppm_output[i] = floor(g_ppm_input[i]+0.5f);

			if (i <2)
			{
				bool neg = g_ppm_output[i] < RC_CENTER;
				if (neg)
					g_ppm_output[i] = -(g_ppm_input[i] - RC_CENTER)*(g_ppm_input[i] - RC_CENTER)/RC_RANGE + RC_CENTER;
				else
					g_ppm_output[i] = (g_ppm_input[i] - RC_CENTER)*(g_ppm_input[i] - RC_CENTER)/RC_RANGE + RC_CENTER;
			}
		}
	}

	if (mode == shutdown || mode == initializing)
	{
		for(int i=0; i<6; i++)
#if QUADCOPTER == 1
			g_ppm_output[i] = THROTTLE_STOP;
#else
			g_ppm_output[i] = i==2 ? THROTTLE_STOP : RC_CENTER;
#endif
	}


	PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);
	return 0;
}

int log(void *data, int size)
{
	int64_t us = getus();

#ifndef LITE
	// NRF
	if (nrf_ok && size <= 32 && LOG_LEVEL & LOG_NRF)
		NRF_Tx_Dat((uint8_t*)data);
#endif
	
	// USART, "\r" are escaped into "\r\r"
	if (LOG_LEVEL & LOG_USART1)
	{
		const char *string = (const char*)data;
		int i,j;
		for(i=0,j=0; i<size; i++,j++)
		{
			USART_SendData(USART1, (unsigned char) string[i]);
			while (!(USART1->SR & USART_FLAG_TXE));
			if (string[i] == '\r')
			{
				USART_SendData(USART1, (unsigned char) string[i]);
				while (!(USART1->SR & USART_FLAG_TXE));
				j++;
			}
		}
		USART_SendData(USART1, (unsigned char) '\r');
		while (!(USART1->SR & USART_FLAG_TXE));
		USART_SendData(USART1, (unsigned char) '\n');
		while (!(USART1->SR & USART_FLAG_TXE));
	}

	// fatfs
	#ifndef LITE
	if (LOG_LEVEL & LOG_SDCARD)
	{
		if (file == NULL && sd_ok)
		{
			static FIL f;
			file = &f;
			char filename[20];
			int done  = 0;
			while(sd_ok)
			{
				sprintf(filename, "%04d.dat", done ++);
				FRESULT res = f_open(file, filename, FA_CREATE_NEW | FA_WRITE | FA_READ);
				if (res == FR_OK)
				{
					f_close(file);
					res = f_open(file, filename, FA_OPEN_EXISTING | FA_WRITE | FA_READ);
					break;
				}
			}
		}

		if (sd_ok && file)
		{
			unsigned int done;
			if (f_write(file, data, size, &done) != FR_OK)
			{
				ERROR("\r\nSDCARD ERROR\r\n");
				sd_ok = false;
			}
			if (getus() - last_log_flush_time > 1000000)
			{
				last_log_flush_time = getus();
				f_sync(file);
			}
		}
	}
	if (getus() - us > 7000)
	{
		TRACE("log cost %d us  ", int(getus()-us));
		TRACE("  fat R/R:%d/%d\r\n", read_count, write_count);
	}
	// 	ERROR("\rfat R/R:%d/%d", read_count, write_count);
	// 	if (read_count + write_count > 1)
	// 		ERROR("\r\n");
	read_count = write_count = 0;
	#endif




	return 0;
}

uint8_t log_buffer[2][512];
volatile int log_pending = 0;
int log_packet_count = 0;

// called by main loop, only copy logs to a memory buffer, should be very fast
int save_logs()
{
	if (LOG_LEVEL == LOG_SDCARD 
		#ifndef LITE
		&& !sd_ok
		#endif
	)
		return 0;

	// if the saving task is transferring logs into 2nd buffer
	if (log_pending !=0)
		return -1;

	int c = 0;

	// send/store debug data
	time = getus();
	rf_data to_send;
	to_send.time = (time & (~TAG_MASK)) | TAG_SENSOR_DATA;
	to_send.data.sensor = *p;


	memcpy(log_buffer[0]+c*32, &to_send, 32);
	c++;

	imu_data imu = 
	{
		ms5611[1],
		ms5611[0],
		{estAccGyro.array[0], estAccGyro.array[1], estAccGyro.array[2]},
		{estGyro.array[0], estGyro.array[1], estGyro.array[2]},
		{estMagGyro.array[0], estMagGyro.array[1], estMagGyro.array[2]},
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_IMU_DATA;
	to_send.data.imu = imu;

	memcpy(log_buffer[0]+c*32, &to_send, 32);
	c++;

	pilot_data pilot = 
	{
		state[0] * 100,
		airspeed_sensor_data * 1000,
		{error_pid[0][0]*180*100/PI, error_pid[1][0]*180*100/PI, error_pid[2][0]*180*100/PI},
		{target[0]*180*100/PI, target[1]*180*100/PI, target[2]*180*100/PI},
		mode,
		mah_consumed,
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_PILOT_DATA;
	to_send.data.pilot = pilot;
	memcpy(log_buffer[0]+c*32, &to_send, 32);
	c++;

	pilot_data2 pilot2 = 
	{
		{error_pid[0][1]*180*100/PI, error_pid[1][1]*180*100/PI, error_pid[2][1]*180*100/PI},
		{error_pid[0][2]*180*100/PI, error_pid[1][2]*180*100/PI, error_pid[2][2]*180*100/PI},
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_PILOT_DATA2;
	to_send.data.pilot2 = pilot2;
	memcpy(log_buffer[0]+c*32, &to_send, 32);
	c++;

	ppm_data ppm = 
	{
		{g_ppm_input[0], g_ppm_input[1], g_ppm_input[2], g_ppm_input[3], g_ppm_input[4], g_ppm_input[5]},
		{g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_output[4], g_ppm_output[5]},
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_PPM_DATA;
	to_send.data.ppm = ppm;
	memcpy(log_buffer[0]+c*32, &to_send, 32);
	c++;

#if QUADCOPTER == 1
	quadcopter_data quad = 
	{
		angle_pos[0] * 18000/PI, angle_pos[1] * 18000/PI, angle_pos[2] * 18000/PI,
		angle_target[0] * 18000/PI, angle_target[1] * 18000/PI, angle_target[2] * 18000/PI,
		pos[0] * 18000/PI, pos[1] * 18000/PI, pos[2] * 18000/PI,
		target[0] * 18000/PI,  target[1] * 18000/PI, target[2] * 18000/PI, 
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_QUADCOPTER_DATA;
	to_send.data.quadcopter = quad;
	memcpy(log_buffer[0]+c*32, &to_send, 32);
	c++;

	quadcopter_data2 quad2 = 
	{
		state[1] * 100,
		airborne,
		state[0] * 100,
		state[2] * 100,
		a_raw_altitude * 100,
		accelz * 100,
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_QUADCOPTER_DATA2;
	to_send.data.quadcopter2 = quad2;
	memcpy(log_buffer[0]+c*32, &to_send, 32);
	c++;

	quadcopter_data3 quad3 = 
	{
		target_altitude * 100,
		_position * 100,
		target_climb_rate * 100,
		_velocity * 100,
		target_accel * 100,
		(accelz + _accel_correction_ef) * 100,
		throttle_result,
		yaw_launch * 18000 / PI,
		yaw_est * 18000 / PI,
		throttle_real_crusing,
		#ifndef LITE
		sonar_result(),
		#else
		0xffff,
		#endif
		accel_error_pid[1]*1000,
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_QUADCOPTER_DATA3;
	to_send.data.quadcopter3 = quad3;
	memcpy(log_buffer[0]+c*32, &to_send, 32);
	c++;
#endif

	// only 5 seconds magnet centering data
	if (getus() < 5000000)
	{
		controll_data &controll = to_send.data.controll;
		to_send.time = (time & (~TAG_MASK)) | TAG_CTRL_DATA;
		controll.cmd = CTRL_CMD_FEEDBACK;
		controll.reg = CTRL_REG_MAGNET;
		controll.value = mag_radius * 1000;
		controll.data[0] = mag_zero.array[0] * 1000;
		controll.data[1] = mag_zero.array[1] * 1000;
		controll.data[2] = mag_zero.array[2] * 1000;

		memcpy(log_buffer[0]+c*32, &to_send, 32);
		c++;
	}


#ifndef LITE
	if (last_gps_tick > getus() - 2000000)
	{
		nmeaINFO &info = *GPS_GetInfo();

		gps_data gps = 
		{
			{info.PDOP*100, info.HDOP*100, info.VDOP*100},
			info.speed/3.6*100,
			info.lon, info.lat, info.elv,
			info.satinfo.inview, info.satinfo.inuse,
			info.sig, info.fix,
		};

		to_send.time = (time & (~TAG_MASK)) | TAG_GPS_DATA;
		to_send.data.gps = gps;
		memcpy(log_buffer[0]+c*32, &to_send, 32);
		c++;
	}
#endif
	
	log_packet_count = c;

	return 0;
}

volatile vector imu_statics[2][4] = {0};		//	[accel, gyro][min, current, max, avg]
int avg_count = 0;

int read_sensors()
{
	#ifndef LITE
	// read external adc
	ads1115_go_on();

	if (sonar_update() == 0)
	{
		sonar_distance = sonar_result() > 0 ? sonar_result()/1000.0f : NAN;
		last_sonar_time = getus();
		TRACE("\rdis=%.2f", sonar_distance);
	}
	#endif

	if (getus()-last_sonar_time > 200000)		//200ms
		sonar_distance = NAN;

	// always read sensors and calculate attitude
	bool imu_error = false;
	if (read_MPU6050(&p->accel[0]) < 0 && read_MPU6050(&p->accel[0]) < 0)
	{
		ERROR("MPU6050 Error\n");
		imu_error = true;
	}
	if (read_HMC5883(&p->mag[0]) < 0 && read_HMC5883(&p->mag[0]) < 0)
	{
		TRACE("HMC5883 Error\n");
	}

	// update imu statics
// #ifdef STM32F1
// 	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
// #endif
// #ifdef STM32F4
// 	NVIC_DisableIRQ(OTG_HS_IRQn);
// 	NVIC_DisableIRQ(OTG_FS_IRQn);
// 	NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn);
// 	NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn);
// #endif
// 	__DSB();
// 	__ISB();
// 	delayus(30);
	if (!imu_error)
	{
		for(int i=0; i<3; i++)
		{
			imu_statics[0][0].array[i] = f_min(p->accel[i], imu_statics[0][0].array[i]);
			imu_statics[0][1].array[i] = p->accel[i];
			imu_statics[0][2].array[i] = f_max(p->accel[i], imu_statics[0][2].array[i]);
			imu_statics[0][3].array[i] = p->accel[i] + imu_statics[0][3].array[i];

			imu_statics[1][0].array[i] = f_min(p->gyro[i], imu_statics[1][0].array[i]);
			imu_statics[1][1].array[i] = p->gyro[i];
			imu_statics[1][2].array[i] = f_max(p->gyro[i], imu_statics[1][2].array[i]);
			imu_statics[1][3].array[i] = p->gyro[i] + imu_statics[1][3].array[i];
		}
		avg_count ++;
	}
// #ifdef STM32F1
// 	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
// #endif
// #ifdef STM32F4
// 	NVIC_EnableIRQ(OTG_HS_IRQn);
// 	NVIC_EnableIRQ(OTG_FS_IRQn);
// 	NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
// 	NVIC_EnableIRQ(OTG_HS_EP1_OUT_IRQn);
// #endif

	for(int i=0; i<3; i++)
		p->mag[i] *= mag_gain.array[i];

	ms5611_result = read_baro(ms5611);

	mpu6050_temperature = p->temperature1  / 340.0f + 36.53f;

	// messure voltage

	// airspeed voltage low pass
#if PCB_VERSION == 3
	airspeed_voltage = ads1115_airspeed * 2.048 / 32767;
	airspeed_sensor_data = - (airspeed_voltage - airspeed_bias);

	//printf("current:%f mA\r\n", v * 0.256f / 32767 *1000000.0f);
#else
	ADC1_SelectChannel(0);
	adc_2_5_V = ADC1_Read()*0.003+adc_2_5_V*0.997;		// always low pass for 2.5V reference
	VCC_3_3V = 2.5f*4095/adc_2_5_V;
	ADC1_SelectChannel(1);
	VCC_5V = 0.997 * VCC_5V + 0.003 * ADC1_Read() * 5.0f/adc_2_5_V;		// always low pass for 5V voltage

	adc_voltage = 0;
	ADC1_SelectChannel(VOLTAGE_PIN);
	adc_voltage += ADC1_Read();
	ADC1_SelectChannel(CURRENT_PIN);
	adc_current += ADC1_Read();
	adc_voltage *= 1000 * VCC_3_3V / 4095 * voltage_divider_factor;		// now unit is mV

	adc_current *= 1000 * VCC_3_3V / 4095;		// now unit is mV
	adc_current = VCC_5V/2*1000 - adc_current;	// now delta mV
	adc_current /= hall_sensor_sensitivity;

	ADC1_SelectChannel(8);
	airspeed_voltage = ADC1_Read() * 2.5f/adc_2_5_V * 0.05 + airspeed_voltage*0.95;
	float airspeed_sensor_data = - (airspeed_voltage - VCC_5V/2 - airspeed_bias * VCC_5V / 5);
#endif
	TRACE("\rairspeed:%f", airspeed_sensor_data);

	// airspeed in m/s
	// airspeed = sqrt( 5 * k * R * T * ( (pd/ph+1)^(1/3.5) -1 ) )
	// k : 1.403
	// R : 287.05287
	// T : us ms5611 sensor, convert to kalvin
	// pd : use airspeed sensor, convert to kPa
	// ph : use ms5611 sensor, convert to kPa


	if (p->voltage <-30000)
		p->voltage = adc_voltage;
	else
		p->voltage = p->voltage * 0.95 + 0.05 * adc_voltage;			// simple low pass

	if (p->current <-30000)
		p->current = adc_current;
	else
		p->current = p->current * 0.95 + 0.05 * adc_current;			// simple low pass

	p->voltage = abs(ads1115_voltage * 4096 * 6 / 32767);
	p->current = abs(ads1115_current * 4096 / 32767 / hall_sensor_sensitivity);

	mah_consumed += interval / 3600 * p->current;
	wh_consumed += interval / 3600 * p->current * p->voltage / 1000000.0f;

	// calculate altitude
	if (ms5611_result == 0)
	{
		TRACE("\r\npressure,temperature=%f, %f, ground pressure & temperature=%f, %f, height=%f, climb_rate=%f, time=%f\r\n", pressure, temperature, ground_pressure, ground_temperature, altitude, climb_rate, (float)getus()/1000000);

		a_raw_pressure = ms5611[0] / 100.0f;
		a_raw_temperature = ms5611[1] / 100.0f;
		altitude_estimation_baro();
	}

	altitude_estimation_inertial();

	return 0;
}

int calculate_attitude()
{

	float GYRO_SCALE = 2000.0f * PI / 180 / 32767 * interval;		// full scale: +/-2000 deg/s  +/-31767, 8ms interval

	// universal
	float dt = mpu6050_temperature - temperature0;
	vector gyro_zero_raw = 
	{
		dt * gyro_temp_k.array[0] + gyro_temp_a.array[0],
		dt * gyro_temp_k.array[1] + gyro_temp_a.array[1],
		dt * gyro_temp_k.array[2] + gyro_temp_a.array[2],
	};

#ifndef LITE
	vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
	vector gyro_zero2 = {-gyro_zero_raw.array[0], -gyro_zero_raw.array[1], -gyro_zero_raw.array[2]};
	vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
#else
	vector gyro = {p->gyro[0], -p->gyro[1], p->gyro[2]};
	vector gyro_zero2 = {gyro_zero_raw.array[0], -gyro_zero_raw.array[1], gyro_zero_raw.array[2]};
	vector acc = {-p->accel[1], -p->accel[0], -p->accel[2]};
#endif
	vector mag = {(p->mag[2]-mag_zero.array[2]), -(p->mag[0]-mag_zero.array[0]), -(p->mag[1]-mag_zero.array[1])};
	vector_sub(&gyro, &gyro_zero2);
// 	for(int i=0; i<3; i++)
// 		if (abs(gyro.array[i])<4)
// 			gyro.array[i] = 0;
	vector_multiply(&gyro, GYRO_SCALE);

	vector_rotate(&estGyro, gyro.array);
	vector_rotate(&estAccGyro, gyro.array);
	vector_rotate(&estMagGyro, gyro.array);

	::gyro = gyro;
	::accel = acc;
	::mag = mag;

	for(int i=0; i<3; i++)
		gyroI.array[i] = radian_add(gyroI.array[i], gyro.array[i]);

	TRACE("gyroI:%f,%f,%f\r", gyroI.array[0] *180/PI, gyroI.array[1]*180/PI, gyroI.array[2]*180/PI);

	// apply CF filter for Mag : 0.5hz low pass for mag
	const float RC = 1.0f/(2*3.1415926 * 0.5f);
	float alpha = interval / (interval + RC);

	vector mag_f = mag;
	vector_multiply(&mag_f, alpha);
	vector_multiply(&estMagGyro, 1-alpha);
	vector_add(&estMagGyro, &mag_f);

	// apply CF filter for Acc if g force is acceptable
	float acc_g = vector_length(&acc)/ accel_1g;
	if (acc_g > 0.90f && acc_g < 1.10f)
	{
		// 0.05 low pass filter for acc reading
		const float RC = 1.0f/(2*3.1415926 * 0.05f);
		float alpha = interval / (interval + RC);


		vector acc_f = acc;
		vector_multiply(&acc_f, alpha);
		vector_multiply(&estAccGyro, 1-alpha);
		vector_add(&estAccGyro, &acc_f);
	}
	else
	{
		TRACE("rapid movement (%fg, angle=%f)", acc_g, acos(vector_angle(&estAccGyro, &acc)) * 180 / PI );
	}

	float dot = acc.V.x * estAccGyro.V.x + acc.V.y * estAccGyro.V.y + acc.V.z * estAccGyro.V.z;
	dot /= vector_length(&estAccGyro);
	dot /= accel_1g;
	accelz = 9.80f * (dot-1);
// 	climb_rate = climb_rate * 0.05 + 0.95 * (climb_rate + accelz * interval);
// 
// 	// test climb rate low pass filter
// 	{
// 		// 0.5hz low pass filter
// 		static const float RC = 1.0f/(2*3.1415926 * 0.5f);
// 		float alpha = interval / (interval + RC);
// 		climb_rate_lowpass = alpha * climb_rate + (1-alpha) * climb_rate_lowpass;
// 	}

	// calculate attitude, unit is radian, range +/-PI
	roll = radian_add(atan2(estAccGyro.V.x, estAccGyro.V.z), PI);
	pitch = atan2(estAccGyro.V.y, (estAccGyro.V.z > 0 ? 1 : -1) * sqrt(estAccGyro.V.x*estAccGyro.V.x + estAccGyro.V.z * estAccGyro.V.z));
	pitch = radian_add(pitch, PI);
	vector estAccGyro16 = estAccGyro;
	vector_divide(&estAccGyro16, 4);
	float xxzz = (estAccGyro16.V.x*estAccGyro16.V.x + estAccGyro16.V.z * estAccGyro16.V.z);
	float G = sqrt(xxzz+estAccGyro16.V.y*estAccGyro16.V.y);
	yaw_est = atan2(estMagGyro.V.z * estAccGyro16.V.x - estMagGyro.V.x * estAccGyro16.V.z,
		(estMagGyro.V.y * xxzz - (estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);
	yaw_gyro = atan2(estGyro.V.z * estAccGyro16.V.x - estGyro.V.x * estAccGyro16.V.z,
		(estGyro.V.y * xxzz - (estGyro.V.x * estAccGyro16.V.x + estGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);

	roll = radian_add(roll, quadcopter_trim[0]);
	pitch = radian_add(pitch, quadcopter_trim[1]);
	yaw_est = radian_add(yaw_est, quadcopter_trim[2]);
	yaw_gyro = radian_add(yaw_gyro, quadcopter_trim[2]);

#ifndef LITE
	accel_earth_frame = acc;
	vector mag_ef = estMagGyro;
	float attitude[3] = {roll, pitch, 0};
	vector_rotate2(&mag_ef, attitude);

	attitude[2] = yaw_est;
	vector_rotate2(&accel_earth_frame, attitude);

	float yaw_mag = atan2(mag_ef.V.x, mag_ef.V.y);

	TRACE("\raccel_ef:%.1f, %.1f, %.1f, accelz:%.2f/%.2f, yaw=%.2f/%.2f", accel_earth_frame.V.x, accel_earth_frame.V.y, accel_earth_frame.V.z, accelz, (accel_earth_frame.V.z+2085)/2085*9.80, yaw_est*180/PI, yaw_mag*180/PI);
#endif
	return 0;
}

void inline debugpin_init()
{
	// use PA-0 as cycle debugger
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_2;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#endif
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#endif
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
}

void inline led_all_on()
{
	#ifdef LITE
	#endif
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
}

void inline led_all_off()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
	GPIO_SetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
}

void inline flashlight_on()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
}
void inline flashlight_off()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
}


int magnet_calibration()
{
	short mag_min[3] = {9999,9999,9999};
	short mag_max[3] = {-9999, -9999, -9999};


	// load magnetemeter cneter
mag_load:
	space_read("magzero", 7, &mag_zero, sizeof(mag_zero),NULL);
	space_read("maggain", 7, &mag_gain, sizeof(mag_gain),NULL);


	for(int i=0; i<3; i++)
	{
		ERROR("mag[%d] max/min/gain/zero=%d,%d,%.2f,%.2f\n", i, mag_max[i], mag_min[i], mag_gain.array[i], mag_zero.array[i]);

		mag_gain.array[i] = mag_gain.array[i] == 0 ? 1 : mag_gain.array[i];
	}

	int enter_throttle = (THROTTLE_IDLE+THROTTLE_MAX)/2;
	// enter magnetemeter centering mode if throttle > THROTTLE_STOP (and slowly flash all LED lights)
	while (g_ppm_input[2] > (enter_throttle))
	{
		mag_offset mag_offset;
		int mag_c = 0;
		int64_t start_tick = getus();

		delayms(100);
		if (g_ppm_input[2] < enter_throttle)
			break;

		while(g_ppm_input[2] > enter_throttle)
		{
			// flash LED lights
			if ((getus()/1000)%250 > 125)
				led_all_on();
			else
				led_all_off();

			// RC pass through except throttle
			for(int i=0; i<6; i++)
#if QUADCOPTER == 1
				g_ppm_output[i] = THROTTLE_STOP;
#else
				g_ppm_output[i] = floor(g_ppm_input[i]+0.5);
			g_ppm_output[2] = THROTTLE_STOP;
#endif
			PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);

			// magnetemeter centering
			sensor_data mag_data;
			read_HMC5883(&mag_data.mag[0]);
			for(int i=0; i<3; i++)
			{
				mag_min[i] = min(mag_min[i], mag_data.mag[i]);
				mag_max[i] = max(mag_max[i], mag_data.mag[i]);
				mag_data.mag[i] *= mag_gain.array[i];
			}

			static float last_mag_data[4] = {0,0,0,1};
			float delta[3] = {mag_data.mag[0]-last_mag_data[0], mag_data.mag[1]-last_mag_data[1], mag_data.mag[2]-last_mag_data[2]};
			if (delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] > 600 )
			{
				last_mag_data[0] = mag_data.mag[0];
				last_mag_data[1] = mag_data.mag[1];
				last_mag_data[2] = mag_data.mag[2];
				mag_offset.add_value(last_mag_data);
			}

			if (mag_c++ % 10 == 0)
			{
				mag_offset.get_result(mag_zero.array, &mag_radius);
				TRACE("mag: center=%f,%f,%f, r=%f\r\n", mag_zero.array[0], mag_zero.array[1], mag_zero.array[2], mag_radius);

				// send magnet centering info back for debugging purpose
				int64_t time = getus();

				rf_data to_send;
				controll_data &controll = to_send.data.controll;
				to_send.time = (time & (~TAG_MASK)) | TAG_CTRL_DATA;
				controll.cmd = CTRL_CMD_FEEDBACK;
				controll.reg = CTRL_REG_MAGNET;
				controll.value = mag_radius*1000;
				controll.data[0] = mag_zero.array[0] * 1000;
				controll.data[1] = mag_zero.array[1] * 1000;
				controll.data[2] = mag_zero.array[2] * 1000;

				int tx_result = log((uint8_t*)&to_send, 32);
			}

			delayms(50);
		}

		if (getus() - start_tick < 5000000)
			goto mag_load;

		// save magnetemeter centering values
		for(int i=0; i<3; i++)
		{
			mag_gain.array[i] = 1000.0f/(mag_max[i] - mag_min[i]);
			ERROR("mag[%d] max/min/gain=%d,%d,%.2f\n", i, mag_max[i], mag_min[i], mag_gain.array[i]);
		}
		mag_offset.get_result(mag_zero.array, &mag_radius);
		space_read("magzero", 7, &mag_zero, sizeof(mag_zero),NULL);
		space_read("maggain", 7, &mag_gain, sizeof(mag_gain),NULL);

		// flash all LED to signal success
		led_all_on();
		delayms(500);
		led_all_off();
		delayms(500);
		led_all_on();
		delayms(500);
		led_all_off();
		delayms(500);
	}

	return 0;
}



int sensor_calibration()
{
	// static base value detection
	for(int i=0; i<300; i++)
	{
		long us = getus();

		TRACE("\r%d/300", i);
		#ifndef LITE
		ads1115_go_on();
		#endif
		read_MPU6050(p->accel);
		read_HMC5883(p->mag);
		if (read_baro(ms5611) == 0)
		{
			baro_counter ++;
			ground_pressure += ms5611[0];
			ground_temperature += ms5611[1];
		}

		mpu6050_temperature += p->temperature1  / 340.0f + 36.53f;

#ifndef LITE
		vector gyro = {p->gyro[0], p->gyro[1], p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
#else
		vector gyro = {p->gyro[0], p->gyro[1], p->gyro[2]};
		vector acc = {-p->accel[1], -p->accel[0], -p->accel[2]};
#endif
	vector mag = {(p->mag[2]-mag_zero.array[2]), -(p->mag[0]-mag_zero.array[0]), -(p->mag[1]-mag_zero.array[1])};
		vector_add(&gyro_zero, &gyro);
		vector_add(&accel_avg, &acc);
		vector_add(&mag_avg, &mag);



#if PCB_VERSION == 3
		airspeed_voltage = ads1115_airspeed*0.03f + airspeed_voltage * 0.97f;
#else

		ADC1_SelectChannel(0);
		adc_2_5_V = adc_2_5_V > 0 ? (ADC1_Read()*0.003+adc_2_5_V*0.997) : (ADC1_Read());

		delayus(500);

		ADC1_SelectChannel(1);
		VCC_5V = VCC_5V > 0 ? (ADC1_Read()*0.003+VCC_5V*0.997) : (ADC1_Read());

		ADC1_SelectChannel(VOLTAGE_PIN);
		VCC_motor = VCC_motor > 0 ? (ADC1_Read()*0.003+VCC_motor*0.997) : (ADC1_Read());

		ADC1_SelectChannel(8);
		airspeed_voltage = airspeed_voltage > 0 ? (ADC1_Read()*0.003+airspeed_voltage*0.997) : (ADC1_Read());		
#endif


		// RC pass through
		for(int i=0; i<6; i++)
#if QUADCOPTER == 1
			g_ppm_output[i] = THROTTLE_STOP;
#else
			g_ppm_output[i] = g_ppm_input[i];
#endif

		PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);

		if ((getus()/1000)%50 > 25)
			led_all_off();
		else
			led_all_on();

		while(getus() - us < cycle_time)
			;
	}

	VCC_5V = 5.0f*VCC_5V/adc_2_5_V;
	VCC_3_3V = 2.5f*4095/adc_2_5_V;
	VCC_motor = voltage_divider_factor * 2.5f * VCC_motor/adc_2_5_V;

#if PCB_VERSION == 3
	airspeed_voltage = 2.048f * airspeed_voltage / 32767;
#else
	airspeed_voltage = 2.5 * airspeed_voltage / adc_2_5_V;
#endif


	airspeed_bias = PCB_VERSION == 3 ? airspeed_voltage : ((airspeed_voltage - VCC_5V/2) *  5 / VCC_5V);
	has_airspeed = abs(airspeed_bias)<0.5f;

	TRACE("5V voltage = %.3fV\n", VCC_5V);
	TRACE("3.3V voltage = %.3fV\n", VCC_3_3V);
	TRACE("VCC motor = %.3fV\n\n", VCC_motor);
	TRACE("airspeed voltage = %.4fV, bias=%.4fV\n\n", airspeed_voltage, airspeed_bias);

	if (VCC_5V / VCC_motor >= 0.85f && VCC_5V / VCC_motor <= 1.15f)
	{
		voltage_divider_factor = voltage_divider_factor * VCC_5V / VCC_motor;

		TRACE("motor factor fix!\n");
		voltage_divider_factor.save();
	}

	groundA = accel_avg;
	groundM = mag_avg;

	vector_divide(&gyro_zero, 300);
	vector_divide(&accel_avg, 300);
	vector_divide(&mag_avg, 300);
	mpu6050_temperature /= 300;
	ground_pressure /= baro_counter * 100;
	ground_temperature /= baro_counter * 100;

	estAccGyro = accel_avg;
	estGyro= estMagGyro = mag_avg;
	accel_1g = vector_length(&accel_avg);	

	if (!isnan((float)gyro_bias[0][0]) && !isnan((float)gyro_bias[1][0]))
	{
		float dt = gyro_bias[1][0] - gyro_bias[0][0];
		if (dt > 1.0f)
		{
			for(int i=0; i<3; i++)
			{
				gyro_temp_a.array[i] = gyro_bias[0][i+1];
				gyro_temp_k.array[i] = (gyro_bias[1][i+1] - gyro_bias[0][i+1]) / dt;
			}
			temperature0 = gyro_bias[0][0];
		}
		else
		{
			// treat as one point
			int group = !isnan(gyro_bias[0][0]) ? 0 : (!isnan(gyro_bias[1][0]) ? 1: -1);
			for(int i=0; i<3; i++)
			{
				gyro_temp_a.array[i] = group >= 0 ? gyro_bias[group][i+1] : 0;
				gyro_temp_k.array[i] = 0;
			}
			temperature0 = group != 0 ? gyro_bias[group][0] : 0;
		}
	}
	else
	{
		int group = !isnan(gyro_bias[0][0]) ? 0 : (!isnan(gyro_bias[1][0]) ? 1: -1);
		for(int i=0; i<3; i++)
		{
			gyro_temp_a.array[i] = group >= 0 ? gyro_bias[group][i+1] : 0;
			gyro_temp_k.array[i] = 0;
		}
		temperature0 = group != 0 ? gyro_bias[group][0] : 0;
	}

	/*
	// gyro bias temperature compensating
	int point_count = 0;
	if (!isnan(gyro_bias[0][0]))
		point_count++;
	if (!isnan(gyro_bias[1][0]))
		point_count++;


	if (point_count == 0)
	{
		// this is the first point.
		for(int i=0; i<4; i++)
		{
			gyro_bias[0][i] = i == 0 ? mpu6050_temperature : gyro_zero.array[i-1];
			gyro_bias[0][i].save();
			if (i<3)
				gyro_temp_a.array[i] = gyro_zero.array[i];
		}
	}
	else if (point_count == 1)
	{

	}
	*/

	ERROR("base value measured, %.3f/ %.3f,%.3f,%.3f\n", mpu6050_temperature, gyro_zero.array[0], gyro_zero.array[1], gyro_zero.array[2]);

	return 0;
}

#ifndef LITE
#else
int Mal_Accessed()
{
	return 0;
}
#endif

#ifdef STM32F4
extern "C" int Mal_Accessed(void);
#endif

int usb_lock()
{
	if (Mal_Accessed())
	{
		for(int i=0; i<sizeof(g_ppm_output)/sizeof(g_ppm_output[0]); i++)
		{
#if QUADCOPTER == 1
			g_ppm_output[i] = THROTTLE_STOP;
#else
			g_ppm_output[i] = i==2?THROTTLE_STOP : RC_CENTER;
#endif

		}
		PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);
	}

	return -1;
}


float last_ch4 = 0;
int64_t arm_start_tick = 0;
int check_mode()
{

	if (g_ppm_input_update[4] > getus() - RC_TIMEOUT || !has_5th_channel)
	{
#if QUADCOPTER == 1
		if (mode == initializing)
			mode = shutdown;

		// emergency switch
		if (fabs(rc[4]-last_ch4) > 0.33f)
		{
			mode = shutdown;
			last_ch4 = rc[4];
			ERROR("shutdown!\n");
		}

		// arm action check: throttle minimum, rudder max or min, aileron & elevator near netrual, for 0.5second
		bool arm_action = rc[2] < 0.1f && fabs(rc[0]) < 0.15f && fabs(rc[1]) < 0.15f && fabs(rc[3]) > 0.85f;
		if (!arm_action)
		{
			arm_start_tick = 0;
		}
		else
		{
			if (arm_start_tick > 0)
			{
				if (getus() - arm_start_tick > 500000)
				{
					mode = quadcopter;
					ERROR("armed!\n");
				}
			}
			else
			{
				arm_start_tick = getus();
			}
		}

#else
		if (g_ppm_input[4] < 1333)
			mode = manual;
		else if (g_ppm_input[4] > 1666)
			mode = acrobatic;
		else
		{
			mode = rc_fail;
		}
#endif
	}
	else
	{
		TRACE("warning: RC out of controll");
		mode = rc_fail;	
	}

	// mode changed?
	if (mode != last_mode)
	{
		last_mode = mode;

		target[0] = pos[0];
		target[1] = pos[1];
		target[2] = pos[2];

		target_altitude = NAN;
		ground_temperature = state[0];
		target_climb_rate = -quadcopter_max_descend_rate;
		target_accel = -quadcopter_max_acceleration*2;
		accel_error_pid[0] = target_accel;
		accel_error_pid[1] = 0;
		accel_error_pid[2] = 0;
		yaw_launch = yaw_est;
		collision_detected = 0;
		tilt_us = 0;

#if QUADCOPTER == 1
		for(int i=0; i<3; i++)
		{
			angle_target[i] = angle_pos[i];
			error_pid[i][1] = 0;	//reset integration
			angle_errorI[i] = 0;
		}
#endif

		airborne = false;
		takeoff_ground_altitude = state[0];

		targetVA = estAccGyro;
		targetVM = estMagGyro;

		vector_normalize(&targetVA);
		vector_normalize(&targetVM);

		for(int i=0; i<6; i++)
			rc_zero[i] = RC_CENTER;
	}


	return 0;
}

int osd()
{
	#ifndef LITE
	// artificial horizon
	//while((MAX7456_Read_Reg(STAT) & 0x10) != 0x00); // wait for vsync
	float roll_constrain = limit(roll, -30*PI/180, +30*PI/180);
	float pitch_constrain = limit(pitch, -30*PI/180, +30*PI/180);

	float tan_roll = tan(roll_constrain);
	float tan_pitch = tan(pitch_constrain);


	static int last_osd_pos[31] = {0};
	for(int x = 15-5; x<= 15+5; x++)
	{
		int y = floor(-(x - 15)*12*tan_roll +  18 * 16 * tan_pitch + 0.5f) + 18*8;

		if (y<0 || y > 18*16)
			continue;

		MAX7456_Write_Char_XY(x,last_osd_pos[x-15], 0);
		last_osd_pos[x-15] = y/18;
		MAX7456_Write_Char_XY(x,y/18, y%18+1);
	}

	// climb rate & altitude
	sprintf(climb_rate_string, "%c%.1f", state[0] >0 ? '+' : '-', fabs(state[0]));
	MAX7456_PrintDigitString(climb_rate_string, 0, 8);
	sprintf(climb_rate_string, "%c%.1f", state[1] >0 ? '+' : '-', fabs(state[1]));
	MAX7456_PrintDigitString(climb_rate_string, 0, 9);

	#endif
	return 0;
}

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

int real_log()
{
	// copy to 2nd buffer
	log_pending = 1;
	memcpy(log_buffer[1], log_buffer[0], 32*log_packet_count);
	log_pending = 0;

	// disable USB interrupt to prevent sdcard dead lock
#ifdef STM32F1
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
#endif
#ifdef STM32F4
	NVIC_DisableIRQ(OTG_HS_IRQn);
	//NVIC_DisableIRQ(OTG_FS_IRQn);
	NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn);
	NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn);
#endif
	__DSB();
	__ISB();

	// real saving / sending
	if (log_packet_count == 0)
		return 0;
	for(int i=0; i<log_packet_count; i++)
		log(log_buffer[1] + 32*i, 32);

	// restore USB
#ifdef STM32F1
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
#endif
#ifdef STM32F4
	NVIC_EnableIRQ(OTG_HS_IRQn);
	NVIC_EnableIRQ(OTG_FS_IRQn);
	NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
	NVIC_EnableIRQ(OTG_HS_EP1_OUT_IRQn);
#endif

	return 0;
}


int64_t land_detect_us = 0;
int land_detector()
{
	if (rc[2] < 0.1f				// low throttle
		&& fabs(state[1]) < (quadcopter_max_descend_rate/4.0f)			// low climb rate : 25% of max descend rate should be reached in such low throttle, or ground was touched
// 		&& fabs(state[2] + state[3]) < 0.5f			// low acceleration
	)
	{
		land_detect_us = land_detect_us == 0 ? getus() : land_detect_us;

		if (getus() - land_detect_us > (airborne ? 1000000 : 3000000))		// 2 seconds for before take off, 1 senconds for landing
		{
			mode = shutdown;
			ERROR("landing detected");
		}
	}
	else
	{
		land_detect_us = 0;
	}
	
	return 0;
}

int crash_detector()
{
	// always detect high G force
	vector ground = {0,0,-2000};		// not very precise, but should be enough
	vector accel_delta;
	for(int i=0; i<3; i++)
		accel_delta.array[i] = accel.array[i] - estAccGyro.array[i];

	float gforce = vector_length(&accel_delta) / accel_1g;
	if (gforce > 1.25f)
	{
		ERROR("high G force (%.2f) detected\n", gforce);
		collision_detected = getus();
	}

	// forced shutdown if >3g external force
	if (gforce > 3.0f)
	{
		ERROR("very high G force (%.2f) detected\n", gforce);
		mode = shutdown;
	}

	int prot = (float)::crash_protect;

	// tilt detection
	if (rc[2] < 0.1f || prot & CRASH_TILT_IMMEDIATE)
	{
		if (vector_angle(&ground, &estAccGyro) < 0.33)		// around 70 degree
			tilt_us = tilt_us > 0 ? tilt_us : getus();
		else
			tilt_us = 0;
	}

	if (((collision_detected > 0 && getus() - collision_detected < 5000000) && (rc[2] < 0.1f || prot & CRASH_COLLISION_IMMEDIATE)) 
		|| (tilt_us> 0 && getus()-tilt_us > 1000000))	// more than 1 second
	{
		ERROR("crash landing detected(%s)\n", (collision_detected > 0 && getus() - collision_detected < 5000000) ? "collision" : "tilt");

		mode = shutdown;
	}

	return 0;
}

float ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert)
{
	float v = (ppm-center_rc) / (ppm>center_rc ? (max_rc-center_rc) : (center_rc-min_rc));

	v = limit(v, -1, +1);

	if (revert)
		v = -v;

	return v;
}

int64_t tic = 0;
int loop(void)
{
	// the main loop
	int64_t round_start_tick = getus();
	interval = (round_start_tick-last_tick)/1000000.0f;
	last_tick = round_start_tick;

	// rc inputs
	TRACE("\rRC");
	for(int i=0; i<8; i++)
	{
		rc[i] = ppm2rc(g_ppm_input[i], rc_setting[i][0], rc_setting[i][1], rc_setting[i][2], rc_setting[i][3] > 0);
		TRACE("%.2f,", rc[i]);
	}

	rc[2] = (rc[2]+1)/2;


	// usb
	usb_lock();	// lock the system if usb transfer occurred


	led_all_off();
	#ifndef LITE
	if (nrf_ok && cycle_counter % 4 == 0)
		NRF_RX_Mode();
	#endif
	
	if (getus() - tic > 1000000)
	{
		tic = getus();
		ERROR("speed: %d\r\n", cycle_counter);
		cycle_counter = 0;
	}

	cycle_counter++;

	// flashlight
	#ifndef LITE
	time = getus();
	int time_mod_1500 = (time%1500000)/1000;
	if (time_mod_1500 < 150 || (time_mod_1500 > 200 && time_mod_1500 < 350) || (time_mod_1500 > 400 && time_mod_1500 < 550 && sd_ok))
		flashlight_on();
	else
		flashlight_off();
	#endif

	// RC modes and RC fail detection
	check_mode();

	// read sensors and update altitude if new air pressure data arrived.
	read_sensors();

	// attitude and  heading
	calculate_attitude();

	// gps		
	#ifndef LITE
	if (GPS_ParseBuffer() > 0)
		last_gps_tick = getus();
	#endif


	//osd();
	calculate_position();
	calculate_target();
	pid();
	output();
	save_logs();

	if (mode == quadcopter)
	{
		land_detector();
		crash_detector();
	}
	else
		land_detect_us = 0;



	TRACE("\rroll,pitch,yaw/yaw2 = %f,%f,%f,%f, target roll,pitch,yaw = %f,%f,%f, error = %f,%f,%f", roll*PI180, pitch*PI180, yaw_est*PI180, yaw_gyro*PI180, target[0]*PI180, target[1]*PI180, target[2]*PI180,
		error_pid[0][0]*PI180, error_pid[1][0]*PI180, error_pid[2][0]*PI180);

	TRACE("time=%.2f,inte=%.4f,out= %d, %d, %d, %d, input=%f,%f,%f,%f\n", getus()/1000000.0f, interval, g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_input[0], g_ppm_input[1], g_ppm_input[3], g_ppm_input[5]);
	TRACE ("\r mag=%.2f,%.2f,%.2f  acc=%.2f,%.2f,%.2f ", estMagGyro.V.x, estMagGyro.V.y, estMagGyro.V.z, estAccGyro.V.x, estAccGyro.V.y, estAccGyro.V.z);
	TRACE ("\racc=%d,%d,%d ", p->accel[0], p->accel[1], p->accel[2]);
	TRACE("\rinput= %.2f, %.2f, %.2f, %.2f,%.2f,%.2f", g_ppm_input[0], g_ppm_input[1], g_ppm_input[2], g_ppm_input[3], g_ppm_input[4], g_ppm_input[5]);
	TRACE("\rinput:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f, ADC=%.2f", (float)g_ppm_output[0], (float)g_ppm_output[1], (float)g_ppm_output[2], (float)g_ppm_output[3], (float)g_ppm_output[0], (float)g_ppm_output[1], p->voltage/1000.0 );

	TRACE("\ryaw=%.2f, yt=%.2f, mag=%d,%d,%d           ", yaw_est *PI180, yaw_launch*PI180, p->mag[0], p->mag[1], p->mag[2]);

	TRACE("\rv/a=%dmV, %dmA, %.1f mah, %.1f Wh   ", p->voltage, p->current, mah_consumed, wh_consumed);
	TRACE("\rgyroI=%.2f", yaw_est * PI180);

	led_all_on();

	if (ms5611_result == 0)
	{
// 		ERROR("%.2f    %.2f    %.2f    %.2f\n", getus()/1000000.0f, a_raw_altitude, state[0], mpu6050_temperature);
	}

	// read and process a packet
	#ifndef LITE
	rf_data packet;
	if (NRF_Rx_Dat((uint8_t*)&packet) == RX_OK)
	{
		TRACE("packet!!!!\r\n\r\n");
	}
	#endif
	
	// wait for next 8ms and send all data out
	#ifndef LITE
	NRF_TX_Mode();
	#endif

	return 0;
}

int int288 = 0;
int power = 0;
int int61152 = 0;
int int2882 = 0;

#include "common/space.h"
int main(void)
{
	//Basic Initialization
	init_timer();
	SysClockInit();
	#ifndef LITE
	sdcard_init();
	#endif

	// priority settings
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	

	// USB
#if defined(STM32F1)
#ifndef LITE
	Set_System();
#endif
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
#endif

#ifdef STM32F4
	USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
		USB_OTG_HS_CORE_ID,
#else
		USB_OTG_FS_CORE_ID,
#endif
		&USR_desc,
		&USBD_CDC_cb,
		&USR_cb);
#endif

	ADC1_Init();
	SysTick_Config(720);
	PPM_init(1);
	printf_init();
	I2C_init(0x30);
	debugpin_init();
	if (init_MPU6050() < 0)
		critical_errors |= error_accelerometer | error_gyro;
	#ifndef LITE
	if (init_MS5611() < 0)
		critical_errors |= error_baro;
	if (init_HMC5883() < 0)
		critical_errors |= error_magnet;	
	GPS_Init(115200);
	NRF_Init();
// 	MAX7456_SYS_Init();
// 	Max7456_Set_System(1);
	sonar_init();
	#else
	if (init_BMP085() < 0)
		critical_errors |= error_baro;
	#endif
	flashlight_on();

	#if PCB_VERSION == 3 && !defined(LITE)
	int16_t int6115 = 0;
	float vpower = NAN;
	float v6115;
	float v288;
	float v61152;
	ads1115_init();	
// 	ads1115_new_work(ads1115_speed_860sps, ads1115_channnel_AIN0, ads1115_gain_4V, &ads1115_voltage);
// 	ads1115_new_work(ads1115_speed_860sps, ads1115_channnel_AIN1_AIN3, ads1115_gain_4V, &ads1115_current);
// 	ads1115_new_work(ads1115_speed_860sps, ads1115_channnel_AIN2_AIN3, ads1115_gain_2V, &ads1115_airspeed);
// 	ads1115_new_work(ads1115_speed_860sps, ads1115_channnel_AIN3, ads1115_gain_4V, &ads1115_2_5V);
	//ads1115_new_work(ads1115_speed_8sps, ads1115_channnel_AIN3, ads1115_gain_6V, &power);
	//ads1115_new_work(ads1115_speed_8sps, ads1115_channnel_AIN1, ads1115_gain_4V, &int61152);
	//ads1115_new_work(ads1115_speed_8sps, ads1115_channnel_AIN1_AIN3, ads1115_gain_1V, &int6115);
	//ads1115_new_work(ads1115_speed_8sps, ads1115_channnel_AIN2, ads1115_gain_4V, &int288);

	if(0)
	{
		init_hp203b();

		while(1)
		{
			int data[2];
			if (read_hp203b(data) == 0)
			{
				printf("data=%d(%08x),%f\n", data[0], data[0], data[1]/256.0f);
			}
		}
	}
	#endif


	if(0)
	{
		space_init();

		char test[200] = "world";
		char key[200] = "hello";
		char test2[200];
		char key2[200];

		int got = 0;
		int write_result;
		int read_result;

		read_result = space_read(key, strlen(key), test2, sizeof(test2), &got);
		ERROR("read:%d, %s", read_result, test2);
		write_result = space_write(key, strlen(key), test, strlen(test), &got); 
		read_result = space_read(key, strlen(key), test2, sizeof(test2), &got);
		read_result = space_read(key, strlen(key), test2, sizeof(test2), &got); 

		strcpy(test, "world2");

		for(int i=0; i<100; i++)
			write_result = space_write("key2", 4, "k345", 4, &got);
		write_result = space_write(key, strlen(key), test, strlen(test), &got); 
		space_init();
		// 	write_result = space_delete(key, strlen(key));
		// 	read_result = space_read(key, strlen(key), test2, sizeof(test2), &got); 

 		//int resort_result = space_resort();

		space_init();

		read_result = space_read("key2", 4, test2, sizeof(test2), &got); 
		read_result = space_read(key, strlen(key), test2, sizeof(test2), &got); 
		ERROR("read:%d, %s", read_result, test2);
	}

	#if !defined(LITE) && !defined(STM32F4) && 0
	ads1256_init();

	ads1256_begin();
	ads1256_tx_rx(CMD_Reset);
	ads1256_end();
	delayms(100);
	
	
	ads1256_status status;
	ads1256_read_registers(REG_Status, 1, &status);
	status.BufferEnable = 0;
	status.AutoCalibration = 1;
	ads1256_write_registers(REG_Status, 1, &status);

	uint8_t speed = 0;
// 	ads1256_mux mux = {ads1256_channnel_AIN6, ads1256_channnel_AIN7};
// 	ads1256_write_registers(REG_MUX, 1, &mux);
// 	ads1256_read_registers(REG_MUX, 1, &mux);
// 	ERROR("1256:mux register = %02x\n", *(uint8_t*)&mux);

	delayms(50);
	ads1256_read_registers(REG_DataRate, 1, &speed);
	delayms(50);
	ERROR("1256:speed register = %02x\n", speed);
	speed = ads1256_speed_100sps;
	ERROR("1256:speed register writing %02x\n", speed);
	ads1256_write_registers(REG_DataRate, 1, &speed);
	delayms(50);
	ads1256_read_registers(REG_DataRate, 1, &speed);
	ERROR("1256:speed register = %02x\n", speed);

	ERROR("time,vpower,v6115,v288,Pa6115,Pa288,ms5611,m\r\n");

// 	ads1256_begin();
// 	ads1256_tx_rx(CMD_SystemOffsetCalibration);
// 	ads1256_end();
// 	delayms(15);


	ads1256_begin();
	ads1256_tx_rx(CMD_OffsetGainCalibration);
	ads1256_end();
	delayms(150);

	while(1)
	{
		status.DataReady = 1;

		while(status.DataReady == 1)		
			ads1256_read_registers(REG_Status, 1, &status);

		status.DataReady = 1;
		ads1256_begin();
		ads1256_tx_rx(CMD_Sync);
		ads1256_tx_rx(CMD_WakeUp);
		ads1256_end();

		while(status.DataReady == 1)
		{
			//ERROR("wait..\n");
			ads1256_read_registers(REG_Status, 1, &status);
		}

		//ERROR("ads1256 status=%02x, d=%d\n", *(uint8_t*)&status, status.DataReady);


		if (status.DataReady == 0)
		{
			uint8_t data[4];
			ads1256_begin();
			ads1256_tx_rx(CMD_ReadData);
			delayus(15);
			for(int i=0; i<3; i++)
				data[i+1] = ads1256_tx_rx(0);
			data[0] = (data[1] & 0x80) ? 0xff : 0x00;
			swap(data, 4);
			ads1256_end();

// 			ERROR("data=%08x(%fV)\n", *(int*)data, *(int*)data * 5.0f / 8388607.0f);
// 			ads1256_read_register(REG_Status, 1, &status);
// 			ERROR("ads1256 status=%d, d=%d\n", *(uint8_t*)&status, status.DataReady);


			float v6115 = *(int*)data * 5.0f / 8388607.0f;
			float pa_6115 = 15000 + (v6115-5.0*0.05f)/(0.9f*5.0)*100000.0f;
			ERROR("%.2f,%f\n", pa_6115, v6115);

		}

		/*
		//int adc = ads1115_go_on();
		int oss = 64;
		power = 0;
		ads1115_config(ads1115_speed_250sps, ads1115_channnel_AIN3, ads1115_gain_6V, ads1115_mode_singleshot);
		for(int i=0; i<oss; i++)
			power += ads1115_convert();

		int61152 = 0;
		ads1115_config(ads1115_speed_250sps, ads1115_channnel_AIN1, ads1115_gain_4V, ads1115_mode_singleshot);
		for(int i=0; i<oss; i++)
			int61152 += ads1115_convert();


		int288 = 0;
		ads1115_config(ads1115_speed_250sps, ads1115_channnel_AIN2, ads1115_gain_4V, ads1115_mode_singleshot);
		for(int i=0; i<oss; i++)
			int288 += ads1115_convert();

		int2882 = 0;
		ads1115_config(ads1115_speed_250sps, ads1115_channnel_AIN2, ads1115_gain_6V, ads1115_mode_singleshot);
		for(int i=0; i<oss; i++)
			int2882 += ads1115_convert();



		vpower = power/32767.0f*6.0f/oss;
		float vpower2 = power/32767.0f*6.0f/oss;
		v6115 = vpower + int6115*1.0f/32767.0f;
		v288 = int2882*6.0f/oss/32767.0f;
		v61152 = int61152/32767.0f*4.0f/oss;



		float pa_6115 = 15000 + (v6115-vpower*0.05f)/(0.9f*vpower)*100000.0f;
		float pa_61152 = 15000 + (v61152-vpower*0.05f)/(0.9f*vpower)*100000.0f;
		float pa_288 = 50000.0f + (v288-vpower*0.06f)/(0.9f*vpower)*65000.0f;
		pa_288 = ((v288/vpower) +0.095f)/0.009f*1000.0f;

		read_baro(ms5611);
		delayms(20);
		read_baro(ms5611);
		delayms(20);
		read_baro(ms5611);
		delayms(20);
		read_baro(ms5611);

		//ERROR("%.3f,%f,%f,%f,%.3f,%.3f,%d,%f\r\n", getus()/1000000.0f, vpower2, v61152, v288, pa_6115, pa_288,ms5611[0], pa_61152);
		*/

	}
	
	delayms(100);
	#endif
	
	#ifndef LITE
	NRF_Init();
	nrf_ok = 0 == NRF_Check();
	TRACE("NRF_Check() = %d\r\n", nrf_ok);
	if (nrf_ok)
		NRF_TX_Mode();
	#endif
	
	p->voltage = -32768;
	p->current = -32768;

			
	magnet_calibration();
	sensor_calibration();

	has_5th_channel = g_ppm_input_update[4] > getus();

	// check critical errors
	if (critical_errors != 0)
	{
		led_all_off();
		flashlight_off();

		while (1)
		{
			// blink error code!
			for(int i=1; i<error_MAX; i<<=1)
			{
				led_all_on();
				flashlight_on();

				if (critical_errors & i)
					delayms(500);
				else
					delayms(150);

				led_all_off();
				flashlight_off();
				delayms(150);
			}

			delayms(1500);
		}
	}


	
	#ifndef LITE
	// check NRF again
	if (!nrf_ok)
		nrf_ok = 0 == NRF_Check();

	// open NRF TX Mode if found
	if (nrf_ok)
		NRF_TX_Mode();

	TRACE("NRF_Check() 2 = %d\r\n", nrf_ok);
	#endif


	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// set timer(TIM1) for main loop
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM_DeInit(TIM1);
	TIM_InternalClockConfig(TIM1);
#ifdef STM32F1
	TIM_TimeBaseStructure.TIM_Prescaler=71;
#endif
#ifdef STM32F4
	TIM_TimeBaseStructure.TIM_Prescaler=167;
#endif
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=3000-1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM1,DISABLE);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM1,ENABLE);

#ifdef STM32F1
	#ifdef __GNUC__
	#define TIM1_UP_IRQn TIM1_UP_TIM10_IRQn
	#endif
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
#endif
#ifdef STM32F4
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
#endif
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);



#if !defined(LITE) && !defined(STM32F1)
	// set timer(TIM12) for log saving task
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
	TIM_DeInit(TIM12);
	TIM_InternalClockConfig(TIM12);
#ifdef STM32F1
	TIM_TimeBaseStructure.TIM_Prescaler=71;
#endif
#ifdef STM32F4
	TIM_TimeBaseStructure.TIM_Prescaler=83;
#endif
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=15000-1;
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM12,TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM12,DISABLE);
	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM12,ENABLE);

 	NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	while(1)
	{

	}

	return -2;
}


extern "C"
{
#ifdef __GNUC__
#define TIM1_UP_IRQHandler TIM1_UP_TIM10_IRQHandler
#endif

#ifdef STM32F1
void TIM1_UP_IRQHandler(void)
#endif
#ifdef STM32F4
void TIM1_UP_TIM10_IRQHandler(void)
#endif
{
	TIM_ClearITPendingBit(TIM1 , TIM_FLAG_Update);
	loop();
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM12 , TIM_FLAG_Update);
	real_log();
}
}

// assume all vector are normalized
// return true if reliable roll pitch target is calculated, false if unreliable
bool calculate_roll_pitch(vector *accel, vector *mag, vector *accel_target, vector *mag_target, float *roll_pitch)
{
	bool got_roll = false;
	bool got_pitch = false;

	// use accelerometer first
	if (accel->V.z*accel->V.z + accel->V.x * accel->V.x > ACCELEROMETER_THRESHOLD)
	{
		float roll1 = atan2(accel->V.z, accel->V.x);
		float roll2 = atan2(accel_target->V.z, accel_target->V.x);

		roll_pitch[0] = radian_sub(roll2, roll1);
		got_roll = true;
	}

	if (accel->V.z*accel->V.z + accel->V.y * accel->V.y > ACCELEROMETER_THRESHOLD)
	{
		float pitch1 = atan2(accel->V.z, accel->V.y);
		float pitch2 = atan2(accel_target->V.z, accel_target->V.y);

		roll_pitch[1] = radian_sub(pitch2, pitch1);
		got_pitch = true;
	}

	// use mag to correct roll on diving
	if (!got_roll && mag->V.z*mag->V.z + mag->V.x * mag->V.x > MAG_THRESHOLD)
	{
		float roll1 = atan2(mag->V.z, mag->V.x);
		float roll2;

		// some times mag_target can become unreliable, in this case, use (mag_target+mag)/2 as mag_target.
		// for example: the plane is diving and target is level flight, mag is reliable, but mag_target can be unreliable for roll target
		// there is at least one among mag_target and (mag_target+mag)/2 is reliable
		if (mag_target->V.z*mag_target->V.z + mag_target->V.x * mag_target->V.x > MAG_THRESHOLD)
		{
			roll2 = atan2(mag_target->V.z, mag_target->V.x);
		}
		else
		{
			vector target = *mag_target;
			vector_add(&target, mag);
			vector_multiply(&target, 0.5);

			roll2 = atan2(target.V.z, target.V.x);

		}

		roll_pitch[0] = radian_sub(roll2, roll1);
		got_roll = true;
	}

	// use mag to correct picth on knife edge
	if (!got_pitch && mag->V.z*mag->V.z + mag->V.y * mag->V.y > MAG_THRESHOLD)
	{
		float pitch1 = atan2(mag->V.z, mag->V.y);
		float pitch2;

		// some times mag_target can become unreliable, in this case, use (mag_target+mag)/2 as mag_target.
		// for example: the plane is diving and target is level flight, mag is reliable, but mag_target can be unreliable for roll target
		// there is at least one among mag_target and (mag_target+mag)/2 is reliable
		if (mag_target->V.z*mag_target->V.z + mag_target->V.y * mag_target->V.y > MAG_THRESHOLD)
		{
			pitch2  = atan2(mag_target->V.z, mag_target->V.y);
		}
		else
		{
			vector target = *mag_target;
			vector_add(&target, mag);
			vector_multiply(&target, 0.5);

			pitch2  = atan2(target.V.z, target.V.y);
		}
		
		roll_pitch[1] = radian_sub(pitch2, pitch1);
		got_pitch = true;
	}


	return got_roll && got_pitch;
}


// System Clock
static void SysClockInit(void)
{
#ifdef STM32F1
	RCC_DeInit();

	RCC_HSEConfig(RCC_HSE_ON);
	RCC_WaitForHSEStartUp();


	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

	FLASH_SetLatency(FLASH_Latency_2);

	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
	RCC_PLLCmd(ENABLE);

	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	while(RCC_GetSYSCLKSource() != 0x08)
	{
	}
#endif
}

#ifndef LITE

int ads1115_new_work(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, int16_t *out)
{
	if (ads1115_work_count >= MAX_ADS1115_WORKS)
		return -1;

	ads1115_work &p = ads1115_works[ads1115_work_count++];
	p.speed = speed;
	p.channel = channel;
	p.gain = gain;
	p.out = out;

	if (ads1115_work_count == 1)
	{
		ads1115_config(speed, channel, gain, ads1115_mode_singleshot);
		ads1115_startconvert();
	}
	
	return 0;
}

int ads1115_go_on()
{
	if (ads1115_work_count == 0)
		return 1;

	int res = ads1115_getresult(ads1115_works[ads1115_work_pos].out);
	if (res == 0)
	{
		ads1115_work_pos = (ads1115_work_pos+1)%ads1115_work_count;
		ads1115_work &p = ads1115_works[ads1115_work_pos];
		ads1115_config(p.speed, p.channel, p.gain, ads1115_mode_singleshot);
		ads1115_startconvert();
		return 0;
	}

	return 1;
}
#endif
