#include <stdio.h>
#include <stm32f10x.h>
#include <misc.h>
#include <math.h>
#include <string.h>
#include <stm32f10x_usart.h>

#include "RFData.h"
#include "common/adc.h"
#include "common/printf.h"
#include "common/I2C.h"
#include "common/NRF24L01.h"
#include "common/PPM.h"
#include "common/common.h"
#include "common/vector.h"
#include "common/config.h"
#include "sensors/HMC5883.h"
#include "sensors/MPU6050.h"
#include "sensors/MS5611.h"
#include "sensors/mag_offset.h"
#include "common/gps.h"
#include "common/config.h"
#include "common/eeprom.h"
#include "common/ads1115.h"
#include "fat/ff.h"
#include "fat/sdcard.h"
#include "osd/MAX7456.h"


extern "C"
{
#include "fat/diskio.h"
#include "osd/osdcore.h"
}

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
class pilot
{
public:
	// funcs
	int device_init();
	int state_init();
	int read_sensors();
	int ahrs();
	int target();
	int pid();
	int output();
	int main();
	
	// state
};


// a helper
bool calculate_roll_pitch(vector *accel, vector *mag, vector *accel_target, vector *mag_target, float *roll_pitch);



// states
bool sd_ok = false;
bool nrf_ok = false;
FIL *file = NULL;
FRESULT res;
FATFS fs;
uint64_t last_log_flush_time = 0;
bool launched = false;
float mpu6050_temperature;
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
static float temperature = 0;
static float pressure = 0;
float ground_pressure = 0;
float ground_temperature = 0;
float climb_rate = 0;
float climb_rate_lowpass = 0;
float climb_rate_filter[7] = {0};			// 7 point Derivative Filter(copied from ArduPilot), see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/#noiserobust_2
float climb_rate_filter_time[7] = {0};
float accelz = 0;
bool airborne = false;
float takeoff_ground_altitude = 0;
int mode = initializing;
u8 data[TX_PLOAD_WIDTH];
static sensor_data *p = (sensor_data*)data;
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
double altitude = 0;
int ms5611[2];
int ms5611_result = -1;
float adc_2_5_V = -1;
float VCC_3_3V = -1;
float VCC_5V = -1;
float VCC_motor = -1;
float airspeed_voltage = -1;
float voltage_divider_factor = 6;
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
float a_accel_z = 0;
float a_climb2_tick = 0;
float a_climb_rate_filter[7] = {0};			// 7 point Derivative Filter(copied from ArduPilot), see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/#noiserobust_2
float a_climb_rate_filter_time[7] = {0};

float _time_constant_z = 2.0f;
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
float target_climb_rate = 0;
float target_accel = 0;
float altitude_error_pid[3] = {0};
float climb_rate_error_pid[3] = {0};
float accel_error_pid[3] = {0};
int throttle_result = 0;

int16_t ads1115_2_5V = 0;
int16_t ads1115_airspeed = 0;
int16_t ads1115_voltage = 0;
int16_t ads1115_current = 0;

int auto_throttle(float user_climb_rate)
{
	// new target altitude
	target_altitude = limit(target_altitude + user_climb_rate * interval, _position - 2.5, _position + 2.5);

	// new altitude error
	altitude_error_pid[0] = target_altitude - _position;

	// new target rate, directly use linear approach since we use very tight limit 
	// TODO: use sqrt approach on large errors (see get_throttle_althold() in Attitude.pde)
	// TODO: apply pid instead of P only
	target_climb_rate = pid_quad_altitude[0] * altitude_error_pid[0];
	target_climb_rate = limit(target_climb_rate, -quadcopter_max_decend_rate, quadcopter_max_climb_rate);


	// new climb rate error
	float climb_rate_error = target_climb_rate - _velocity;

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
	climb_rate_error_pid[0] = climb_rate_error_pid[0] * (1-alpha) + alpha * climb_rate_error;
	target_accel = climb_rate_error_pid[0] * pid_quad_alt_rate[0];
	target_accel = limit(target_accel, airborne ? -quadcopter_max_acceleration : -2*quadcopter_max_acceleration, quadcopter_max_acceleration);

	
	// new accel error, +2Hz LPF
	float accel_error = target_accel - (accelz + _accel_correction_ef);
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

	throttle_result  = output/500.0f * (THROTTLE_MAX - THROTTLE_CRUISE) + THROTTLE_CRUISE;
	throttle_result = limit(throttle_result, THROTTLE_IDLE, THROTTLE_MAX);

	TRACE("\rthrottle=%d, altitude = %.2f/%.2f, pid=%.2f,%.2f,%.2f", throttle_result, _position, target_altitude,
		accel_error_pid[0], accel_error_pid[1], accel_error_pid[2]);

	return 0;
}

int climb2()
{
	// delta time
	float time = getus()/1000000.0f;
	float delta_time = a_climb2_tick == 0 ? 0 : (time - a_climb2_tick);
	a_climb2_tick = time;

	// raw altitude
	float scaling = (float)a_raw_pressure / ground_pressure;
	float temp = ((float)ground_temperature) + 273.15f;
	a_raw_altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));

	/*
	// raw_altitude --LPF-->> altitude
	// 0.3Hz low pass filter
	if isnan(a_altitude)
		a_altitude = a_raw_altitude;
	const float RC = 1.0f/(2*3.1415926 * 0.03f);
	float alpha = delta_time / (delta_time + RC);
	a_altitude = a_raw_altitude * alpha + (1-alpha) * a_altitude;

	// altitude ---7 point Derivative Filter -->> raw_climb
	memmove(a_climb_rate_filter, a_climb_rate_filter+1, sizeof(float)*6);
	memmove(a_climb_rate_filter_time, a_climb_rate_filter_time+1, sizeof(float)*6);
	a_climb_rate_filter[6] = altitude;
	a_climb_rate_filter_time[6] = time;
	a_raw_climb =   2 * 5 * (a_climb_rate_filter[3] - a_climb_rate_filter[5]) / (a_climb_rate_filter_time[3] - a_climb_rate_filter_time[5]) +
		4 * 4 * (a_climb_rate_filter[2] - a_climb_rate_filter[6]) / (a_climb_rate_filter_time[3] - a_climb_rate_filter_time[5]) + 
		6 * 1 * (a_climb_rate_filter[2] - a_climb_rate_filter[6]) / (a_climb_rate_filter_time[3] - a_climb_rate_filter_time[5]);
	a_raw_climb /= 32.0f;

	// raw_climb ---LPF(0.3hz)--->> climb_rate
	const float RC2 = 1.0f/(2*3.1415926 * 0.3f);
	float alpha2 = delta_time / (delta_time + RC2);
	a_climb = a_raw_climb * alpha2 + (1-alpha2) * a_climb;

	// calculate acceleration into ground including gravity
	// a = (gravity estamation vector * accelerometer vector) / | gravity estamation vector | / | 1g reference vector | * g
	vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
	float dot = acc.V.x * estAccGyro.V.x + acc.V.y * estAccGyro.V.y + acc.V.z * estAccGyro.V.z;
	dot /= vector_length(&estAccGyro);
	dot /= accel_1g;
	a_accel_z = 9.80f * (dot-1);

	// acceleration + baro climb rate CF fusion
	a_climb = a_climb * alpha2 + (1-alpha2) * (a_climb + a_accel_z * delta_time);


	// climb + altitudee CF fusion
	a_altitude = a_altitude * alpha2 + (1-alpha2) * (a_altitude + a_climb * delta_time);

	char test[20];
	sprintf(test, "altitude= %.3f, climb_rate= %.3f, p_ac=%.3f,%.3f", _position, _velocity, altitude, a_accel_z);
	printf("%s\r", test);
	MAX7456_PrintDigitString(test, 0, 0);
	
	*/

	// apm test
	_position_error = a_raw_altitude - (_position_base + _position_correction);


	return 0;
}

int apm()
{
	float &dt = interval;
	vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
	float dot = acc.V.x * estAccGyro.V.x + acc.V.y * estAccGyro.V.y + acc.V.z * estAccGyro.V.z;
	dot /= vector_length(&estAccGyro);
	dot /= accel_1g;
	a_accel_z = 9.80f * (dot-1);

	_accel_correction_ef += _position_error * _k3_z  * dt;
	_velocity_correction += _position_error * _k2_z  * dt;
	_position_correction += _position_error * _k1_z  * dt;

	const float velocity_increase = (a_accel_z + _accel_correction_ef) * dt;
	_position_base += (_velocity_base + _velocity_correction + velocity_increase*0.5) * dt;
	_velocity_base += velocity_increase;

	_position = _position_base + _position_correction;
	_velocity = _velocity_base + _velocity_correction;

	return 0;
}

int sdcard_init()
{
	// SD CARD
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	TRACE("sdcard init...");
	FIL f;
  res = disk_initialize(0) == RES_OK ? FR_OK : FR_DISK_ERR;
	res = f_mount(&fs, "", 0);
	res = f_open(&f, "test.bin", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	f_close(&f);
	sd_ok = res == FR_OK;

	TRACE("%s\r\n", sd_ok ? "OK" : "FAIL");
	return 0;
}

int log(void *data, int size)
{
	int64_t us = getus();
	
	// NRF
	if (nrf_ok && size <= 32 && LOG_LEVEL & LOG_NRF)
		NRF_Tx_Dat((u8*)data);
	
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
		TRACE("log cost %d us\r\n\r\n", int(getus()-us));

	return 0;
}


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
			float v = (g_ppm_input[2] - THROTTLE_STOP) / (THROTTLE_MAX - THROTTLE_STOP) - 0.5f;
			v = limit(v, -0.5f, 0.5f);
			float user_rate;
			if (abs(v)<0.1f)
				user_rate = 0;
			else if (v>= 0.1f)
				user_rate = (v-0.1f)/0.4f * quadcopter_max_climb_rate;
			else
				user_rate = (v+0.1f)/0.4f * quadcopter_max_decend_rate;
			auto_throttle(user_rate);


			// first, calculate target angle
			// roll & pitch, RC trim is accepted.
			for(int i=0; i<2; i++)
			{
				float limit_l = angle_target_unrotated[i] - 2 * PI * interval;
				float limit_r = angle_target_unrotated[i] + 2 * PI * interval;
				angle_target_unrotated[i] = limit(-(g_ppm_input[i] - RC_CENTER) * rc_reverse[i] / RC_RANGE, -1, 1) * quadcopter_range[i];
				angle_target_unrotated[i] = limit(angle_target_unrotated[i], limit_l, limit_r);
				angle_target[i] = angle_target_unrotated[i];
			}

			// simple mode test
			if (g_ppm_input[5] > RC_CENTER)
			{
				float diff = yaw_launch - yaw_est;
				float cosdiff = cos(diff);
				float sindiff = sin(diff);
				angle_target[0] = angle_target_unrotated[0] * cosdiff - angle_target_unrotated[1] * sindiff;
				angle_target[1] = angle_target_unrotated[0] * sindiff + angle_target_unrotated[1] * cosdiff;
			}

			//trim
			angle_target[0] += quadcopter_trim[0];
			angle_target[1] += quadcopter_trim[1];

			// yaw:
			//target[2] = limit((g_ppm_input[3] - RC_CENTER) * rc_reverse[2] / RC_RANGE, -1, 1) * quadcopter_range[2] + yaw_gyro + quadcopter_trim[2];
			float rc = g_ppm_input[3] - rc_zero[3];
			if (abs(rc) < RC_DEAD_ZONE)
				rc = 0;
			else
				rc *= QUADCOPTER_ACRO_YAW_RATE * interval / RC_RANGE;

			rc_d[2] = rc * rc_reverse[2] * sensor_reverse[2];

			float trimmed_pos = radian_add(angle_pos[2], quadcopter_trim[2]);
			float new_target = radian_add(angle_target[2], rc_d[2]);
			float new_error = abs(radian_sub(trimmed_pos, new_target));
			if (new_error > QUADCOPTER_MAX_YAW_OFFSET && new_error > abs(angle_error[2]))
				rc_d[2] = 0;
			else
				angle_target[2] = new_target;


			// now calculate target angle rate
			// based on a PID stablizer
			for(int i=0; i<3; i++)
			{
				float new_angle_error = radian_sub(angle_pos[i], angle_target[i]);	// use radian_sub mainly for yaw


				// 5hz low pass filter for D, you won't be that crazy, right?
				static const float lpf_RC = 1.0f/(2*PI * 5.0f);
				float alpha = interval / (interval + lpf_RC);

				angle_errorI[i] = angle_error[i] + new_angle_error * interval;
				angle_errorD[i] = (1-alpha) * angle_errorD[i] + alpha * (new_angle_error - angle_error[i]) / interval;
				angle_error[i] = new_angle_error;

				// apply angle pid
				target[i] = - (angle_error[i] * pid_factor2[i][0] + angle_errorI[i] * pid_factor2[i][1] + angle_errorD[i] * pid_factor2[i][2]);

				// max target rate: 180 degree/second
				target[i] = limit(target[i], -PI, PI);
			}
			TRACE("angle pos,target=%f,%f\r\n", angle_pos[2] * PI180, angle_target[2] * PI180);

			// check takeoff
			if ( (altitude > takeoff_ground_altitude + 2.0) ||
				(altitude > takeoff_ground_altitude && g_ppm_input[2] > THROTTLE_CRUISE) ||
				((g_ppm_input[5] > RC_CENTER ? throttle_result : g_ppm_input[2]) > THROTTLE_CRUISE + 200))
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
		error_pid[i][1] = limit(error_pid[i][1], -pid_limit[i][1], pid_limit[i][1]);

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
		float p_rc = limit((g_ppm_input[5] - 1000.0) / 520.0, 0, 2);
		for(int j=0; j<3; j++)
		{
#if QUADCOPTER == 1
			pid_result[i] += error_pid[i][j] * pid_factor[i][j];
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
		g_ppm_output[i] = floor(g_ppm_input[i]+0.5);

	if (mode != rc_fail)
	{
		// throttle pass through
		g_ppm_output[2] = floor(g_ppm_input[2]+0.5);
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
		int motor_count = sizeof(quadcopter_mixing_matrix) / sizeof(quadcopter_mixing_matrix[0]);
		for(int i=0; i<motor_count; i++)
		{
			float mix = mode != rc_fail ? g_ppm_input[2] : THROTTLE_STOP;
			mix = (mix-THROTTLE_STOP)*0.8 + THROTTLE_STOP;

			if (mode != rc_fail && g_ppm_input[5] > RC_CENTER)
				mix = throttle_result;
			for(int j=0; j<3; j++)
				mix += quadcopter_mixing_matrix[i][j] * limit(pid_result[j],-3,3) * QUADCOPTER_MAX_DELTA;

			if (mode == rc_fail)
				g_ppm_output[i] = THROTTLE_STOP;
			else
				g_ppm_output[i] = limit(mix, THROTTLE_IDLE, THROTTLE_MAX);

			TRACE("\rpid[x] = %f, %f, %f", pid[0], pid[1], pid[2]);
		}
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
			g_ppm_output[i] = floor(g_ppm_input[i]+0.5);

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

int save_logs()
{
	// send/store debug data
	time = getus();
	rf_data to_send;
	to_send.time = (time & (~TAG_MASK)) | TAG_SENSOR_DATA;
	to_send.data.sensor = *p;


	int tx_result;
	tx_result = log((u8*)&to_send, 32);

	static int t1 = 0, t2=0;
	if (tx_result == TX_OK)
		t1++;

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

	tx_result = log((u8*)&to_send, 32);
	if (tx_result == TX_OK)
		t2++;

	extern int tx_ok;
	extern int max_retry;
	//TRACE("\r t1,t2,ok,timeout=%d,%d,%d,%d", t1, t2, tx_ok, max_retry);

	pilot_data pilot = 
	{
		altitude * 100,
		airspeed_sensor_data * 1000,
		{error_pid[0][0]*180*100/PI, error_pid[1][0]*180*100/PI, error_pid[2][0]*180*100/PI},
		{target[0]*180*100/PI, target[1]*180*100/PI, target[2]*180*100/PI},
		mode,
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_PILOT_DATA;
	to_send.data.pilot = pilot;
	tx_result = log((u8*)&to_send, 32);

	pilot_data2 pilot2 = 
	{
		{error_pid[0][1]*180*100/PI, error_pid[1][1]*180*100/PI, error_pid[2][1]*180*100/PI},
		{error_pid[0][2]*180*100/PI, error_pid[1][2]*180*100/PI, error_pid[2][2]*180*100/PI},
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_PILOT_DATA2;
	to_send.data.pilot2 = pilot2;
	tx_result = log((u8*)&to_send, 32);

	ppm_data ppm = 
	{
		{g_ppm_input[0], g_ppm_input[1], g_ppm_input[2], g_ppm_input[3], g_ppm_input[4], g_ppm_input[5]},
		{g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_output[4], g_ppm_output[5]},
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_PPM_DATA;
	to_send.data.ppm = ppm;
	tx_result = log((u8*)&to_send, 32);

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
	tx_result = log((u8*)&to_send, 32);

	quadcopter_data2 quad2 = 
	{
		climb_rate * 100,
		airborne,
		_position * 100,
		_velocity * 100,
		a_raw_altitude * 100,
		a_accel_z * 100,
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_QUADCOPTER_DATA2;
	to_send.data.quadcopter2 = quad2;
	tx_result = log((u8*)&to_send, 32);

	quadcopter_data3 quad3 = 
	{
		target_altitude * 100,
		_position * 100,
		target_climb_rate * 100,
		_velocity * 100,
		target_accel * 100,
		(a_accel_z + _accel_correction_ef) * 100,
		throttle_result,
		yaw_launch * 18000 / PI,
		yaw_est * 18000 / PI,
	};

	to_send.time = (time & (~TAG_MASK)) | TAG_QUADCOPTER_DATA3;
	to_send.data.quadcopter3 = quad3;
	tx_result = log((u8*)&to_send, 32);
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

		tx_result = log((u8*)&to_send, 32);
	}



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
		tx_result = log((u8*)&to_send, 32);
	}
	
	return 0;
}


int read_sensors()
{
	// read external adc
	ads1115_go_on();

	// always read sensors and calculate attitude
	read_MPU6050(&p->accel[0]);		
	read_HMC5883(&p->mag[0]);
	for(int i=0; i<3; i++)
		p->mag[i] *= mag_gain.array[i];

	ms5611_result = read_MS5611(ms5611);

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


	// calculate altitude
	if (ms5611_result == 0)
	{
		float this_pressure = ms5611[0] / 100.0f;
		float this_temperature = ms5611[1] / 100.0f;

		long this_baro_time =getus();
		float delta_time = (this_baro_time - last_baro_time)/1000000.0f;


		// 3hz discrete low pass filter
		if (last_baro_time > 0)
		{
			const float RC = 1.0f/(2*3.1415926 * 0.3f);
			float alpha = delta_time / (delta_time + RC);
			pressure = pressure * (1-alpha) + this_pressure * alpha;
			temperature = temperature * (1-alpha) + this_temperature * alpha;
		}




		double scaling = (double)pressure / ground_pressure;
		double temp = ((double)ground_temperature) + 273.15f;
		altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));

		if (fabs(altitude)<5.0f)
			ground_temperature = temperature;

		memmove(climb_rate_filter, climb_rate_filter+1, sizeof(float)*6);
		memmove(climb_rate_filter_time, climb_rate_filter_time+1, sizeof(float)*6);
		climb_rate_filter[6] = altitude;
		climb_rate_filter_time[6] = this_baro_time / 1000000.0f;
		float raw_climb_rate =   2 * 5 * (climb_rate_filter[3] - climb_rate_filter[5]) / (climb_rate_filter_time[3] - climb_rate_filter_time[5]) +
			4 * 4 * (climb_rate_filter[2] - climb_rate_filter[6]) / (climb_rate_filter_time[3] - climb_rate_filter_time[5]) + 
			6 * 1 * (climb_rate_filter[2] - climb_rate_filter[6]) / (climb_rate_filter_time[3] - climb_rate_filter_time[5]);
		raw_climb_rate /= 32.0f;

		// low pass filter
		const float RC = 1.0f/(2*3.1415926 * 0.5f);
		float alpha = delta_time / (delta_time + RC);
		climb_rate = raw_climb_rate * alpha + (1-alpha) * climb_rate;

		TRACE("\r\npressure,temperature=%f, %f, ground pressure & temperature=%f, %f, height=%f, climb_rate=%f, time=%f\r\n", pressure, temperature, ground_pressure, ground_temperature, altitude, climb_rate, (double)getus()/1000000);

		last_baro_time = this_baro_time;


		// experimental accelerometer assisted altitude/climb fusion.
		a_raw_pressure = ms5611[0] / 100.0f;
		a_raw_temperature = ms5611[1] / 100.0f;
		climb2();

	}

	apm();

	return 0;
}

int calculate_attitude()
{

	float GYRO_SCALE = 2000.0 * PI / 180 / 32767 * interval;		// full scale: +/-2000 deg/s  +/-31767, 8ms interval

	vector gyro_zero2 = {0.681 * mpu6050_temperature - 28.075,
		0.1063 * mpu6050_temperature +3.1409,
		-0.3083 * mpu6050_temperature - 0.6421};

	if (mpu6050_temperature < 35)
		gyro_zero2.array[0] = 1.0313*mpu6050_temperature - 14.938;

	vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
	vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
	vector mag = {(p->mag[2]-mag_zero.array[2]), -(p->mag[0]-mag_zero.array[0]), -(p->mag[1]-mag_zero.array[1])};
	vector_sub(&gyro, &gyro_zero2);
	for(int i=0; i<3; i++)
		if (abs(gyro.array[i])<3)
			gyro.array[i] = 0;
	vector_multiply(&gyro, GYRO_SCALE);





	vector_rotate(&estGyro, gyro.array);
	vector_rotate(&estAccGyro, gyro.array);
	vector_rotate(&estMagGyro, gyro.array);

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
	if (acc_g > 0.90 && acc_g < 1.10)
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
	climb_rate = climb_rate * 0.05 + 0.95 * (climb_rate + accelz * interval);

	// test climb rate low pass filter
	{
		// 0.5hz low pass filter
		static const float RC = 1.0f/(2*3.1415926 * 0.5f);
		float alpha = interval / (interval + RC);
		climb_rate_lowpass = alpha * climb_rate + (1-alpha) * climb_rate_lowpass;
	}

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

	return 0;
}

void inline debugpin_init()
{
	// use PA-0 as cycle debugger
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 |  GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);
}

void inline led_all_on()
{
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
	GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}
void inline flashlight_off()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}


int magnet_calibration()
{
	short mag_min[3] = {9999,9999,9999};
	short mag_max[3] = {-9999, -9999, -9999};


	// load magnetemeter cneter
mag_load:
	for(int i=0; i<sizeof(mag_zero); i+=2)
		EE_ReadVariable(VirtAddVarTab[0]+i/2+EEPROM_MAG_ZERO, (uint16_t*)(((uint8_t*)&mag_zero.array)+i));
// 	for(int i=0; i<sizeof(mag_gain); i+=2)
// 		EE_ReadVariable(VirtAddVarTab[0]+i/2+EEPROM_MAG_GAIN, (uint16_t*)(((uint8_t*)&mag_gain.array)+i));

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

				int tx_result = log((u8*)&to_send, 32);
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
		for(int i=0; i<sizeof(mag_zero); i+=2)
			EE_WriteVariable(VirtAddVarTab[0]+i/2+EEPROM_MAG_ZERO, *(uint16_t*)(((uint8_t*)&mag_zero.array)+i));
// 		for(int i=0; i<sizeof(mag_gain); i+=2)
// 			EE_WriteVariable(VirtAddVarTab[0]+i/2+EEPROM_MAG_GAIN, *(uint16_t*)(((uint8_t*)&mag_gain.array)+i));

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
		ads1115_go_on();
		read_MPU6050(p->accel);
		read_HMC5883(p->mag);
		if (read_MS5611(ms5611) == 0)
		{
			baro_counter ++;
			ground_pressure += ms5611[0];
			ground_temperature += ms5611[1];
		}

		vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
		vector mag = {(p->mag[2]-mag_zero.array[2]), -(p->mag[0]-mag_zero.array[0]), -(p->mag[1]-mag_zero.array[1])};
		vector_add(&gyro_zero, &gyro);
		vector_add(&accel_avg, &acc);
		vector_add(&mag_avg, &mag);



#if PCB_VERSION == 3
		airspeed_voltage = ads1115_voltage*0.03 + airspeed_voltage * 0.97;
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
			g_ppm_output[i] = floor(g_ppm_input[i]+0.5);
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
	airspeed_voltage = 2.048 * airspeed_voltage / 32767;
#else
	airspeed_voltage = 2.5 * airspeed_voltage / adc_2_5_V;
#endif


	airspeed_bias = PCB_VERSION == 3 ? airspeed_voltage : ((airspeed_voltage - VCC_5V/2) *  5 / VCC_5V);
	has_airspeed = abs(airspeed_bias)<0.5f;

	TRACE("5V voltage = %.3fV\n", VCC_5V);
	TRACE("3.3V voltage = %.3fV\n", VCC_3_3V);
	TRACE("VCC motor = %.3fV\n\n", VCC_motor);
	TRACE("airspeed voltage = %.4fV, bias=%.4fV\n\n", airspeed_voltage, airspeed_bias);

	if (VCC_5V / VCC_motor >= 0.85 && VCC_5V / VCC_motor <= 1.15)
	{
		voltage_divider_factor *= VCC_5V / VCC_motor;

		TRACE("motor factor fix!\n");
		for(int i=0; i<sizeof(voltage_divider_factor); i+=2)
			EE_WriteVariable(VirtAddVarTab[0]+i/2+EEPROM_VOLTAGE_DIVIDER, ((uint16_t*)(((uint8_t*)&voltage_divider_factor)+i))[0]);
	}

	groundA = accel_avg;
	groundM = mag_avg;

	vector_divide(&gyro_zero, 300);
	vector_divide(&accel_avg, 300);
	vector_divide(&mag_avg, 300);
	ground_pressure /= baro_counter * 100;
	ground_temperature /= baro_counter * 100;
	pressure = ground_pressure;
	temperature = ground_temperature;

	estAccGyro = accel_avg;
	estGyro= estMagGyro = mag_avg;
	accel_1g = vector_length(&accel_avg);	


	TRACE("base value measured\r\n");
	return 0;
}

int check_mode()
{

	if (g_ppm_input_update[4] > getus() - RC_TIMEOUT)
	{
		if (g_ppm_input[4] < 1333)
#if QUADCOPTER == 1
			mode = shutdown;
#else
			mode = manual;
#endif
		else if (g_ppm_input[4] > 1666)
#if QUADCOPTER == 1
			mode = quadcopter;
#else
			mode = acrobatic;
#endif
		else
		{
			mode = rc_fail;
		}
	}
	else
	{
		TRACE("warning: RC out of controll");
		mode = rc_fail;	
	}

	// quadcopter startup protection
	// if not startup in shutdown mode, we flash the light and refuse to work
#if QUADCOPTER == 1
	if (last_mode == initializing && mode != shutdown)
	{
		while(true)
		{
			delayms(500);
			led_all_off();
			delayms(500);
			led_all_on();
		}
	}
#endif

	// mode changed?
	if (mode != last_mode)
	{
		last_mode = mode;

		target[0] = pos[0];
		target[1] = pos[1];
		target[2] = pos[2];

		target_altitude = _position - 3.5;
		target_climb_rate = -quadcopter_max_decend_rate;
		target_accel = -quadcopter_max_acceleration*2;
		accel_error_pid[0] = target_accel;
		accel_error_pid[1] = 0;
		accel_error_pid[2] = 0;
		yaw_launch = yaw_est;

#if QUADCOPTER == 1
		for(int i=0; i<3; i++)
		{
			angle_target[i] = angle_pos[i];
			error_pid[i][1] = 0;	//reset integration
		}
#endif

		airborne = false;
		takeoff_ground_altitude = altitude;

		targetVA = estAccGyro;
		targetVM = estMagGyro;

		vector_normalize(&targetVA);
		vector_normalize(&targetVM);

		for(int i=0; i<6; i++)
			rc_zero[i] = g_ppm_input[i];
	}


	return 0;
}

int osd()
{
	// artificial horizon
	//while((MAX7456_Read_Reg(STAT) & 0x10) != 0x00); // wait for vsync
	float roll_constrain = limit(roll, -30*PI/180, +30*PI/180);
	float pitch_constrain = limit(pitch, -30*PI/180, +30*PI/180);

	float tan_roll = tan(roll_constrain);
	float tan_pitch = tan(pitch_constrain);


	static int last_osd_pos[31] = {0};
	for(int x = 15-5; x<= 15+5; x++)
	{
		int y = floor(-(x - 15)*12*tan_roll +  18 * 16 * tan_pitch + 0.5) + 18*8;

		if (y<0 || y > 18*16)
			continue;

		MAX7456_Write_Char_XY(x,last_osd_pos[x-15], 0);
		last_osd_pos[x-15] = y/18;
		MAX7456_Write_Char_XY(x,y/18, y%18+1);
	}

	// climb rate & altitude
	sprintf(climb_rate_string, "%c%.1f", altitude >0 ? '+' : '-', fabs(altitude));
	MAX7456_PrintDigitString(climb_rate_string, 0, 8);
	sprintf(climb_rate_string, "%c%.1f", climb_rate_lowpass >0 ? '+' : '-', fabs(climb_rate_lowpass));
	MAX7456_PrintDigitString(climb_rate_string, 0, 9);

	return 0;
}

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

int main(void)
{
	// Basic Initialization
	SysClockInit();
	FLASH_Unlock();
	EE_Init();
	ADC1_Init();
	SysTick_Config(720);
	PPM_init(1);
	printf_init();
	I2C_init(0x30);
	init_timer();
	init_MPU6050();
	init_HMC5883();	
	init_MS5611();
	debugpin_init();
	GPS_Init(115200);
	sdcard_init();
	
	NRF_Init();
	MAX7456_SYS_Init();
	Max7456_Set_System(1);
	flashlight_on();

	
	#if PCB_VERSION == 3
	ads1115_init();	
	ads1115_new_work(ads1115_speed_16sps, ads1115_channnel_AIN0, ads1115_gain_4V, &ads1115_voltage);
	ads1115_new_work(ads1115_speed_16sps, ads1115_channnel_AIN1_AIN3, ads1115_gain_4V, &ads1115_current);
	ads1115_new_work(ads1115_speed_16sps, ads1115_channnel_AIN2_AIN3, ads1115_gain_2V, &ads1115_airspeed);
	ads1115_new_work(ads1115_speed_16sps, ads1115_channnel_AIN3, ads1115_gain_4V, &ads1115_2_5V);
	#endif
	
	delayms(100);
	
	NRF_Init();
	nrf_ok = 0 == NRF_Check();
	TRACE("NRF_Check() = %d\r\n", nrf_ok);
	if (nrf_ok)
		NRF_TX_Mode();
	
	p->voltage = -32768;
	p->current = -32768;

	// load voltage divider factor
	for(int i=0; i<sizeof(voltage_divider_factor); i+=2)
		EE_ReadVariable(VirtAddVarTab[0]+i/2+EEPROM_VOLTAGE_DIVIDER, (uint16_t*)(((uint8_t*)&voltage_divider_factor)+i));
	
	magnet_calibration();
	sensor_calibration();


	
	// check NRF again
	if (!nrf_ok)
		nrf_ok = 0 == NRF_Check();

	// open NRF TX Mode if found
	if (nrf_ok)
		NRF_TX_Mode();

	TRACE("NRF_Check() 2 = %d\r\n", nrf_ok);

	// the main loop
	while(1)
	{
		cycle_counter++;
		int64_t round_start_tick = getus();
		interval = (round_start_tick-last_tick)/1000000.0f;
		last_tick = round_start_tick;
		
		led_all_off();
		if (nrf_ok && cycle_counter % 4 == 0)
			NRF_RX_Mode();

		// flashlight
		time = getus();
		int time_mod_1500 = (time%1500000)/1000;
		if (time_mod_1500 < 150 || (time_mod_1500 > 200 && time_mod_1500 < 350))
			flashlight_on();
		else
			flashlight_off();
		
		// RC modes and RC fail detection
		check_mode();

		// read sensors and update altitude if new air pressure data arrived.
		read_sensors();

		// attitude and  heading
		calculate_attitude();

		// gps		
		if (GPS_ParseBuffer() > 0)
			last_gps_tick = getus();

		
		
		osd();
		calculate_position();
		calculate_target();
		pid();
		output();
		save_logs();
		
		
		
		TRACE("\rroll,pitch,yaw/yaw2 = %f,%f,%f,%f, target roll,pitch,yaw = %f,%f,%f, error = %f,%f,%f", roll*PI180, pitch*PI180, yaw_est*PI180, yaw_gyro*PI180, target[0]*PI180, target[1]*PI180, target[2]*PI180,
			error_pid[0][0]*PI180, error_pid[1][0]*PI180, error_pid[2][0]*PI180);
		
		TRACE("time=%.2f,inte=%.4f,out= %d, %d, %d, %d, input=%f,%f,%f,%f\n", getus()/1000000.0f, interval, g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_input[0], g_ppm_input[1], g_ppm_input[3], g_ppm_input[5]);
		TRACE (" mag=%.2f,%.2f,%.2f  acc=%.2f,%.2f,%.2f ", estMagGyro.V.x, estMagGyro.V.y, estMagGyro.V.z, estAccGyro.V.x, estAccGyro.V.y, estAccGyro.V.z);
		TRACE("input= %.2f, %.2f, %.2f, %.2f,%.2f,%.2f", g_ppm_input[0], g_ppm_input[1], g_ppm_input[2], g_ppm_input[3], g_ppm_input[4], g_ppm_input[5]);
		TRACE("input:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f, ADC=%.2f", g_ppm_input[0], g_ppm_input[1], g_ppm_input[3], g_ppm_input[5], g_ppm_input[4], g_ppm_input[5], p->voltage/1000.0 );
		
		TRACE("\ryaw=%.2f, yt=%.2f, at=%.2f,%.2f", yaw_est *PI180, yaw_launch*PI180, angle_target[0] * PI180, angle_target[1] * PI180);


		led_all_on();

		// read and process a packet
		rf_data packet;
		if (NRF_Rx_Dat((u8*)&packet) == RX_OK)
		{
			TRACE("packet!!!!\r\n\r\n");
		}

		// wait for next 8ms and send all data out
		NRF_TX_Mode();
		while(getus()-round_start_tick < cycle_time)
		{
			NRF_Handle_Queue();
		}
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
}


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
