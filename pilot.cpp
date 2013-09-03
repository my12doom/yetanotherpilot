#include <stdio.h>
#include <stm32f10x.h>
#include <math.h>
#include "RFData.h"

extern "C"
{
#include "common/printf.h"
#include "common/adc.h"
#include "common/I2C.h"
#include "common/NRF24L01.h"
#include "common/PPM.h"
#include "common/common.h"
#include "common/vector.h"
#include "sensors/HMC5883.h"
#include "sensors/MPU6050.h"
#include "sensors/MS5611.h"
}

#define PI 3.14159265
#define interval (0.008)

#define RC_TIMEOUT 1000000				// 1 seconds
#define RC_RANGE 400
#define RC_DEAD_ZONE 5
#define BARO_OSS 50

#define ACRO_ROLL_RATE (PI*3/2)				// 270 degree/s
#define ACRO_PITCH_RATE (PI)			// 180 degree/s
#define ACRO_YAW_RATE (PI/2)			// 90 degree/s
#define ACRO_MANUAL_FACTOR (0.3)		// final output in acrobatic mode, 70% pid, 30% rc


static float pid_factor[3][3] = 			// pid_factor[roll,pitch,yaw][p,i,d]
{
	{1, 0, 0,},
	{1, 0, 0,},
	{1, 0, 0,},
};

static float pid_limit[3][3] = 				// pid_limit[roll,pitch,yaw][p max offset, I limit, d dummy]
{
	{PI/6, PI*6, 1},
	{PI/6, PI*6, 1},
	{PI/6, PI*6, 1},
};

#define ACRO_MAX_ROLL_OFFSET (pid_limit[0][0])		// 30 degree, max roll advance before airframe can response in acrobatic mode
#define ACRO_MAX_PITCH_OFFSET (pid_limit[1][0])	// 30 degree, max pitch advance before airframe can response in acrobatic mode
#define ACRO_MAX_YAW_OFFSET (pid_limit[2][0])		// 30 degree, max yaw advance before airframe can response in acrobatic mode

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

enum fly_mode
{
	initializing,
	manual,
	acrobatic,
	fly_by_wire,
	rc_fail,
};

float limit(float v, float low, float high)
{
	if (v < low)
		return low;
	if (v > high)
		return high;
	return v;
}

float radian_add(float a, float b)
{
	a += b;
	if (a>=PI)
		a -= 2*PI;
	if (a<-PI)
		a += 2*PI;
	
	return a;
}

// a & b : -PI ~ PI
// return a - b
float radian_sub(float a, float b)
{
	float v1 = a-b;
	float v2 = a+2*PI-b;
	float v3 = a-2*PI-b;
	
	v1 = abs(v1)>abs(v2) ? v2 : v1;
	return abs(v1)>abs(v3) ? v3 : v1;
}

int main(void)
{
	// Basic Initialization
	SysTick_Config(720);
	PPM_init(1);
	printf_init();
	SPI_NRF_Init();
	int nrf = NRF_Check();
	printf("NRF_Check() = %d\r\n", nrf);
	if (nrf == 0)
		NRF_TX_Mode();
	I2C_init(0x30);
	init_timer();
	init_MPU6050();
	init_HMC5883();	
	init_MS5611();
	
	
	while(0)
	{
		int us = getus();
		u8 data[32];
		NRF_Tx_Dat(data);
		
		while(getus()-us < 8000)
			;
	}
	
	// use PA-04 as cycle debugger
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	int mode = initializing;
	u8 data[TX_PLOAD_WIDTH];
	sensor_data *p = (sensor_data*)data;
	sensor_data old;
	vector estAccGyro = {0};			// for roll & pitch
	vector estMagGyro = {0};			// for yaw
	vector estGyro = {0};				// for gyro only yaw, yaw lock on this
	
	vector mag_avg = {0};
	vector gyro_zero = {0};
	vector accel_avg = {0};
	double ground_pressure = 0;
	double ground_temperature = 0;
	double altitude = -999;
	int baro_counter = 0;

	// static base value detection
	for(int i=0; i<1000; i++)
	{
		printf("\r%d/1000", i);
		if (read_MPU6050(p->accel)<0 && read_MPU6050(p->accel)<0)
		{
			printf("warning, MPU6050 sensor error during initializing\r\n");
			i--;
			continue;
		}
		if (read_HMC5883(p->mag)<0 && read_HMC5883(p->mag)<0)
		{
			printf("warning, HMC5883 sensor error during initializing\r\n");
			i--;
			continue;
		}

		int baro[2];
		read_MS5611(baro);
		
		vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
		vector mag = {p->mag[1], -p->mag[0], -p->mag[2]};
		vector_add(&gyro_zero, &gyro);
		vector_add(&accel_avg, &acc);
		vector_add(&mag_avg, &mag);

		delayms(2);
	}
	
	vector_divide(&gyro_zero, 1000);
	vector_divide(&accel_avg, 1000);
	vector_divide(&mag_avg, 1000);
	ground_pressure /= baro_counter * 100;
	ground_temperature /= baro_counter * 100;

	estAccGyro = accel_avg;
	estGyro= estMagGyro = mag_avg;
	float accel_1g = vector_length(&accel_avg);	
	
	
	printf("base value measured\r\n");

	mode = manual;
	float target[3];		// target[roll, pitch, yaw]
		
	double temperature = 0;
	double pressure = 0;
	int oss = BARO_OSS;
	

	// the main loop

	int last_mode = mode;
	float rc_zero[] = {1520, 1520, 1520};
	float error_pid[3][3] = {0};		// error_pid[roll, pitch, yaw][p,i,d]

	while(1)
	{
		static const float factor = 0.995;
		static const float factor_1 = 1-factor;
		int start_tick = getus();
		bool rc_works = false;
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		
		// if rc works and is switched to bypass mode, pass the PPM inputs directly to outputs
		if (g_ppm_input_update[3] > getus() - RC_TIMEOUT)
		{
			if (g_ppm_input[3] > 1666)
				mode = manual;
			else if (g_ppm_input[3] < 1666)
				mode = acrobatic;
			else
				mode = rc_fail;
			rc_works = true;
		}
		else
		{
			//printf("warning: RC out of controll\r\n");
			mode = acrobatic;
			g_ppm_input[0] = g_ppm_input[1] = g_ppm_input[2] = 1500;			
		}
		
		

		// always read sensors and calculate attitude
		int I2C_time = getus();
		int I2C_retry = 5;
		while (read_MPU6050(&p->accel[0])<0 && getus()- I2C_time < 5000)
		{
			I2C_retry--;
			printf("read_MPU6050 error!\r\n");
		}
		
		while (read_HMC5883(&p->mag[0])<0 && getus()- I2C_time < 5000)
		{
			I2C_retry--;
			printf("read_HMC5883 error!\r\n");
		}
		
		// MS5611 never return invalid value even on error, so no retry
		int ms5611[2];
		int ms5611result = read_MS5611(ms5611);

		// calculate altitude
		if (ms5611result == 0)
		{
			pressure += ms5611[0];
			temperature += ms5611[1];
			oss --;

			if (oss == 0)
			{
				oss = BARO_OSS;
				pressure /= BARO_OSS*100;
				temperature /= BARO_OSS*100;
				
				if (ground_pressure >0 && ground_pressure > 0)
				{
					double scaling = (double)pressure / ground_pressure;
					double temp = ((double)ground_temperature) + 273.15f;
					altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));
					printf("\r\npressure,temperature=%f, %f, ground pressure & temperature=%f, %f, height=%f, time=%f\r\n", pressure, temperature, ground_pressure, ground_temperature, altitude, (double)getus()/1000000);
				}
				else
				{
					ground_pressure = pressure;
					ground_temperature = temperature;
				}

				pressure = 0;
				temperature = 0;				
			}
		}
		
		if (I2C_retry == 0)
		{
			*p = old;
		}
		else
		{
			old = *p;
		}
		
		
		// send raw sensor data and IMU data back if NRF exists
		if (nrf == 0)
		{
			int64_t time = getus();
			
			rf_data to_send;
			to_send.time = (time & (~TAG_MASK)) | TAG_SENSOR_DATA;
			to_send.data.sensor = *p;
			
			int tx_result;
			tx_result = NRF_Tx_Dat((u8*)&to_send);
			
			static int t1 = 0, t2=0;
			if (tx_result == TX_OK)
				t1++;
			
			imu_data imu = 
			{
				{estAccGyro.array[0], estAccGyro.array[1], estAccGyro.array[2]},
				{estGyro.array[0], estGyro.array[1], estGyro.array[2]},
				{estMagGyro.array[0], estMagGyro.array[1], estMagGyro.array[2]},
				ms5611[0],
				ms5611[1],
			};
			
			to_send.time = (time & (~TAG_MASK)) | TAG_IMU_DATA;
			to_send.data.imu = imu;
			
			tx_result = NRF_Tx_Dat((u8*)&to_send);
			if (tx_result == TX_OK)
				t2++;
			
			extern int tx_ok;
			extern int max_retry;
			//printf("\r t1,t2,ok,timeout=%d,%d,%d,%d", t1, t2, tx_ok, max_retry);
			
			pilot_data pilot = 
			{
				mode,
				{error_pid[0][0]*180*100/PI, error_pid[1][0]*180*100/PI, error_pid[2][0]*180*100/PI},
				{target[0]*180*100/PI, target[1]*180*100/PI, target[2]*180*100/PI},
				altitude * 100,
				{g_ppm_input[0], g_ppm_input[1], g_ppm_input[2]}
			};

			to_send.time = (time & (~TAG_MASK)) | TAG_PILOT_DATA;
			to_send.data.pilot = pilot;
			tx_result = NRF_Tx_Dat((u8*)&to_send);
		}
		
		static const float GYRO_SCALE = 2000.0 * PI / 180 / 8192 * interval / 4 / 10;		// full scale: +/-2000 deg/s  +/-8192, 8ms interval, divided into 10 piece to better use small angle approximation
		
		
		vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
		vector mag = {p->mag[1], -p->mag[0], -p->mag[2]};
		vector_sub(&gyro, &gyro_zero);
		vector_multiply(&gyro, GYRO_SCALE);
		
		//printf("gyro:%d,%d,%d\r\n", p->gyro[0], p->gyro[1], p->gyro[2]);
		

		for(int j=0; j<10; j++)
		{
			vector_rotate(&estGyro, gyro.array);
			vector_rotate(&estAccGyro, gyro.array);
			vector_rotate(&estMagGyro, gyro.array);
		}
		
		// apply CF filter for Mag
		vector mag_f = mag;
		vector_multiply(&mag_f, factor_1);
		vector_multiply(&estMagGyro, factor);
		vector_add(&estMagGyro, &mag_f);
		
		// apply CF filter for Acc if g force is acceptable
		float acc_g = vector_length(&acc)/ accel_1g;
		if (acc_g > 0.85 && acc_g < 1.15)
		{
			vector acc_f = acc;
			vector_multiply(&acc_f, factor_1);
			vector_multiply(&estAccGyro, factor);
			vector_add(&estAccGyro, &acc_f);
		}

		// calculate attitude, unit is radian, range +/-PI
		float roll = radian_add(atan2(estAccGyro.V.x, estAccGyro.V.z), PI);
		float pitch = atan2(estAccGyro.V.y, (estAccGyro.V.z > 0 ? 1 : -1) * sqrt(estAccGyro.V.x*estAccGyro.V.x + estAccGyro.V.z * estAccGyro.V.z));
		pitch = radian_add(pitch, PI);
		vector estAccGyro16 = estAccGyro;
		vector_divide(&estAccGyro16, 16);
		float xxzz = (estAccGyro16.V.x*estAccGyro16.V.x + estAccGyro16.V.z * estAccGyro16.V.z);
		float G = sqrt(xxzz+estAccGyro16.V.y*estAccGyro16.V.y);
		float yaw_est = atan2(estMagGyro.V.z * estAccGyro16.V.x - estMagGyro.V.x * estAccGyro16.V.z,
			(estMagGyro.V.y * xxzz - (estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);
		float yaw_gyro = atan2(estGyro.V.z * estAccGyro16.V.x - estGyro.V.x * estAccGyro16.V.z,
			(estGyro.V.y * xxzz - (estGyro.V.x * estAccGyro16.V.x + estGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);

		float pos[3] = {roll, pitch, yaw_gyro};

		// mode changed?
		if (mode != last_mode)
		{
			last_mode = mode;

			target[0] = roll;
			target[1] = pitch;
			target[2] = yaw_gyro;
			
			rc_zero[0] = g_ppm_input[0];
			rc_zero[1] = g_ppm_input[1];
			rc_zero[2] = g_ppm_input[2];
		}
		
		if (!rc_works)
		{
			target[0] = -PI;
			target[1] = 0;
			target[2] = yaw_gyro;			
		}
		
		// calculate new pid & apply pid controll & output
		for(int i=0; i<3; i++)
		{
			float new_p = radian_sub(pos[i], target[i]) * sensor_reverse[i];
			error_pid[i][1] += new_p;																	// I
			error_pid[i][1] = limit(error_pid[i][1], -pid_limit[i][1], pid_limit[i][1]);
			error_pid[i][2] = new_p - error_pid[i][2];								// D
			error_pid[i][0] = new_p;																	// P
			
			float fly_controll = 0;
			for(int j=0; j<3; j++)
				fly_controll += limit(error_pid[i][j]/ pid_limit[i][j], -1, 1) * pid_factor[i][j];
			fly_controll *= (1-ACRO_MANUAL_FACTOR)*RC_RANGE;
			int rc = rc_reverse[i]*(g_ppm_input[i] - rc_zero[i]);
			
			//if (rc * fly_controll> 0)
				g_ppm_output[i] = 1520 + fly_controll + rc * ACRO_MANUAL_FACTOR;
			//else
			//	g_ppm_output[i] = 1520 + fly_controll;
			
			
			g_ppm_output[i] = limit(g_ppm_output[i], 1000, 2000);			
		}

		if (mode == manual)
		{
			for(int i=0; i<3; i++)
			{
				g_ppm_output[i] = g_ppm_input[i];
			}
		}

		PPM_update_output_channel(PPM_OUTPUT_CHANNEL0 | PPM_OUTPUT_CHANNEL1 | PPM_OUTPUT_CHANNEL2);
		
		float PI180 = 180/PI;
		
		
		printf("\rroll,pitch,yaw/yaw2 = %f,%f,%f,%f, target roll,pitch,yaw = %f,%f,%f, error = %f,%f,%f", roll*PI180, pitch*PI180, yaw_est*PI180, yaw_gyro*PI180, target[0]*PI180, target[1]*PI180, target[2]*PI180,
			error_pid[0][0]*PI180, error_pid[1][0]*PI180, error_pid[2][0]*PI180);
		
		printf(",out= %d, %d, %d, %d, input=%f,%f,%f,%f", g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_input[0], g_ppm_input[1], g_ppm_input[2], g_ppm_input[3]);
		
		// calculate new target
		switch (mode)
		{
		case acrobatic:
			{
				static const float rate[3] = {ACRO_ROLL_RATE * interval / RC_RANGE, 
														ACRO_PITCH_RATE * interval / RC_RANGE,
														ACRO_YAW_RATE * interval / RC_RANGE};

				for(int i=0; i<3; i++)
				{
					float rc = g_ppm_input[i] - rc_zero[i];
					if (abs(rc) < RC_DEAD_ZONE)
						rc = 0;
					else
						rc *= rate[i];
					
					float new_target = radian_add(target[i], -rc * rc_reverse[i] * sensor_reverse[i]);
					float new_error = abs(radian_sub(pos[i], new_target));
					if (new_error > pid_limit[i][0] && new_error > abs(error_pid[i][0]))
						;
					else
						target[i] = new_target;
				}
			}
			break;
		}


		GPIO_SetBits(GPIOA, GPIO_Pin_4);

		// wait for next 8ms
		while(getus()-start_tick < 8000)
			;		
	}
}
