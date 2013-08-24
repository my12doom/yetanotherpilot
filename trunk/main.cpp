#include <stdio.h>
#include <stm32f10x.h>
#include <math.h>

extern "C"
{
#include "common/printf.h"
#include "common/adc.h"
#include "common/I2C.h"
#include "common/NRF24L01.h"
#include "common/PPM.h"
#include "common/sdio_sdcard.h"
#include "common/common.h"
#include "common/vector.h"
#include "sensors/HMC5883.h"
#include "sensors/MPU6050.h"
#include "sensors/MS5611.h"
}

#define PI 3.14159265
#define interval (0.008)

#define RC_TIMEOUT 1000000				// 1 seconds

#define ACRO_ROLL_RATE (PI)				// 180 degree/s
#define ACRO_PITCH_RATE (PI/2)			// 90 degree/s
#define ACRO_YAW_RATE (PI/4)			// 45 degree/s
#define ACRO_MAX_ROLL_OFFSET (PI/6)		// 30 degree, max roll advance before airframe can response in acrobatic mode
#define ACRO_MAX_PITCH_OFFSET (PI/6)	// 30 degree, max pitch advance before airframe can response in acrobatic mode
#define ACRO_MAX_YAW_OFFSET (PI/6)		// 30 degree, max yaw advance before airframe can response in acrobatic mode
#define ACRO_MANUAL_FACTOR (0.3)		// final output in acrobatic mode, 70% pid, 30% rc

static float pid_factor[3][3] = 
{
	{0.2, 0, 0,},						// roll p, i, d
	{0.2, 0, 0,},						// pitch p, i, d
	{0.2, 0, 0,},						// yaw p, i, d
};


typedef struct
{
	short mag[3];	
	short accel[3];	
	short temperature1;	
	short gyro[3];		// roll, pitch, yaw	
	long temperature2;
	long pressure;	
} sensor_data;

enum fly_mode
{
	initializing,
	manual,
	acrobatic,
	fly_by_wire,
};

int abs(int i)
{
	if (i>0)
		return i;
	return -i;
}

int max(int a, int b)
{
	if (a>b)
		return a;
	return b;
}


float fmax(float a, float b)
{
	if (a>b)
		return a;
	return b;
}
int min(int a, int b)
{
	if (a<b)
		return a;
	return b;
}

// a & b : -PI ~ PI
// return a - b
float radian_delta(float a, float b)
{
	if (a>b)
		return -radian_delta(b,a);
	
	float d1 = a-b;
	float d2 = -(a+2*PI-b);

	return abs(d1) > abs(d2) ? d2 : d1;
}


int main(void)
{
	// Basic Initialization
	SysTick_Config(720);
	printf_init();
	SPI_NRF_Init();
	printf("NRF_Check() = %d\r\n", NRF_Check());
	PPM_init(1);
	I2C_init(0x30);
	init_timer();
	init_MPU6050();
	init_HMC5883();	
	init_MS5611();
	
	// use PA-04 as cycle debugger
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	int mode = initializing;
	sensor_data *p = new sensor_data;
	vector estAccGyro = {0};			// for roll & pitch
	vector estMagGyro = {0};			// for yaw
	vector estGyro = {0};				// for gyro only yaw, yaw lock on this
	
	vector mag_avg = {0};
	vector gyro_zero = {0};
	vector accel_avg = {0};
	double ground_pressure = 0;
	double ground_temperature = 0;
	int baro_counter = 0;

	// static base value detection
	for(int i=0; i<1000; i++)
	{
		printf("\r%d/1000", i);
		if (read_MPU6050(p->accel)<0 && read_MPU6050(p->accel)<0)
		{
			printf("warning, MPU6050 sensor error during initializing\r\n");
			continue;
		}
		if (read_HMC5883(p->mag)<0 && read_HMC5883(p->mag)<0)
		{
			printf("warning, HMC5883 sensor error during initializing\r\n");
			continue;
		}

		int baro[2];
		if (read_MS5611(baro) == 0)
		{
			ground_pressure += baro[0];
			ground_temperature += baro[1];
			baro_counter ++;
		}
		
		vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
		vector mag = {p->mag[1], -p->mag[0], -p->mag[2]};
		vector_add(&gyro_zero, &gyro);
		vector_add(&accel_avg, &acc);
		vector_add(&mag_avg, &mag);

		delayms(1);
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
	
	while(0)
	{
		double temperature = 0;
		double pressure = 0;
		int OSS = 50;
		for(int i=0; i<OSS; i++)
		{
			int data[2];
			while (read_MS5611(data) != 0) 
				;

			pressure += data[0];
			temperature += data[1];			
			printf("\r%d/%d", i, OSS);
		}

		pressure /= OSS * 100;
		temperature /= OSS * 100;

		double scaling = (double)pressure / ground_pressure;
		double temp = ((double)ground_temperature) + 273.15f;
		double altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));
		printf("pressure,temperature=%f, %f, ground pressure & temperature=%f, %f, height=%f, time=%f\r\n", pressure, temperature, ground_pressure, ground_temperature, altitude, (double)getus()/1000000);
	}
	

	// the main loop

	int last_mode = mode;

	while(1)
	{
		static const float factor = 0.995;
		static const float factor_1 = 1-factor;
		int start_tick = getus();
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);

		// if rc works and is switched to bypass mode, pass the PPM inputs directly to outputs
		if (g_ppm_input_update[3] > getus() - RC_TIMEOUT )
		{
			if (g_ppm_input[3] < 1520)
				mode = manual;
			else
				mode = acrobatic;
		}
		else
		{
			//printf("warning: RC out of controll\r\n");
			mode = acrobatic;
			g_ppm_input[0] = g_ppm_input[1] = g_ppm_input[2] = 1500;			
		}


		// always read sensors and calculate attitude
		if (read_MPU6050(&p->accel[0])<0 && read_MPU6050(&p->accel[0])<0)
		{
			printf("read_MPU6050 error!\r\n");
			continue;
		}
		
		if (read_HMC5883(&p->mag[0])<0 && read_HMC5883(&p->mag[0])<0)
		{
			printf("read_HMC5883 error!\r\n");
			continue;
		}
		
		static const float GYRO_SCALE = 2000.0 * PI / 180 / 8192 * interval / 4 / 10;		// full scale: +/-2000 deg/s  +/-8192, 8ms interval, divided into 10 piece to better use small angle approximation
		vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
		vector mag = {p->mag[1], -p->mag[0], -p->mag[2]};
		vector_sub(&gyro, &gyro_zero);
		vector_multiply(&gyro, GYRO_SCALE);
		

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
		float roll = atan2(estAccGyro.V.x, estAccGyro.V.z);
		float pitch = atan2(estAccGyro.V.y, sqrt(estAccGyro.V.x*estAccGyro.V.x + estAccGyro.V.z * estAccGyro.V.z));
		vector estAccGyro16 = estAccGyro;
		vector_divide(&estAccGyro16, 16);
		float xxzz = (estAccGyro16.V.x*estAccGyro16.V.x + estAccGyro16.V.z * estAccGyro16.V.z);
		float G = sqrt(xxzz+estAccGyro16.V.y*estAccGyro16.V.y);
		float yaw_est = atan2(estMagGyro.V.z * estAccGyro16.V.x - estMagGyro.V.x * estAccGyro16.V.z,
			(estMagGyro.V.y * xxzz - (estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);
		float yaw_gyro = atan2(estGyro.V.z * estAccGyro16.V.x - estGyro.V.x * estAccGyro16.V.z,
			(estGyro.V.y * xxzz - (estGyro.V.x * estAccGyro16.V.x + estGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);


		// mode changed?
		if (mode != last_mode)
		{
			last_mode = mode;

			target[0] = roll;
			target[1] = pitch;
			target[2] = yaw_gyro;
		}
		
		// apply pid controll
		float error_pid[3][3] = {0};		// error_pid[roll, pitch, yaw][p,i,d]
		error_pid[0][0] = radian_delta(roll, target[0]);
		error_pid[1][0] = radian_delta(pitch, target[1]);
		error_pid[2][0] = radian_delta(yaw_gyro, target[2]);
		

		g_ppm_output[0] = 1520 + (1-ACRO_MANUAL_FACTOR)*(error_pid[0][0] * pid_factor[0][0] + error_pid[0][1] * pid_factor[0][1] + error_pid[0][2] * pid_factor[0][2]);
		g_ppm_output[1] = 1520 + (1-ACRO_MANUAL_FACTOR)*(error_pid[0][0] * pid_factor[0][0] + error_pid[0][1] * pid_factor[0][1] + error_pid[0][2] * pid_factor[0][2]);
		g_ppm_output[2] = 1520 + (1-ACRO_MANUAL_FACTOR)*(error_pid[0][0] * pid_factor[0][0] + error_pid[0][1] * pid_factor[0][1] + error_pid[0][2] * pid_factor[0][2]);

		for(int i=0; i<3; i++)
		{
			g_ppm_output[i] += g_ppm_input[i] * ACRO_MANUAL_FACTOR;
			g_ppm_output[i] = max(1000, g_ppm_output[i]);
			g_ppm_output[i] = min(2000, g_ppm_output[i]);
		}
		

		PPM_update_output_channel(PPM_OUTPUT_CHANNEL0 | PPM_OUTPUT_CHANNEL1 | PPM_OUTPUT_CHANNEL2);
		
		float PI180 = 180/PI;
		
		
		printf("\r roll,pitch,yaw/yaw2 = %f,%f,%f,%f, target roll,pitch,yaw = %f,%f,%f, error = %f,%f,%f", roll*PI180, pitch*PI180, yaw_est*PI180, yaw_gyro*PI180, target[0]*PI180, target[1]*PI180, target[2]*PI180,
			error_pid[0][0]*PI180, error_pid[1][0]*PI180, error_pid[2][0]*PI180);

		// calculate new target & targetM ( the lock target)
		switch (mode)
		{
		case acrobatic:
			{
				float rc[3] = {(float)(g_ppm_input[0] - 1520) / 480 * ACRO_ROLL_RATE * interval, 
											(float)(g_ppm_input[1] - 1520) / 480 * ACRO_PITCH_RATE * interval,
											(float)(g_ppm_input[2] - 1520) / 480 * ACRO_YAW_RATE * interval};

				for(int i=0; i<3; i++)
				{
					if (abs(error_pid[i][0] + rc[i]) > ACRO_MAX_ROLL_OFFSET)
						rc[i] = 0;
					target[i] += rc[i];
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
