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
#define RC_TIMEOUT 1000000		// 1 seconds

typedef struct
{
	short mag[3];	
	short accel[3];	
	short temperature1;	
	short gyro[3];		// roll, pitch, yaw	
	long temperature2;
	long pressure;	
} sensor_data;



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


	sensor_data *p = new sensor_data;
	vector estAccGyro = {0};			// for roll & pitch
	vector estMagGyro = {0};			// for yaw
	vector estGyro = {0};				// for gyro only yaw, yaw lock on this
	
	vector mag_avg = {0};
	vector gyro_base = {0};
	vector accel_avg = {0};

	// static base value detection
	for(int i=0; i<1000; i++)
	{
		printf("\r%d/1000", i);
		if (read_MPU6050(p->accel)<0)
			if (read_MPU6050(p->accel)<0)
				continue;
		if (read_HMC5883(p->mag)<0)
			if (read_HMC5883(p->mag)<0)
				continue;
		
		vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
		vector mag = {p->mag[1], -p->mag[0], -p->mag[2]};
		vector_add(&gyro_base, &gyro);
		vector_add(&accel_avg, &acc);
		vector_add(&mag_avg, &mag);
		delayms(1);
	}
	
	vector_divide(&gyro_base, 1000);
	vector_divide(&accel_avg, 1000);
	vector_divide(&mag_avg, 1000);

	
	vector target = {0};
	vector targetM = {0};
	target = estAccGyro = accel_avg;
	targetM = estGyro= estMagGyro = mag_avg;
	float accel_1g = vector_length(&accel_avg);


	
	printf("base value measured\r\n");
	

	// the main loop
	while(1)
	{
		static const float factor = 0.995;
		static const float factor_1 = 1-factor;
		int start_tick = getus();
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);


		// if rc works and is switched to bypass mode, pass the PPM inputs directly to outputs
		if (g_ppm_input_update[3] > getus() - RC_TIMEOUT && g_ppm_input[3] < 1520)
		{
			for(int i=0; i<3; i++)
				g_ppm_output[i] = g_ppm_input[i];
			PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);
		}


		
		if (read_MPU6050(&p->accel[0])<0)
			if (read_MPU6050(&p->accel[0])<0)
			{
				printf("read_MPU6050 error!\r\n");
				continue;
			}
		
		if (read_HMC5883(&p->mag[0])<0)
			if (read_HMC5883(&p->mag[0])<0)
			{
				printf("read_HMC5883 error!\r\n");
				continue;
			}
		
		static float max_gyro = 0;

		static const float GYRO_SCALE = 2000.0 * PI / 180 / 8192 * 0.0002;		// full scale: +/-2000 deg/s  +/-8192, 8ms interval, divided into 10 piece to better use small angle approximation
		vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
		vector mag = {p->mag[1], -p->mag[0], -p->mag[2]};
		vector_sub(&gyro, &gyro_base);
		vector_multiply(&gyro, GYRO_SCALE);
		max_gyro = fmax(gyro.array[0], max_gyro);
		
		float acc_g = vector_length(&acc)/ accel_1g;

		for(int j=0; j<10; j++)
		{
			vector_rotate(&estGyro, gyro.array);
			vector_rotate(&estAccGyro, gyro.array);
			vector_rotate(&estMagGyro, gyro.array);
		}
		
		// apply CF filter if g force is acceptable
		if (acc_g > 0.85 && acc_g < 1.15)
		{
			vector acc_f = acc;
			vector_multiply(&acc_f, factor_1);
			vector_multiply(&estAccGyro, factor);
			vector_add(&estAccGyro, &acc_f);
		}

		// apply CF filter for Mag
		vector mag_f = mag;
		vector_multiply(&mag_f, factor_1);
		vector_multiply(&estMagGyro, factor);
		vector_add(&estMagGyro, &mag_f);
		
		
		
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		static int i = 0;
		if(i++ %10== 0)
		{
			float roll = atan2(estAccGyro.V.x, estAccGyro.V.z) * 180 / PI;
			float pitch = atan2(estAccGyro.V.y, sqrt(estAccGyro.V.x*estAccGyro.V.x + estAccGyro.V.z * estAccGyro.V.z)) * 180 / PI;
			
			vector estAccGyro16 = estAccGyro;
			vector_divide(&estAccGyro16, 16);
			float xxzz = (estAccGyro16.V.x*estAccGyro16.V.x + estAccGyro16.V.z * estAccGyro16.V.z);
			float G = sqrt(xxzz+estAccGyro16.V.y*estAccGyro16.V.y);
			float yaw_est = atan2(estMagGyro.V.z * estAccGyro16.V.x - estMagGyro.V.x * estAccGyro16.V.z,
				(estMagGyro.V.y * xxzz - (estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G) * 180 / PI;
			float yaw_gyro = atan2(estGyro.V.z * estAccGyro16.V.x - estGyro.V.x * estAccGyro16.V.z,
				(estGyro.V.y * xxzz - (estGyro.V.x * estAccGyro16.V.x + estGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G) * 180 / PI;
			
			printf("\r mag:%f,%f,%f, gyro_est:%f,%f,%f, yaw_est=%f", mag.V.x, mag.V.y, mag.V.z, estMagGyro.V.x, estMagGyro.V.y, estMagGyro.V.z, yaw_est);
			//printf("\r gyro:%f, %f, %f, acc:%f, %f, %f,  max_gyro=%f, roll, pitch,yaw = %f,%f,%f         ", estAccGyro.array[0], estAccGyro.array[1], estAccGyro.array[2], acc.array[0], acc.array[1], acc.array[2], max_gyro, roll, pitch, yaw_est);
			//printf("xyz=%f,%f,%f, dx,dy,dz = %d,%d,%d, angel(pitch,roll,yaw, pitch_accel)=%f,%f,%f,%f\r\n", pitch, roll, yaw, p->gyro[0], p->gyro[1], p->gyro[2], pitch*9/17500, roll*9/17500 , yaw*9/17500, pitch_accel);
			//printf("accel xyz = %d, %d, %d, pitch_accel = %f\r\n", p->accel[0], p->accel[1], p->accel[2], pitch_accel);
			//printf("pitch, pitchI, pitch_accel, roll, rollI, roll_accel=%f,%f,%f,%f,%f,%f \r\n", pitch*9/17500, pitchI*9/17500, pitch_accel, roll*9/17500, rollI*9/17500, roll_accel);
			
			//printf("\rdelta roll,pitch=%f, %f", (roll - roll_target)*9/17500, (pitch - pitch_target)*9/17500);			
		}
		
		//g_ppm_output[0] = 1500 - (roll - roll_target)*9/17500 * 25;
		//g_ppm_output[1] = 1500 + (pitch - pitch_target)*9/17500 * 12;
		//g_ppm_output[2] = 1500 + (yaw - yaw_target)*9/17500 * 12;
		
		for(int i=0; i<3; i++)
		{
			g_ppm_output[i] = max(1000, g_ppm_output[i]);
			g_ppm_output[i] = min(2000, g_ppm_output[i]);
		}
		
		PPM_update_output_channel(PPM_OUTPUT_CHANNEL0 | PPM_OUTPUT_CHANNEL1 | PPM_OUTPUT_CHANNEL2);
					
		
		while(getus()-start_tick < 8000)
			;
	}
}
