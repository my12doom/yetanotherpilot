#include <stdio.h>
#include <stm32f10x.h>
#include <math.h>
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

#include "nmea/nmea.h"

int strlen(const char*p)
{
	int o = 0;
	while (*p++)
		o++;
	return o;
}

int abs(int x)
{
	return x>0 ? x : -x;
}

int main(void)
{
	// NMEA test
		const char *buff[] = {
			"$GPRMC,173843,A,3349.896,N,11808.521,W,000.0,360.0,230108,013.4,E*69\r\n",
			"$GPGGA,111609.14,5001.27,N,3613.06,E,3,08,0.0,10.2,M,0.0,M,0.0,0000*70\r\n",
			"$GPGSV,2,1,08,01,05,005,80,02,05,050,80,03,05,095,80,04,05,140,80*7f\r\n",
			"$GPGSV,2,2,08,05,05,185,80,06,05,230,80,07,05,275,80,08,05,320,80*71\r\n",
			"$GPGSA,A,3,01,02,03,04,05,06,07,08,00,00,00,00,0.0,0.0,0.0*3a\r\n",
			"$GPRMC,111609.14,A,5001.27,N,3613.06,E,11.2,0.0,261206,0.0,E*50\r\n",
			"$GPVTG,217.5,T,208.8,M,000.00,N,000.01,K*4C\r\n"
	};

	int it;
	nmeaINFO info;
	nmeaPARSER parser;

	nmea_zero_INFO(&info);
	nmea_parser_init(&parser);

	for(it = 0; it < 6; ++it)
			nmea_parse(&parser, buff[it], (int)strlen(buff[it]), &info);

	nmea_parser_destroy(&parser);

	// Basic Initialization
	ADC1_Init();
	SysTick_Config(720);
	PPM_init(1);
	printf_init();
	NRF_Init();
	int nrf = NRF_Check();
	TRACE("NRF_Check() = %d\r\n", nrf);
	if (nrf == 0)
		NRF_TX_Mode();
	I2C_init(0x30);
	init_timer();
	init_MPU6050();
	init_HMC5883();	
	init_MS5611();
	
		
	// use PA-11 as cycle debugger
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_11);


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
	p->voltage = -1;

	// static base value detection
	for(int i=0; i<1000; i++)
	{
		TRACE("\r%d/1000", i);
		if (read_MPU6050(p->accel)<0 && read_MPU6050(p->accel)<0)
		{
			TRACE("warning, MPU6050 sensor error during initializing\r\n");
			i--;
			continue;
		}
		if (read_HMC5883(p->mag)<0 && read_HMC5883(p->mag)<0)
		{
			TRACE("warning, HMC5883 sensor error during initializing\r\n");
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

		// RC pass through
		for(int i=0; i<6; i++)
		#if QUADCOPTER == 1
			g_ppm_output[i] = 1125;
		#else
			g_ppm_output[i] = floor(g_ppm_input[i]+0.5);
		#endif

		PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);

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
	
	
	TRACE("base value measured\r\n");

#if QUADCOPTER == 1
	mode = initializing;
#else
	mode = manual;
#endif
	float target[3];		// target[roll, pitch, yaw]
	float target_quad_base[3];		// quad base, usually for ground horizontal calibration
	vector gyroI;	// attitude by gyro only
		
	double temperature = 0;
	double pressure = 0;
	int oss = BARO_OSS;
	

	// the main loop

	int last_mode = mode;
	float rc_zero[] = {1520, 1520, 1520, 1520, 1520, 1520};
	float error_pid[3][3] = {0};		// error_pid[roll, pitch, yaw][p,i,d]

	while(1)
	{
		static const float factor = 0.997;
		static const float factor_1 = 1-factor;
		int start_tick = getus();
		bool rc_works = true;
		
		GPIO_ResetBits(GPIOA, GPIO_Pin_11);
		
		// if rc works and is switched to bypass mode, pass the PPM inputs directly to outputs
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
				rc_works = false;
			}
		}
		else
		{
			//TRACE("warning: RC out of controll\r\n");
			rc_works = false;
			mode = rc_fail;	
		}
		
		

		// always read sensors and calculate attitude
		int I2C_time = getus();
		int I2C_retry = 5;
		while (read_MPU6050(&p->accel[0])<0 && getus()- I2C_time < 5000)
		{
			I2C_retry--;
			TRACE("read_MPU6050 error!\r\n");
		}
		
		while (read_HMC5883(&p->mag[0])<0 && getus()- I2C_time < 5000)
		{
			I2C_retry--;
			TRACE("read_HMC5883 error!\r\n");
		}
		
		// MS5611 never return invalid value even on error, so no retry
		int ms5611[2];
		int ms5611result = read_MS5611(ms5611);

		// messure voltage
		int adc_oss = 0;
		for(int i=0; i<50; i++)
		{
			adc_oss += ADC_ConvertedValue;
			delayus(10);
		}
		adc_oss *= 20 * ref_vaoltage / 4095 * resistor_total / resistor_vaoltage;		// now unit is mV
		if (p->voltage <0)
			p->voltage = adc_oss;
		else
			p->voltage = p->voltage * 0.95 + 0.05 * adc_oss;			// simple low pass
		


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
					//TRACE("\r\npressure,temperature=%f, %f, ground pressure & temperature=%f, %f, height=%f, time=%f\r\n", pressure, temperature, ground_pressure, ground_temperature, altitude, (double)getus()/1000000);
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
				ms5611[1],
				ms5611[0],
				{estAccGyro.array[0], estAccGyro.array[1], estAccGyro.array[2]},
				{estGyro.array[0], estGyro.array[1], estGyro.array[2]},
				{estMagGyro.array[0], estMagGyro.array[1], estMagGyro.array[2]},
			};
			
			to_send.time = (time & (~TAG_MASK)) | TAG_IMU_DATA;
			to_send.data.imu = imu;
			
			tx_result = NRF_Tx_Dat((u8*)&to_send);
			if (tx_result == TX_OK)
				t2++;
			
			extern int tx_ok;
			extern int max_retry;
			//TRACE("\r t1,t2,ok,timeout=%d,%d,%d,%d", t1, t2, tx_ok, max_retry);
			
			pilot_data pilot = 
			{
				altitude * 100,
				{error_pid[0][0]*180*100/PI, error_pid[1][0]*180*100/PI, error_pid[2][0]*180*100/PI},
				{target[0]*180*100/PI, target[1]*180*100/PI, target[2]*180*100/PI},
				mode,
			};

			to_send.time = (time & (~TAG_MASK)) | TAG_PILOT_DATA;
			to_send.data.pilot = pilot;
			tx_result = NRF_Tx_Dat((u8*)&to_send);

			pilot_data2 pilot2 = 
			{
				{error_pid[0][1]*180*100/PI, error_pid[1][1]*180*100/PI, error_pid[2][1]*180*100/PI},
				{error_pid[0][2]*180*100/PI, error_pid[1][2]*180*100/PI, error_pid[2][2]*180*100/PI},
			};

			to_send.time = (time & (~TAG_MASK)) | TAG_PILOT_DATA2;
			to_send.data.pilot2 = pilot2;
			tx_result = NRF_Tx_Dat((u8*)&to_send);

			ppm_data ppm = 
			{
				{g_ppm_input[0], g_ppm_input[1], g_ppm_input[2], g_ppm_input[3], g_ppm_input[4], g_ppm_input[5]},
				{g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_output[4], g_ppm_output[5]},
			};

			to_send.time = (time & (~TAG_MASK)) | TAG_PPM_DATA;
			to_send.data.ppm = ppm;
			tx_result = NRF_Tx_Dat((u8*)&to_send);
		}
		
		static const float GYRO_SCALE = 2000.0 * PI / 180 / 32767 * interval;		// full scale: +/-2000 deg/s  +/-31767, 8ms interval
		
		
		vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
		vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
		vector mag = {p->mag[1], -p->mag[0], -p->mag[2]};
		vector_sub(&gyro, &gyro_zero);
		vector_multiply(&gyro, GYRO_SCALE);
		
		

		vector_rotate(&estGyro, gyro.array);
		vector_rotate(&estAccGyro, gyro.array);
		vector_rotate(&estMagGyro, gyro.array);
		
		for(int i=0; i<3; i++)
			gyroI.array[i] = radian_add(gyroI.array[i], gyro.array[i]);
		
		TRACE("gyroI:%f,%f,%f\r", gyroI.array[0] *180/PI, gyroI.array[1]*180/PI, gyroI.array[2]*180/PI);
		
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

		//float pos[3] = {roll, pitch, yaw_gyro};
		float pos[3] = {gyroI.array[0], gyroI.array[1], gyroI.array[2]};
		#if QUADCOPTER == 1		
			pos[0] = roll;
			pos[1] = pitch;
			pos[2] = yaw_gyro;
		#endif

		// quadcopter startup protection
		// if not startup in shutdown mode, we flash the light and refuse to work
		#if QUADCOPTER == 1
		if (last_mode == initializing && mode != shutdown)
		{
			while(true)
			{
				delayms(500);
				GPIO_SetBits(GPIOA, GPIO_Pin_11);
				delayms(500);
				GPIO_ResetBits(GPIOA, GPIO_Pin_11);
			}
		}
		#endif

		// mode changed?
		if (mode != last_mode)
		{
			last_mode = mode;

			target_quad_base[0] = target[0] = pos[0];
			target_quad_base[1] = target[1] = pos[1];
			target_quad_base[2] = target[2] = pos[2];
			
			for(int i=0; i<6; i++)
				rc_zero[i] = g_ppm_input[i];
		}
		
		
		// rc protecting:
		// level flight for 10 second
		// 10 degree pitch down if still no rc responding.
		// currently no throttle protection
		static int64_t last_rc_work = 0;
		if (!rc_works)
		{
			#if QUADCOPTER == 1
				target[0] = target_quad_base[0];
				target[1] = target_quad_base[1];
				target[2] = yaw_gyro;
			#else
				target[0] = -PI/18*sensor_reverse[0];						// 10 degree bank
				target[1] = (getus() - last_rc_work > 10000000) ? PI/18*sensor_reverse[1] : 0;						// 10 degree pitch down
				target[2] = yaw_gyro;
				g_ppm_output[2] = (getus() - last_rc_work > 10000000) ? 1178 : 1350;		// 1350 should be enough to maintain altitude for my plane, 1178 should harm nobody
			#endif
		}
		else
		{
			last_rc_work = getus();
			// throttle pass through
			if (rc_works)
				g_ppm_output[2] = floor(g_ppm_input[2]+0.5);
		}


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
					float rc = g_ppm_input[i==2?3:i] - rc_zero[i==2?3:i];
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

		#if QUADCOPTER == 1
		case quadcopter:
			{
				// roll & pitch
				// RC trim is accepted.
				for(int i=0; i<2; i++)
					target[i] = limit((g_ppm_input[i] - RC_CENTER) * rc_reverse[i] / RC_RANGE, -1, 1) * quadcopter_range[i] + quadcopter_trim[i];// + target_quad_base[i];

				// yaw:
				target[2] = limit((g_ppm_input[3] - RC_CENTER) * rc_reverse[2] / RC_RANGE, -1, 1) * quadcopter_range[2] + yaw_gyro + quadcopter_trim[2];
			}
			break;
		#endif
		}
		
		// calculate new pid & apply pid controll & output
		float pid[3] = {0}; // total pid for roll, pitch, yaw
		for(int i=0; i<3; i++)
		{
			float new_p = radian_sub(pos[i], target[i]) * sensor_reverse[i];
			error_pid[i][1] += new_p;																	// I
			error_pid[i][1] = limit(error_pid[i][1], -pid_limit[i][1], pid_limit[i][1]);
			new_p = limit(new_p, -pid_limit[i][0], pid_limit[i][0]);
			error_pid[i][2] = new_p - error_pid[i][0];													// D
			error_pid[i][0] = new_p;																	// P

			if (error_pid[i][1] * error_pid[i][0] < 0)
				error_pid[i][1] = 0;					// reset I if overshoot
			
			float p_rc = limit((g_ppm_input[5] - 1000.0) / 520.0, 0, 2);
			for(int j=0; j<3; j++)
				pid[i] += limit(error_pid[i][j]/ pid_limit[i][j], -1, 1) * pid_factor[i][j] * p_rc;
			pid[i] *= (1-ACRO_MANUAL_FACTOR);
			int rc = rc_reverse[i]*(g_ppm_input[i==2?3:i] - rc_zero[i==2?3:i]);
			
			//if (rc * pid[i]> 0)
				pid[i] = pid[i] + rc * ACRO_MANUAL_FACTOR / RC_RANGE;
			//else
				pid[i] = pid[i];


			int new_v = limit(rc_zero[i==2?3:i] + pid[i]*RC_RANGE, 1000, 2000);
			
			
			//g_ppm_output[i==2?3:i] = abs(new_v - g_ppm_output[i==2?3:i]) > RC_DEAD_ZONE ? new_v : g_ppm_output[i==2?3:i];
			g_ppm_output[i==2?3:i] = new_v;
		}

		#if QUADCOPTER == 1
		if (mode == quadcopter || (!rc_works) )
		{
			int motor_count = sizeof(quadcopter_mixing_matrix) / sizeof(quadcopter_mixing_matrix[0]);
			for(int i=0; i<motor_count; i++)
			{
				float mix = rc_works ? g_ppm_input[2] : 1200;
				mix = (mix-1100)*0.6 + 1100;
				for(int j=0; j<3; j++)
					mix += quadcopter_mixing_matrix[i][j] * (pid[j]/pid_limit[j][0]) * QUADCOPTER_MAX_DELTA;
				g_ppm_output[i] = mix;
				
				TRACE("pid[x] = %f, %f, %f", pid[0], pid[1], pid[2]);
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
				g_ppm_output[i] = floor(g_ppm_input[i]+0.5);
		}

		if (mode == shutdown || mode == initializing)
		{
			for(int i=0; i<6; i++)
			#if QUADCOPTER == 1
				g_ppm_output[i] = 1125;
			#else
				g_ppm_output[i] = i==2 ? 1125 : RC_CENTER;
			#endif
		}


		PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);
		
		
		float PI180 = 180/PI;
		
		TRACE("\rroll,pitch,yaw/yaw2 = %f,%f,%f,%f, target roll,pitch,yaw = %f,%f,%f, error = %f,%f,%f", roll*PI180, pitch*PI180, yaw_est*PI180, yaw_gyro*PI180, target[0]*PI180, target[1]*PI180, target[2]*PI180,
			error_pid[0][0]*PI180, error_pid[1][0]*PI180, error_pid[2][0]*PI180);
		
		TRACE(",out= %d, %d, %d, %d, input=%f,%f,%f,%f", g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_input[0], g_ppm_input[1], g_ppm_input[3], g_ppm_input[5]);
		//TRACE ("error pid[0] = %f,%f,%f", error_pid[0][0], error_pid[0][1], error_pid[0][2]);
		
		
		static int last_ppm = 1120;
		if ((int)floor(g_ppm_input[2]) != last_ppm)
		{
			//TRACE("newppm:%d\r\n", (int)floor(g_ppm_input[2]));
			last_ppm = floor(g_ppm_input[2]);
		}
		
		TRACE("input:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f, ADC=%.2f", g_ppm_input[0], g_ppm_input[1], g_ppm_input[3], g_ppm_input[5], g_ppm_input[4], g_ppm_input[5], p->voltage/1000.0 );



		GPIO_SetBits(GPIOA, GPIO_Pin_11);

		// wait for next 8ms
		while(getus()-start_tick < 8000)
			NRF_Handle_Queue();		
	}
}
