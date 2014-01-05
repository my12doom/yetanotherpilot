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
#include "sensors/mag_offset.h"
#include "common/gps.h"

int abs(int x)
{
	return x>0 ? x : -x;
}

// a helper
bool calculate_roll_pitch(vector *accel, vector *mag, vector *accel_target, vector *mag_target, float *roll_pitch);

void inline debugpin_init()
{
	// use PA-0 as cycle debugger
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);	
}
void inline debugpin_high()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_8);	
}
void inline debugpin_low()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);	
}

int main(void)
{
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
	debugpin_init();
	GPS_Init(9600);
	
	mag_offset mag_offset;
	
	int mode = initializing;
	u8 data[TX_PLOAD_WIDTH];
	sensor_data *p = (sensor_data*)data;
	vector estAccGyro = {0};			// for roll & pitch
	vector estMagGyro = {0};			// for yaw
	vector estGyro = {0};				// for gyro only yaw, yaw lock on this
	vector groundA;						// ground accerometer vector
	vector groundM;						// ground magnet vector
	
	vector mag_avg = {0};
	vector gyro_zero = {0};
	vector accel_avg = {0};
	double ground_pressure = 0;
	double ground_temperature = 0;
	double altitude = -999;
	int baro_counter = 0;
	int ms5611[2];
	p->voltage = -32768;
	p->current = -32768;

	// static base value detection
	for(int i=0; i<1000; i++)
	{
		TRACE("\r%d/1000", i);
		read_MPU6050(p->accel);
		read_HMC5883(p->mag);
		
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
		
		if ((getus()/1000)%50 > 25)
			debugpin_high();
		else
			debugpin_low();

		delayms(2);
	}

	groundA = accel_avg;
	groundM = mag_avg;
	
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
	vector targetVA;		// target accelerate vector
	vector targetVM;		// target magnet vector
	
	double temperature = 0;
	double pressure = 0;
	int oss = BARO_OSS;
	

	// the main loop

	int last_mode = mode;
	float rc_zero[] = {1520, 1520, 1520, 1520, 1520, 1520};
	float error_pid[3][3] = {0};		// error_pid[roll, pitch, yaw][p,i,d]
	int64_t last_tick = getus();


	while(1)
	{
		static const float factor = 0.997;
		static const float factor_1 = 1-factor;
		int64_t start_tick = getus();
		float interval = (start_tick-last_tick)/1000000.0f;
		last_tick = start_tick;
		bool rc_works = true;
		
		debugpin_high();
		
		// RC modes and RC fail detection
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
			TRACE("warning: RC out of controll");
			rc_works = false;
			mode = rc_fail;	
		}
		
		

		// always read sensors and calculate attitude
		read_MPU6050(&p->accel[0]);		
		read_HMC5883(&p->mag[0]);		
		int ms5611result = read_MS5611(ms5611);


		float mag_data[4] = {p->mag[0], p->mag[1], p->mag[2], 1};
		mag_offset.add_value(mag_data);
		
		static int mag_c = 0;
		if (mag_c++ % 100 == 0)
		{
			float center[3], r;
			mag_offset.get_result(center, &r);
			TRACE("mag: center=%f,%f,%f, r=%f\r\n", center[0], center[1], center[2], r);
		}
		
		// messure voltage
		int adc_voltage = 0;
		int adc_current = 0;
		ADC1_SelectPin(GPIO_Pin_4);
		for(int i=0; i<50; i++)
			adc_voltage += ADC1_Read();
		ADC1_SelectPin(GPIO_Pin_0);
		for(int i=0; i<50; i++)
			adc_current += ADC1_Read();
		adc_voltage *= 20 * ref_vaoltage / 4095 * resistor_total / resistor_vaoltage;		// now unit is mV

		adc_current *= 20 * ref_vaoltage / 4095;		// now unit is mV
		adc_current = hall_sensor_ref_voltage/2*1000 - adc_current;	// now delta mV
		adc_current /= hall_sensor_sensitivity;

		if (p->voltage <-30000)
			p->voltage = adc_voltage;
		else
			p->voltage = p->voltage * 0.95 + 0.05 * adc_voltage;			// simple low pass
		
		if (p->current <-30000)
			p->current = adc_current;
		else
			p->current = p->current * 0.95 + 0.05 * adc_current;			// simple low pass
		


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
					TRACE("\r\npressure,temperature=%f, %f, ground pressure & temperature=%f, %f, height=%f, time=%f\r\n", pressure, temperature, ground_pressure, ground_temperature, altitude, (double)getus()/1000000);
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
			
			GPS_ParseBuffer();
			{
				nmeaINFO &info = *GPS_GetInfo();

				gps_data gps = 
				{
					{info.PDOP*100, info.HDOP*100, info.VDOP*100},
					info.lon, info.lat, info.elv, info.speed/3.6*100,
					info.satinfo.inview, info.satinfo.inuse,
					info.sig, info.fix,
				};

				to_send.time = (time & (~TAG_MASK)) | TAG_GPS_DATA;
				to_send.data.gps = gps;
				tx_result = NRF_Tx_Dat((u8*)&to_send);
			}
		}
		
		float GYRO_SCALE = 2000.0 * PI / 180 / 32767 * interval;		// full scale: +/-2000 deg/s  +/-31767, 8ms interval
		
		
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
		else
		{
			TRACE("rapid movement (%fg, angle=%f)", acc_g, acos(vector_angle(&estAccGyro, &acc)) * 180 / PI );
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
				debugpin_high();
				delayms(500);
				debugpin_low();
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

			targetVA = estAccGyro;
			targetVM = estMagGyro;

			vector_normalize(&targetVA);
			vector_normalize(&targetVM);
			
			for(int i=0; i<6; i++)
				rc_zero[i] = g_ppm_input[i];
		}
		
		
		// rc protecting:
		// level flight for 10 second
		// 5 degree pitch down if still no rc responding.
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
				target[1] = (getus() - last_rc_work > 10000000) ? PI/36*sensor_reverse[1] : 0;						// 5 degree pitch down
				target[2] = yaw_gyro;
				g_ppm_output[2] = (getus() - last_rc_work > 10000000) ? 1178 : 1350;		// 1350 should be enough to maintain altitude for my plane, 1178 should harm nobody


				vector acc = estAccGyro;
				vector mag = estMagGyro;
				vector_normalize(&acc);
				vector_normalize(&mag);

				targetVA = groundA;
				targetVM = groundM;

				float delta[3] = {target[0], target[1], 0};
				vector_rotate(&targetVA, delta);
				vector_rotate(&targetVM, delta);

				calculate_roll_pitch(&acc, &mag, &targetVA, &targetVM, delta);

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
		float rc_d[3] = {0};
		float rc_dv[3] = {0};
		float errorV[2] = {0};
		switch (mode)
		{
		case acrobatic:
			{
				float rate[3] = {ACRO_ROLL_RATE * interval / RC_RANGE, 
								ACRO_PITCH_RATE * interval / RC_RANGE,
								ACRO_YAW_RATE * interval / RC_RANGE};

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

#ifdef VECTOR_PID

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

				calculate_roll_pitch(&acc, &mag, &targetVA, &targetVM, errorV);
#endif
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
#ifdef VECTOR_PID
			new_p = (i<2 ? errorV[i] : 0) * sensor_reverse[i];
#endif
			error_pid[i][1] += new_p;																	// I
			error_pid[i][1] = limit(error_pid[i][1], -pid_limit[i][1], pid_limit[i][1]);
			error_pid[i][2] = new_p - error_pid[i][0] + rc_d[i]* sensor_reverse[i];													// D
			error_pid[i][0] = new_p;																	// P

			if (error_pid[i][1] * error_pid[i][0] < 0)
				error_pid[i][1] = 0;					// reset I if overshoot
			
			float p_rc = limit((g_ppm_input[5] - 1000.0) / 520.0, 0, 2);
			for(int j=0; j<3; j++)
				pid[i] += limit(limit(error_pid[i][j],-pid_limit[i][j],+pid_limit[i][j]) / pid_limit[i][j], -1, 1) * pid_factor[i][j] * p_rc;
			pid[i] *= (1-ACRO_MANUAL_FACTOR);
			int rc = rc_reverse[i]*(g_ppm_input[i==2?3:i] - rc_zero[i==2?3:i]);
			
			//if (rc * pid[i]> 0)
				pid[i] = pid[i] + rc * ACRO_MANUAL_FACTOR / RC_RANGE;
			//else
				pid[i] = pid[i];


			int new_v = limit(rc_zero[i==2?3:i] + pid[i]*RC_RANGE, 1000, 2000);
			
			
			g_ppm_output[i==2?3:i] = abs(new_v - g_ppm_output[i==2?3:i]) > RC_DEAD_ZONE ? new_v : g_ppm_output[i==2?3:i];
			//g_ppm_output[i==2?3:i] = new_v;
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



		debugpin_low();

		// wait for next 8ms
		while(getus()-start_tick < cycle_time)
			NRF_Handle_Queue();		
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
