#include <stdio.h>
#include "../common/mcu.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "../common/RFData.h"
#include "../common/adc.h"
#include "../common/printf.h"
#include "../common/I2C.h"
#include "../common/PPM.h"
#include "../common/common.h"
#include "../common/vector.h"
#include "../common/build.h"
#include "../sensors/HMC5883.h"
#include "../sensors/MPU6050.h"
#include "../sensors/MPU9250.h"
#include "../sensors/mag_offset.h"
#include "../common/matrix.h"
#include "../common/param.h"
#include "../common/space.h"
#include "../library/ahrs.h"
#include "../library/ahrs2.h"
#include "../library/altitude_estimator.h"
#include "../library/altitude_estimatorCF.h"
#include "../library/altitude_controller.h"
#include "../library/log.h"

#ifndef LITE
#include "../common/crc32.h"
#include "../sensors/HMC5983.h"
#include "../sensors/MPU9250SPI.h"
#include "../sensors/ADIS16405.h"
#include "../library/pos_estimator.h"
#include "../library/pos_controll.h"
#include "../library/of_controller.h"
#include "../common/gps.h"
#include "../common/uart4.h"
#include "../common/ads1115.h"
#include "../common/ads1256.h"
#include "../sensors/sonar.h"
#include "../common/ads1115_worker.h"
#include "../common/NRF24L01.h"
#include "../fat/ff.h"
#include "../osd/MAX7456.h"
#include "../sensors/MS5611.h"
#include "../sensors/MS5611spi.h"
#include "../sensors/hp203b.h"
#include "../sensors/adxrs453.h"
#include "../common/console.h"
#include "../sensors/px4flow.h"
#else
#include "../sensors/BMP085.h"
#endif


extern "C"
{

//#include "osd/osdcore.h"

#ifdef STM32F1
	#include "../stm32/usb_com/hw_config.h"
	#include "../stm32/usb_com/usb_init.h"
#endif

#ifdef STM32F4
	#include "../usb_comF4/cdc/usbd_cdc_core.h"
	#include "../usb_comF4/core/usbd_usr.h"
	#include "../usb_comF4/usb_conf/usbd_desc.h"
	#include "../usb_comF4/usb_conf/usb_conf.h"

	#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
	#if defined ( __ICCARM__ ) /*!< IAR Compiler */
	#pragma data_alignment=4
	#endif
	#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

	__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_dev  __ALIGN_END ;
#endif
}


enum
{
	error_gyro = 1,
	error_accelerometer = 2,
	error_magnet = 4,
	error_baro = 8,
	error_RC = 16,
	error_MAX,
} critical_error;

const char * critical_error_desc[] = 
{
	"error_gyro",
	"error_accelerometer",
	"error_magnet",
	"error_baro",
	"error_RC",
	"error_MAX",
};

int critical_errors = 0;

static param crash_protect("prot", 0);		// crash protection
#define CRASH_TILT_IMMEDIATE	1
#define CRASH_COLLISION_IMMEDIATE	2

static param simple_mode("simp", 0.0f) ;
static param use_adis16405("16xx", 0.0f) ;
static param pid_factor[3][4] = 			// pid_factor[roll,pitch,yaw][p,i,d,i_limit]
{
	{param("rP1",0.60), param("rI1",0.80), param("rD1",0.03),param("rM1",PI)},
	{param("rP2",0.60), param("rI2",0.80), param("rD2",0.03),param("rM2",PI)},
	{param("rP3",1.75), param("rI3",0.25), param("rD3",0.01),param("rM3",PI)},
};
static param pid_factor2[3][4] = 			// pid_factor2[roll,pitch,yaw][p,i,d,i_limit]
{
	{param("sP1", 6), param("sI1", 0), param("sD1", 0),param("sM1", PI/45)},
	{param("sP2", 6), param("sI2", 0), param("sD2", 0),param("sM2", PI/45)},
	{param("sP3", 8), param("sI3", 0), param("sD3", 0),param("sM3", PI/45)},
};
static param quadcopter_max_climb_rate("maxC",5);
static param quadcopter_max_descend_rate("maxD", 2);
static param quadcopter_trim[3] = 
{
	param("trmR", 0 * PI / 18),				// roll
	param("trmP", 0 * PI / 180),			// pitch
	param("trmY", 0.0f),					// yaw
};
static param quadcopter_range[3] = 
{
	param("rngR", PI / 5),			// roll
	param("rngP", PI / 5),			// pitch
	param("rngY", PI / 8),			// yaw
};

static param _gyro_bias[2][4] =	//[p1,p2][temperature,g0,g1,g2]
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
static param THROTTLE_IDLE("idle", 1176);
static param acc_bias[3] = 
{
	param("abix", 0), param("abiy", 0), param("abiz", 0),
};
static param acc_scale[3] = 
{
	param("ascx", 1), param("ascy", 1), param("ascz", 1),
};
static param mag_bias[3] = 
{
	param("mbx", 0), param("mby", 0), param("mbz", 0),
};
static param mag_scale[3] = 
{
	param("mgx", 1), param("mgy", 1), param("mgz", 1),
};
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

// states
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
int imu_packet_counter = 0;
int64_t last_imu_packet_time = 0;
int byte_count;
float gyro620 = 0;
struct
{
	float data[16];
	unsigned long crc;
} packets[3], imupacket;
float ground_pressure = 0;
float ground_temperature = 0;
float rc[8] = {0};			// ailerron : full left -1, elevator : full down -1, throttle: full down 0, rudder, full left -1
float rc_mobile[4] = {0};	// rc from mobile devices
float accelz = 0;
bool airborne = false;
bool nearground = false;
float takeoff_ground_altitude = 0;
fly_mode mode = initializing;
copter_mode submode = basic;
int64_t collision_detected = 0;	// remember to clear it before arming
int64_t tilt_us = 0;	// remember to clear it before arming
uint8_t data[32];
static sensor_data *p = (sensor_data*)data;
bool gyro_bias_estimating_end = false;
vector gyro_radian;
vector gyro_LSB;
vector accel = {NAN, NAN, NAN};
vector mag;
float mag_radius = -999;
vector mag_zero = {0};
vector mag_gain = {0.7924,0.8354,0.8658};
vector accel_earth_frame_mwc;
vector accel_earth_frame;
vector mag_earth_frame;
param voltage_divider_factor("vfac",6);
int ms5611[2];
int ms5611_result = -1;
float adc_2_5_V = -1;
float VCC_3_3V = -1;
float VCC_5V = -1;
float VCC_motor = -1;
float airspeed_voltage = -1;
long last_baro_time = 0;
char climb_rate_string[10];
int64_t time;
float error_pid[3][3] = {0};		// error_pid[roll, pitch, yaw][p,i,d]
const int lpf_order = 5;
float errorD_lpf[lpf_order][3] = {0};			// variable for high order low pass filter, [order][roll, pitch, yaw]

int64_t last_tick = getus();
int64_t last_gps_tick = 0;
static unsigned short gps_id = 0;
#ifndef LITE
adis16405_burst_packet adis16405_packet;
pos_estimator estimator;
pos_controller controller;
altitude_estimator alt_estimator;
float ground_altitude;
altitude_estimatorCF alt_estimatorCF;
altitude_controller alt_controller;
OpticalFlowController of_controller;
float ground_speed_north;		// unit: m/s
float ground_speed_east;		// unit: m/s
#endif
float airspeed_sensor_data;
int adc_voltage = 0;
int adc_current = 0;
float airspeed_bias = 0;
bool has_airspeed = false;
float interval = 0;

int64_t last_rc_work = 0;
float yaw_launch;
float pid_result[3] = {0}; // total pid for roll, pitch, yaw

float a_raw_pressure = 0;
float a_raw_temperature = 0;
float a_raw_altitude = 0;



float throttle_real = 0;
float throttle_result = 0;

int16_t ads1115_2_5V = 0;
int16_t ads1115_airspeed = 0;
int16_t ads1115_voltage = 0;
int16_t ads1115_current = 0;
float mah_consumed = 0;
float wh_consumed = 0;

float sonar_distance = NAN;
int64_t last_sonar_time = getus();

short adxrs453_value = 0;
short mpu9250_value[7] = {0};
bool has_5th_channel = true;
bool has_6th_channel = true;

float bluetooth_roll = 0;
float bluetooth_pitch = 0;
int64_t bluetooth_last_update = 0;
int64_t mobile_last_update = 0;


vector gyro_temp_k = {0};		// gyro temperature compensating curve (linear)
vector gyro_temp_a = {0};
float temperature0 = 0;

px4_frame frame;
int loop_hz = 0;

double_sensor_data double_sensor = {0};


double NDEG2DEG(double ndeg)
{
	int degree = ndeg / 100;
	int minute = int(floor(ndeg)) % 100;	

	return degree + minute/60.0 + modf(ndeg, &ndeg)/60.0;
}


int calculate_baro_altitude()
{
	// raw altitude
	double scaling = (double)a_raw_pressure / ground_pressure;
	float temp = ((float)ground_temperature) + 273.15f;
	a_raw_altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));
	if (fabs(a_raw_altitude) < 5.0f)
		ground_temperature = a_raw_temperature;

	return 0;
}

int prepare_pid()
{
	// calculate current core pid position

	float new_angle_pos[3] = {euler[0], euler[1], -euler[2]};

	// the quadcopter's main pid lock on angle rate
	for(int i=0; i<3; i++)
	{
		pos[i] = angle_posD[i] = ::gyro_radian.array[i];
		angle_pos[i] = new_angle_pos[i];
	}


	switch (mode)
	{
#if QUADCOPTER == 1
	case quadcopter:
		{

			// airborne or armed and throttle up
			bool after_unlock_action = airborne || rc[2] > 0.2f;

			// throttle
			if (submode == althold || submode == poshold || submode == bluetooth || submode == optical_flow)
			{
				float v = rc[2] - 0.5f;
				float user_rate;
				if (fabs(v)<0.05f)
					user_rate = 0;
				else if (v>= 0.05f)
				{
					user_rate = (v-0.05f)/0.45f;
					user_rate = user_rate * user_rate * quadcopter_max_climb_rate;
				}
				else
				{
					user_rate = (v+0.05f)/0.45f;
					user_rate = -user_rate * user_rate * quadcopter_max_descend_rate;
				}

				float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
				alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, MOTOR_LIMIT_NONE, airborne);
				alt_controller.update(interval, user_rate);
				throttle_result = alt_controller.get_result();

				LOGE("\rthr=%f/%f", throttle_result, alt_controller.get_result());
			}
			else
			{
				throttle_result = rc[2];
			}

			// lean angle
			if (submode == basic || submode == althold)
			{
				if (after_unlock_action)	// airborne or armed and throttle up
				{
					// roll & pitch, RC trim is accepted.
					for(int i=0; i<2; i++)
					{
						float limit_l = angle_target_unrotated[i] - PI*2 * interval;
						float limit_r = angle_target_unrotated[i] + PI*2 * interval;
						angle_target_unrotated[i] = rc[i] * quadcopter_range[i] * (i==1?-1:1);	// pitch stick and coordinate are reversed 
						angle_target_unrotated[i] = limit(angle_target_unrotated[i], limit_l, limit_r);
						angle_target[i] = angle_target_unrotated[i];
					}

					if (simple_mode > 0.1f)
					{
						float diff = yaw_launch - euler[2];
						float cosdiff = cos(diff);
						float sindiff = sin(diff);
						angle_target[0] = angle_target_unrotated[0] * cosdiff - angle_target_unrotated[1] * sindiff;
						angle_target[1] = angle_target_unrotated[0] * sindiff + angle_target_unrotated[1] * cosdiff;
					}
					
				}
				else
				{
					angle_target[0] = 0;
					angle_target[1] = 0;
					angle_target[2] = angle_pos[2];
				}
			}

#ifndef LITE
			else if (submode == optical_flow)
			{
				if (after_unlock_action)	// airborne or armed and throttle up
				{
					float stick_roll = rc[0] * quadcopter_range[0];
					float stick_pitch = -rc[1] * quadcopter_range[1];	// pitch stick and coordinate are reversed

					float flow_roll = frame.flow_comp_m_y/1000.0f;
					float flow_pitch = -frame.flow_comp_m_x/1000.0f;
					of_controller.update_controller(flow_roll, flow_pitch, stick_roll, stick_pitch, interval);		// TODO: correct axises
					of_controller.get_result(&angle_target[0], &angle_target[1]);
				}
				else
				{
					angle_target[0] = 0;
					angle_target[1] = 0;
					angle_target[2] = angle_pos[2];
				}

				// TODO : handle yaw flow

			}

			else if (submode == bluetooth)
			{
				angle_target[0] = bluetooth_roll;
				angle_target[1] = bluetooth_pitch;
			}
			else if (submode == poshold)
			{
				// 10hz pos controller rate
				static int64_t last_pos_controll_time = 0;
				float dt = (getus() - last_pos_controll_time) / 1000000.0f;
				if (dt > 0.1f)
				{
					last_pos_controll_time = getus();
					if (dt < 1.0f)
					{
						position_meter meter = estimator.get_estimation_meter();


						float ne_pos[2] = {meter.latitude, meter.longtitude};
						float ne_velocity[2] = {meter.vlatitude, meter.vlongtitude};
						float desired_velocity[2] = {rc[1] * 5, rc[0] * 5};
						if (abs(desired_velocity[0]) < 0.4f)
							desired_velocity[0] = 0;
						if (abs(desired_velocity[1]) < 0.4f)
							desired_velocity[1] = 0;

						controller.provide_attitue_position(euler, ne_pos, ne_velocity);
						controller.set_desired_velocity(desired_velocity);
						controller.update_controller(dt);

						controller.get_target_angles(angle_target);
					}
				}
			}
#endif

			// yaw:
			float delta_yaw = ((fabs(rc[3]) < (float)RC_DEAD_ZONE/RC_RANGE) ? 0 : -rc[3]) * interval * QUADCOPTER_ACRO_YAW_RATE;

			float new_target = radian_add(angle_target[2], delta_yaw);
			float new_error = abs(radian_sub(angle_pos[2], new_target));
			if (new_error > (airborne?QUADCOPTER_MAX_YAW_OFFSET:(QUADCOPTER_MAX_YAW_OFFSET/5)) && new_error > abs(angle_error[2]))
				;
			else
				angle_target[2] = new_target;

			if (!after_unlock_action)
				angle_target[2] = angle_pos[2];

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
			TRACE("angle pos,target=%f,%f, air=%s\r\n", angle_pos[2] * PI180, angle_target[2] * PI180, airborne ? "true" : "false");

			// check takeoff
			if ( (alt_estimator.state[0] > takeoff_ground_altitude + 1.0f) ||
				(alt_estimator.state[0] > takeoff_ground_altitude && throttle_result > alt_controller.throttle_hover) ||
				(throttle_result > alt_controller.throttle_hover + QUADCOPTER_THROTTLE_RESERVE))
			{
				airborne = true;
				gyro_bias_estimating_end = true;
			}
		}
		break;
#endif
	}

	return 0;
}

int pid()
{
	for(int i=0; i<3; i++)
	{
		float new_p = (target[i]-pos[i]);

		if (i == 1 && QUADCOPTER == 1)
			new_p = -new_p;

		TRACE("p[%d]=%f", i, new_p*PI180);


		// I
		if (airborne)		// only integrate after takeoff
		error_pid[i][1] += new_p * interval;
		error_pid[i][1] = limit(error_pid[i][1], -pid_factor[i][3], pid_factor[i][3]);

		// D, with 40hz 4th order low pass filter
		static const float lpf_RC = 1.0f/(2*PI * 40.0f);
		float alpha = interval / (interval + lpf_RC);
		float derivative = (new_p - error_pid[i][0] )/interval;

		for(int j=0; j<lpf_order; j++)
			errorD_lpf[j][i] = errorD_lpf[j][i] * (1-alpha) + alpha * (j==0?derivative:errorD_lpf[j-1][i]);

		error_pid[i][2] = errorD_lpf[lpf_order-1][i];

		// P
		error_pid[i][0] = new_p;

		if (mode == fly_by_wire)		// D disabled for fly by wire for now
			error_pid[i][2] = 0;

		// sum
		pid_result[i] = 0;
		float p_rc = limit(rc[5]+1, 0, 2);
		for(int j=0; j<3; j++)
		{
			pid_result[i] += error_pid[i][j] * pid_factor[i][j] * power_factor;
		}
	}
	TRACE(", pid=%.2f, %.2f, %.2f\n", pid_result[0], pid_result[1], pid_result[2]);

	return 0;
}

int output()
{
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
				float mix = throttle_result;

				TRACE("\r%.2f, mode=%d", mix, submode);

				for(int j=0; j<3; j++)
					mix += quadcopter_mixing_matrix[matrix][i][j] * pid_result[j] * QUADCOPTER_THROTTLE_RESERVE;

				g_ppm_output[i] = limit(THROTTLE_IDLE + mix*(THROTTLE_MAX-THROTTLE_IDLE), THROTTLE_IDLE, THROTTLE_MAX);

				TRACE("\rpid[x] = %f, %f, %f", pid[0], pid[1], pid[2]);
				throttle_real += mix;
			}
		}
		throttle_real /= motor_count;
	}

	if (mode == _shutdown || mode == initializing)
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



#ifndef LITE

// called by main loop, only copy logs to a memory buffer, should be very fast
int save_logs()
{
	if (LOG_LEVEL == LOG_SDCARD 
		#ifndef LITE
		&& !log_ready
		#endif
	)
		return 0;

	// if the saving task is transferring logs into 2nd buffer
	if (log_pending !=0)
		return -1;

	// send/store debug data
	time = getus();
// 	packet.data.sensor.gyro[2] = adxrs453_value;
// 	packet.data.sensor.gyro[0] = mpu9250_value[6];
	log(p, TAG_SENSOR_DATA, time);
	log(&double_sensor, TAG_DOUBLE_SENSOR_DATA, time);


	imu_data imu = 
	{
		ms5611[1],
		ms5611[0],
		{estAccGyro.array[0], estAccGyro.array[1], estAccGyro.array[2]},
		{estGyro.array[0], estGyro.array[1], estGyro.array[2]},
		{estMagGyro.array[0], estMagGyro.array[1], estMagGyro.array[2]},
	};
	log(&imu, TAG_IMU_DATA, time);
	log(&frame, TAG_PX4FLOW_DATA, time);


	#ifndef LITE
	
	if (last_imu_packet_time > getus() - 1000000)
	{
		adv_sensor_data adv_sensor1 = 
		{
			imupacket.data[0], imupacket.data[1], imupacket.data[2], imupacket.data[3], imupacket.data[4], imupacket.data[5],
		};
		adv_sensor_data adv_sensor2 = 
		{
			imupacket.data[6], imupacket.data[7], imupacket.data[8], imupacket.data[9], imupacket.data[10], imupacket.data[11],
		};
		adv_sensor_data adv_sensor3 = 
		{
			imupacket.data[12], imupacket.data[13], imupacket.data[14], imupacket.data[15],
		};

// 		log(&adv_sensor1, TAG_ADV_SENSOR_DATA1);
// 		log(&adv_sensor2, TAG_ADV_SENSOR_DATA2);
// 		log(&adv_sensor3, TAG_ADV_SENSOR_DATA3);
	}

	position p = estimator.get_estimation();
	ned_data ned = 
	{
// 		{accel_earth_frame_mwc.array[0] * 1000* 9.8f, accel_earth_frame_mwc.array[1] * 1000* 9.8f, -accel_earth_frame_mwc.array[2] * 1000* 9.8f-9800.0f},
		0,
		{accel_earth_frame.array[0] * 1000, accel_earth_frame.array[1] * 1000, accel_earth_frame.array[2] * 1000},
		p.latitude * double(10000000.0/COORDTIMES), 
		p.longtitude * double(10000000.0/COORDTIMES), 
		error_lat : estimator.abias_lat,
		error_lon : estimator.abias_lon,
	};

	log(&ned, TAG_NED_DATA, time);
	#endif


	pilot_data pilot = 
	{
		alt_estimator.state[0] * 100,
		airspeed_sensor_data * 1000,
		{error_pid[0][0]*180*100/PI, error_pid[1][0]*180*100/PI, error_pid[2][0]*180*100/PI},
		{target[0]*180*100/PI, target[1]*180*100/PI, target[2]*180*100/PI},
		mode,
		mah_consumed,
	};

	log(&pilot, TAG_PILOT_DATA, time);

	pilot_data2 pilot2 = 
	{
		{error_pid[0][1]*180*100/PI, error_pid[1][1]*180*100/PI, error_pid[2][1]*180*100/PI},
		{error_pid[0][2]*180*100/PI, error_pid[1][2]*180*100/PI, error_pid[2][2]*180*100/PI},
	};

	log(&pilot2, TAG_PILOT_DATA2, time);

	ppm_data ppm = 
	{
		{g_pwm_input[0], g_pwm_input[1], g_pwm_input[2], g_pwm_input[3], g_pwm_input[4], g_pwm_input[5]},
		{g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_ppm_output[4], g_ppm_output[5]},
	};

	log(&ppm, TAG_PPM_DATA, time);

#if QUADCOPTER == 1
	quadcopter_data quad = 
	{
		angle_pos[0] * 18000/PI, angle_pos[1] * 18000/PI, angle_pos[2] * 18000/PI,
		angle_target[0] * 18000/PI, angle_target[1] * 18000/PI, angle_target[2] * 18000/PI,
		pos[0] * 18000/PI, pos[1] * 18000/PI, pos[2] * 18000/PI,
		target[0] * 18000/PI,  target[1] * 18000/PI, target[2] * 18000/PI, 
	};

	log(&quad, TAG_QUADCOPTER_DATA, time);


	quadcopter_data2 quad2 = 
	{
		alt_estimator.state[1] * 100,
		airborne,
		submode,
		alt_estimator.state[0] * 100,
		alt_estimator.state[2] * 100,
		a_raw_altitude * 100,
		accelz_mwc * 100,
		loop_hz,
		THROTTLE_IDLE + throttle_result * (THROTTLE_MAX-THROTTLE_IDLE),
		kalman_accel_bias : alt_estimator.state[3] * 1000,
		{gyro_bias[0] * 1800000/PI, gyro_bias[1] * 1800000/PI, gyro_bias[2] * 1800000/PI,}
	};

	log(&quad2, TAG_QUADCOPTER_DATA2, time);

	quadcopter_data3 quad3 = 
	{
		alt_controller.target_altitude * 100,
		alt_estimatorCF.state[0] * 100,
		alt_controller.target_climb_rate * 100,
		alt_estimatorCF.state[1] * 100,
		alt_controller.target_accel * 100,
		(accelz + alt_estimatorCF.state[3]) * 100,
		throttle_result*1000,
		yaw_launch * 18000 / PI,
		euler[2] * 18000 / PI,
		alt_controller.throttle_hover*1000,
		#ifndef LITE
		sonar_result(),
		#else
		0xffff,
		#endif
		alt_controller.accel_error_pid[1]*1000,
	};

	log(&quad3, TAG_QUADCOPTER_DATA3, time);

	// pos controller data1
	pos_controller_data pc = 
	{
		controller.setpoint[0],
		controller.setpoint[1],
		controller.pos[0],
		controller.pos[1],
		controller.target_velocity[0]*1000,
		controller.target_velocity[1]*1000,
		controller.velocity[0]*1000,
		controller.velocity[1]*1000,
	};
	log(&pc, TAG_POS_CONTROLLER_DATA1, time);


	// pos controller data2
	pos_controller_data2 pc2 = 
	{
		controller.target_accel[0]*1000,
		controller.target_accel[1]*1000,
		{
			{controller.pid[0][0]*100, controller.pid[0][1]*100, controller.pid[0][2]*100,},
			{controller.pid[1][0]*100, controller.pid[1][1]*100, controller.pid[1][2]*100,},
		}
	};

	log(&pc2, TAG_POS_CONTROLLER_DATA2, time);

#endif


#ifndef LITE
	if (last_gps_tick > getus() - 2000000)
	{
		nmeaINFO &info = *GPS_GetInfo();

		gps_data gps = 
		{
			{info.PDOP*100, info.HDOP*100, info.VDOP*100},
			info.speed/3.6*100,
			NDEG2DEG(info.lon) * 10000000, NDEG2DEG(info.lat) * 10000000, info.elv,
			info.satinfo.inview, info.satinfo.inuse,
			info.sig, info.fix,
			gps_id & 0xf,
			info.direction,
		};

		log(&gps, TAG_GPS_DATA, time);

	}
#endif
	
	return 0;
}
#endif

volatile vector imu_statics[2][4] = {0};		//	[accel, gyro][min, current, max, avg]
int avg_count = 0;

int read_sensors()
{
	#ifndef LITE
	// read external adc
	ads1115_go_on();

// 	adxrs453_value = adxrs453_get_data();
// 	adxrs453_value = adxrs453_get_data();
//	read_MPU9250(mpu9250_value);

	TRACE("\radis16405 supply = %fV", adis16405_packet.supply_measurement * 0.00242f);

	if (sonar_update() == 0)
	{
		sonar_distance = sonar_result() > 0 ? sonar_result()/1000.0f : NAN;
		last_sonar_time = getus();
		TRACE("\rdis=%.2f", sonar_distance);
	}
	#endif

	if (getus()-last_sonar_time > 200000)		//200ms
		sonar_distance = NAN;

	bool imu_error = false;
	if (read_MPU6050(&p->accel[0]) < 0 && read_MPU6050(&p->accel[0]) < 0)
	{
		LOGE("MPU6050 Error\n");
		critical_errors |= error_accelerometer | error_gyro;
		imu_error = true;
	}
	if (read_HMC5883(&p->mag[0]) < 0 && read_HMC5883(&p->mag[0]) < 0)
	{
		TRACE("HMC5883 Error\n");
#ifndef LITE
		critical_errors |= error_magnet;
#endif
	}

	for(int i=0; i<3; i++)
		p->mag[i] *= mag_gain.array[i];

	ms5611_result = read_baro(ms5611);
	if (ms5611_result < 0)
		critical_errors |= error_baro;

	mpu6050_temperature = p->temperature1  / 340.0f + 36.53f;

	// universal
	float GYRO_SCALE = 500.0f * PI / 180 / 32767;		// full scale: +/-2000 deg/s  +/-32767
	float dt = mpu6050_temperature - temperature0;
	vector gyro_bias = 
	{
		dt * gyro_temp_k.array[0] + gyro_temp_a.array[0],
		dt * gyro_temp_k.array[1] + gyro_temp_a.array[1],
		dt * gyro_temp_k.array[2] + gyro_temp_a.array[2],
	};

#ifndef LITE
// 	vector acc = {-p->accel[1], p->accel[0], p->accel[2]};
// 	vector gyro = {-p->gyro[0], -p->gyro[1], -p->gyro[2]};
// 	vector mag = {(p->mag[2]-mag_zero.array[2]), -(p->mag[0]-mag_zero.array[0]), -(p->mag[1]-mag_zero.array[1])};
	vector acc = {p->accel[1], p->accel[0], -p->accel[2]};
	vector gyro = {-p->gyro[0], p->gyro[1], p->gyro[2]};
	vector mag = {(p->mag[0]), (p->mag[2]), (p->mag[1])};
	
// 	gyro.array[1] = mpu9250_value[6];

// 	LOGE("\rmag:%d,%d,%d,%d, %.2f   ", int(mag.array[0]), int(mag.array[1]), int(mag.array[2]), (int)sqrt(mag.array[0]*mag.array[0]+mag.array[1]*mag.array[1]+mag.array[2]*mag.array[2]), yaw_est*PI180);
#else
	vector acc = {p->accel[1], p->accel[0], -p->accel[2]};
	vector gyro = {-p->gyro[0], p->gyro[1], p->gyro[2]};
	vector mag = {0};
#endif
	for(int i=0; i<3; i++)
	{
		acc.array[i] += acc_bias[i];
		acc.array[i] *= acc_scale[i];
		mag.array[i] += mag_bias[i];
		mag.array[i] *= mag_scale[i];

		acc.array[i] /= 2048.0f;
	}
	float tmp = sqrt(mag.array[0]*mag.array[0]+mag.array[1]*mag.array[1]+mag.array[2]*mag.array[2]);
	TRACE("\rmag_size:%.3f", tmp);
	gyro_LSB = gyro;
	vector_sub(&gyro, &gyro_bias);
	vector_multiply(&gyro, GYRO_SCALE);

// 	gyro.array[0] = adxrs453_value * 0.0002181f;

	#ifndef LITE

	if (use_adis16405 > 0.5f)
	{
		for(int i=0; i<3; i++)
		{
			adis16405_burst_read(&adis16405_packet);
			if (adis16405_packet.supply_measurement > 1962 && adis16405_packet.supply_measurement < 2170)
				break;		
			else
			{
				LOGE("adis16405 error\n");
			}
		}
		
		// if packet is ok and voltage is OK (4.75 ~ 5.25 V)
		float angular_rate[3] = 
		{
			float(adis16405_packet.gyro_x) * 0.05f * PI / 180,
			float(-adis16405_packet.gyro_y) * 0.05f * PI / 180,
			float(adis16405_packet.gyro_z) * 0.05f * PI / 180,
		};
		float accel_16405[3] = 
		{
			float(adis16405_packet.accel_y) * 0.003333f,
			float(adis16405_packet.accel_x) * 0.003333f,
			float(adis16405_packet.accel_z) * 0.003333f,
		};

// 		printf("\n%f,%f     ", angular_rate[0] * PI180, gyro.array[0] * PI180);
//  	printf("\n%f,%f     ", accel[2], ::accel.array[2]);

		double_sensor.acc1[0] = acc.array[0] * 2048;
		double_sensor.acc1[1] = acc.array[1] * 2048;
		double_sensor.acc1[2] = acc.array[2] * 2048;
		double_sensor.gyro1[0] = gyro.array[0] * 18000 / PI;
		double_sensor.gyro1[1] = gyro.array[1] * 18000 / PI;
		double_sensor.gyro1[2] = gyro.array[2] * 18000 / PI;

		double_sensor.acc2[0] = accel_16405[0] * 2048;
		double_sensor.acc2[1] = accel_16405[1] * 2048;
		double_sensor.acc2[2] = accel_16405[2] * 2048;
		double_sensor.gyro2[0] = angular_rate[0] * 18000 / PI;
		double_sensor.gyro2[1] = angular_rate[1] * 18000 / PI;
		double_sensor.gyro2[2] = angular_rate[2] * 18000 / PI;

		float abias[3] = {-0.003765, 0.020273, 0.010231};
		float ascale[3] = {1.001485, 0.998428, 1.006582};
		
		for(int i=0; i<3; i++)
		{
			accel_16405[i] += abias[i];
			accel_16405[i] *= ascale[i];

			acc.array[i] = accel_16405[i];
			gyro.array[i] = angular_rate[i];
		}
// 		LOGE("gyro=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", angular_rate[0]*PI180, angular_rate[1]*PI180, angular_rate[2]*PI180,
// 			gyro.array[0]*PI180, gyro.array[1]*PI180, gyro.array[2]*PI180);
	}
	#endif

	::gyro_radian = gyro;
	::mag = mag;


	// apply a 5hz LPF to accelerometer readings
	const float RC20 = 1.0f/(2*3.1415926 * 20.0f);
	float alpha20 = interval / (interval + RC20);

	if (isnan(accel.array[0]) || interval == 0)
		::accel = acc;
	else
	{
		vector_multiply(&::accel, 1-alpha20);
		vector_multiply(&acc, alpha20);
		vector_add(&::accel, &acc);
	}

// 	vector tmp_gyro = gyro;
// 	float alpha01 = interval / (interval + 1.0f/(2*3.1415926 * 0.01f));
// 	vector_multiply(&::gyro_LP, 1-alpha01);
// 	vector_multiply(&tmp_gyro, alpha01);
// 	vector_add(&::gyro_LP, &tmp_gyro);
// 	LOGE("%.3f,%.3f,%.3f\n", gyro_LP.array[0] * PI180, gyro_LP.array[1] * PI180, gyro_LP.array[2] * PI180);

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
	for(int i=0; i<3; i++)
	{
		imu_statics[0][0].array[i] = f_min(accel.array[i], imu_statics[0][0].array[i])* 2048.0f;
		imu_statics[0][1].array[i] = accel.array[i]* 2048.0f;
		imu_statics[0][2].array[i] = f_max(accel.array[i], imu_statics[0][2].array[i])* 2048.0f;
		imu_statics[0][3].array[i] = accel.array[i] + imu_statics[0][3].array[i]* 2048.0f;

		imu_statics[1][0].array[i] = f_min(gyro_LSB.array[i], imu_statics[1][0].array[i]);
		imu_statics[1][1].array[i] = gyro_LSB.array[i];
		imu_statics[1][2].array[i] = f_max(gyro_LSB.array[i], imu_statics[1][2].array[i]);
		imu_statics[1][3].array[i] = gyro_LSB.array[i] + imu_statics[1][3].array[i];
	}
	avg_count ++;
	// #ifdef STM32F1
	// 	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
	// #endif
	// #ifdef STM32F4
	// 	NVIC_EnableIRQ(OTG_HS_IRQn);
	// 	NVIC_EnableIRQ(OTG_FS_IRQn);
	// 	NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
	// 	NVIC_EnableIRQ(OTG_HS_EP1_OUT_IRQn);
	// #endif

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

	TRACE("\rvoltage=%.2fV, current=%.2fA", p->voltage/1000.0f, p->current/1000.0f);

	return 0;
}

int calculate_attitude()
{
	if (interval <=0 || interval > 0.2f)
		return -1;

	//LOGE("\rdeg:%.2f,%.2f", adxrs453_value / 80.0f, ::gyro.array[0] * PI180);

	static float mag_tolerate = 0.25f;

	vector acc_norm = accel;
	vector_multiply(&acc_norm, 9.8065f);
	float pix_acc[3] = {acc_norm.V.y, -acc_norm.V.x, -acc_norm.V.z};
	float pix_mag[3] = {mag.V.y, -mag.V.x, -mag.V.z};

	NonlinearSO3AHRSupdate(
		gyro_radian.array[0], gyro_radian.array[1], -gyro_radian.array[2],
// 		0,0,0,
		pix_acc[0], pix_acc[1], pix_acc[2],
		pix_mag[0], pix_mag[1], pix_mag[2], 
		0.15f, 0.0015f, 0.15f, 0.0015f, interval);
// 	MadgwickAHRSupdateIMU(gyro.array[0] /*+ (getus() > 15000000 ? PI*5.0f/180.0f : 0)*/, gyro.array[1], -gyro.array[2], -acc_norm.V.y, acc_norm.V.x, acc_norm.V.z, 1.5f, interval);


	euler[0] = radian_add(euler[0], quadcopter_trim[0]);
	euler[1] = radian_add(euler[1], quadcopter_trim[1]);
	euler[2] = radian_add(euler[2], quadcopter_trim[2]);

	TRACE("euler:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f, time:%f, bias:%.2f/%.2f/%.2f, pressure=%.2f \n ", euler[0]*PI180, euler[1]*PI180, euler[2]*PI180, roll*PI180, pitch*PI180, yaw_mag*PI180, getus()/1000000.0f, gyro_bias[0]*PI180, gyro_bias[1]*PI180, gyro_bias[2]*PI180, a_raw_pressure);

	for(int i=0; i<3; i++)
		accel_earth_frame.array[i] = acc_ned[i];

// 	LOGE("angle target:%.2f,%.2f,%.2f\n", angle_target[0]*PI180, angle_target[1]*PI180, angle_target[2]*PI180);


	ahrs_mwc_update(gyro_radian, accel, mag, interval);
	roll = radian_add(roll, quadcopter_trim[0]);
	pitch = radian_add(pitch, quadcopter_trim[1]);
	yaw_mag = radian_add(yaw_mag, quadcopter_trim[2]);
	yaw_gyro = radian_add(yaw_gyro, quadcopter_trim[2]);




	// calculate altitude
	if (ms5611_result == 0)
	{
		TRACE("\r\npressure,temperature=%f, %f, ground pressure & temperature=%f, %f, height=%f, climb_rate=%f, time=%f\r\n", pressure, temperature, ground_pressure, ground_temperature, altitude, climb_rate_kalman, (float)getus()/1000000);

		a_raw_pressure = ms5611[0] / 100.0f;
		a_raw_temperature = ms5611[1] / 100.0f;
		calculate_baro_altitude();
	}

	accelz_mwc = accelz;
	accelz = acc_ned[2];

	alt_estimator.set_land_effect(mode == quadcopter && (!airborne || (!isnan(sonar_distance) && sonar_distance < 1.0f) || fabs(alt_estimator.state[0] - ground_altitude) < 1.0f));
	alt_estimator.update(accelz, ms5611_result == 0 ? a_raw_altitude : NAN, interval);
	alt_estimatorCF.set_land_effect(mode == quadcopter && (!airborne || (!isnan(sonar_distance) && sonar_distance < 1.0f) || fabs(alt_estimator.state[0] - ground_altitude) < 1.0f));
	alt_estimatorCF.update(accelz, ms5611_result == 0 ? a_raw_altitude : NAN, interval);

	return 0;
}

void inline debugpin_init()
{
	// use PA-0 as cycle debugger
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_2;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
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
		LOGE("mag[%d] max/min/gain/zero=%d,%d,%.2f,%.2f\n", i, mag_max[i], mag_min[i], mag_gain.array[i], mag_zero.array[i]);

		mag_gain.array[i] = mag_gain.array[i] == 0 ? 1 : mag_gain.array[i];
	}

	int enter_throttle = (THROTTLE_IDLE+THROTTLE_MAX)/2;
	// enter magnetemeter centering mode if throttle > THROTTLE_STOP (and slowly flash all LED lights)
	while (g_pwm_input[2] > (enter_throttle))
	{
		mag_offset mag_offset;
		int mag_c = 0;
		int64_t start_tick = getus();

		delayms(100);
		if (g_pwm_input[2] < enter_throttle)
			break;

		while(g_pwm_input[2] > enter_throttle)
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
				g_ppm_output[i] = floor(g_pwm_input[i]+0.5);
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

// 				int tx_result = real_log_packet((uint8_t*)&to_send, 32);
			}

			delayms(50);
		}

		if (getus() - start_tick < 5000000)
			goto mag_load;

		// save magnetemeter centering values
		for(int i=0; i<3; i++)
		{
			mag_gain.array[i] = 1000.0f/(mag_max[i] - mag_min[i]);
			LOGE("mag[%d] max/min/gain=%d,%d,%.2f\n", i, mag_max[i], mag_min[i], mag_gain.array[i]);
		}
		mag_offset.get_result(mag_zero.array, &mag_radius);
		space_write("magzero", 7, &mag_zero, sizeof(mag_zero),NULL);
		space_write("maggain", 7, &mag_gain, sizeof(mag_gain),NULL);

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
	// update gyro temperature compensation
	if (!isnan((float)_gyro_bias[0][0]) && !isnan((float)_gyro_bias[1][0]))
	{
		float dt = _gyro_bias[1][0] - _gyro_bias[0][0];
		if (dt > 1.0f)
		{
			for(int i=0; i<3; i++)
			{
				gyro_temp_a.array[i] = _gyro_bias[0][i+1];
				gyro_temp_k.array[i] = (_gyro_bias[1][i+1] - _gyro_bias[0][i+1]) / dt;
			}
			temperature0 = _gyro_bias[0][0];
		}
		else
		{
			// treat as one point
			int group = !isnan(_gyro_bias[0][0]) ? 0 : (!isnan(_gyro_bias[1][0]) ? 1: -1);
			for(int i=0; i<3; i++)
			{
				gyro_temp_a.array[i] = group >= 0 ? _gyro_bias[group][i+1] : 0;
				gyro_temp_k.array[i] = 0;
			}
			temperature0 = group != 0 ? _gyro_bias[group][0] : 0;
		}
	}
	else
	{
		int group = !isnan(_gyro_bias[0][0]) ? 0 : (!isnan(_gyro_bias[1][0]) ? 1: -1);
		for(int i=0; i<3; i++)
		{
			gyro_temp_a.array[i] = group >= 0 ? _gyro_bias[group][i+1] : 0;
			gyro_temp_k.array[i] = 0;
		}
		temperature0 = group != 0 ? _gyro_bias[group][0] : 0;
	}



	// static base value detection
restart:
	vector mag_avg = {0};
	vector accel_avg = {0};
	vector gyro_avg = {0};
	int baro_counter = 0;
	ground_pressure = 0;
	int calibrating_count = 1500;
	ground_temperature = 0;
	for(int i=0; i<calibrating_count; i++)
	{
		long us = getus();

		TRACE("\r%d/%d", i, calibrating_count);
		#ifndef LITE
		ads1115_go_on();
		#endif

		read_sensors();
		if (ms5611_result == 0)
		{
			baro_counter ++;
			ground_pressure += ms5611[0];
			ground_temperature += ms5611[1];
		}

		mpu6050_temperature += p->temperature1  / 340.0f + 36.53f;

		vector_add(&accel_avg, &accel);
		vector_add(&mag_avg, &mag);
		vector_add(&gyro_avg, &gyro_radian);

		#ifndef LITE
		if (fabs(gyro_radian.array[0]*PI180)>5.0f || fabs(gyro_radian.array[1]*PI180)>5.0f || fabs(gyro_radian.array[2]*PI180)>5.0f)
		{
			float angular_rate[3] = 
			{
				float(adis16405_packet.gyro_x) * 0.05f * PI / 180,
				float(-adis16405_packet.gyro_y) * 0.05f * PI / 180,
				float(adis16405_packet.gyro_z) * 0.05f * PI / 180,
			};
			LOGE("wtf? %.2f,%.2f,%.2f\n", 		float(adis16405_packet.gyro_x) * 0.05f * PI / 180,
			float(-adis16405_packet.gyro_y) * 0.05f * PI / 180,
			float(adis16405_packet.gyro_z) * 0.05f * PI / 180);

			// wait a second (with blinking)
			int64_t us = getus();
			while(getus()-us < 1000000)
			{
				if ((getus()/1000)%50 > 25)
					led_all_on();
				else
					led_all_off();
			}

			LOGE("wtf\n");
			goto restart;
		}
		#endif


		airspeed_voltage = ads1115_airspeed*0.03f + airspeed_voltage * 0.97f;

		if ((getus()/1000)%50 > 25)
			led_all_on();
		else
			led_all_off();

		while(getus() - us < cycle_time)
			;
	}

	VCC_5V = 5.0f*VCC_5V/adc_2_5_V;
	VCC_3_3V = 2.5f*4095/adc_2_5_V;
	VCC_motor = voltage_divider_factor * 2.5f * VCC_motor/adc_2_5_V;
	airspeed_voltage = 2.048f * airspeed_voltage / 32767;
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

	vector_divide(&accel_avg, calibrating_count);
	vector_divide(&mag_avg, calibrating_count);
	vector_divide(&gyro_avg, calibrating_count);
	mpu6050_temperature /= calibrating_count;
	ground_pressure /= baro_counter * 100;
	ground_temperature /= baro_counter * 100;

	LOGE("base value measured\n");

	// init ahrs
	vector pix_acc = {accel_avg.V.y, -accel_avg.V.x, -accel_avg.V.z};
	float pix_gyro[3] = {gyro_avg.array[0], gyro_avg.array[1], -gyro_avg.array[2]};
	vector_multiply(&pix_acc, 9.8065f);
#ifndef LITE
	float pix_mag[3] = {mag_avg.V.y, -mag_avg.V.x, -mag_avg.V.z};
#else
	float pix_mag[3] = {0};
#endif

	ahrs_mwc_init(gyro_avg, accel_avg, mag_avg);

	NonlinearSO3AHRSinit(pix_acc.array[0], pix_acc.array[1], pix_acc.array[2],
		pix_mag[0], pix_mag[1], pix_mag[2],
		pix_gyro[0], pix_gyro[1], pix_gyro[2]);

	return 0;
}

#ifdef STM32F1
extern __IO uint32_t bDeviceState;
#define UNCONNECTED 0
int Mal_Accessed()
{
	return bDeviceState == UNCONNECTED ? 0 : 1;
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

int set_submode(copter_mode newmode)
{
	if (mode != quadcopter)
		newmode = invalid;
	
	if (newmode == submode)
		return 0;

	bool has_pos_controller = submode == poshold;
	bool to_use_pos_controller = newmode == poshold;
	bool has_alt_controller = submode == poshold || submode == althold || submode == bluetooth || submode == optical_flow;
	bool to_use_alt_controller = newmode == poshold || newmode == althold || newmode == bluetooth || newmode == optical_flow;

#ifndef LITE
	if (!has_pos_controller && to_use_pos_controller)
	{
		// reset pos controller
		position_meter meter = estimator.get_estimation_meter();
		float ne_pos[2] = {meter.latitude, meter.longtitude};
		float ne_velocity[2] = {meter.vlatitude, meter.vlongtitude};
		float desired_velocity[2] = {0, 0};

		controller.provide_attitue_position(euler, ne_pos, ne_velocity);
		controller.set_desired_velocity(desired_velocity);
		controller.get_target_angles(angle_target);
		controller.reset();
	}
#endif

	if (!has_alt_controller && to_use_alt_controller)
	{
		float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
		alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, MOTOR_LIMIT_NONE, airborne);
		alt_controller.reset();
	}

	if (newmode == optical_flow && submode != optical_flow)
	{
		of_controller.reset();
	}
	
	submode = newmode;

	return 0;
}

int set_mode(fly_mode newmode)
{
	if (newmode == mode)
		return 0;

	target[0] = pos[0];
	target[1] = pos[1];
	target[2] = pos[2];

	takeoff_ground_altitude = alt_estimator.state[0];
	yaw_launch = euler[2];
	collision_detected = 0;
	tilt_us = 0;
	throttle_result = 0;
	airborne = false;

	float alt_state[3] = {alt_estimator.state[0], alt_estimator.state[1], alt_estimator.state[3] + accelz};
	alt_controller.provide_states(alt_state, sonar_distance, euler, throttle_real, MOTOR_LIMIT_NONE, airborne);
	alt_controller.reset();


#if QUADCOPTER == 1
	for(int i=0; i<3; i++)
	{
		angle_target[i] = angle_pos[i];
		error_pid[i][1] = 0;	//reset integration
		angle_errorI[i] = 0;
	}
#endif

	mode = newmode;

	return 0;
}


int check_mode()
{
	// sub mode swtich
	if (g_pwm_input_update[5] > getus() - RC_TIMEOUT)
	{
		copter_mode newmode = submode;
		if(!has_6th_channel)
			newmode = althold;
#ifndef LITE
		else if (rc[5] < -0.6f)
			newmode = basic;
		else if (rc[5] > 0.6f)
			newmode = airborne ? optical_flow : althold;
// 			newmode = (bluetooth_last_update > getus() - 500000) ? bluetooth : althold;
// 			newmode = (estimator.healthy && airborne) ? poshold : althold;
		else if (rc[5] > -0.5f && rc[5] < 0.5f)
			newmode = althold;
#else
		else
			newmode = rc[5] < 0 ? basic : althold;
#endif

		set_submode(newmode);
	}

	if (g_pwm_input_update[4] > getus() - RC_TIMEOUT || !has_5th_channel)
	{
#if QUADCOPTER == 1
		if (mode == initializing)
			set_mode(_shutdown);

		// emergency switch
		static float last_ch4 = 0;
		if (fabs(rc[4]-last_ch4) > 0.20f)
		{
			set_mode(_shutdown);
			last_ch4 = rc[4];
			LOGE("shutdown!\n");
		}

		// arm action check: RC first four channel active, throttle minimum, elevator stick down, rudder max or min, aileron max or min, for 0.5second
		static int64_t arm_start_tick = 0;
		bool rc_fail = false;
		for(int i=0; i<4; i++)
			if (g_pwm_input_update[i] < getus() - RC_TIMEOUT)
				rc_fail = true;
		bool arm_action = !rc_fail && rc[2] < 0.1f  && fabs(rc[0]) > 0.85f
						&& fabs(rc[1]) > 0.85f && fabs(rc[3]) > 0.85f;
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
					set_mode(quadcopter);
					LOGE("armed!\n");
				}
			}
			else
			{
				arm_start_tick = getus();
			}
		}

#else
		if (g_pwm_input[4] < 1333)
			set_mode(manual);
		else if (g_pwm_input[4] > 1666)
			set_mode(acrobatic);
		else
		{
			set_mode(rc_fail);
		}
#endif
	}
	else
	{
		TRACE("warning: RC out of controll");
		set_mode(rc_fail);
	}

	return 0;
}

int osd()
{
	#ifndef LITE
	// artificial horizon
	//while((MAX7456_Read_Reg(STAT) & 0x10) != 0x00); // wait for vsync
	float roll_constrain = limit(euler[0], -30*PI/180, +30*PI/180);
	float pitch_constrain = limit(euler[1], -30*PI/180, +30*PI/180);

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
	sprintf(climb_rate_string, "%c%.1f", alt_estimator.state[0] >0 ? '+' : '-', fabs(alt_estimator.state[0]));
	MAX7456_PrintDigitString(climb_rate_string, 0, 8);
	sprintf(climb_rate_string, "%c%.1f", alt_estimator.state[1] >0 ? '+' : '-', fabs(alt_estimator.state[1]));
	MAX7456_PrintDigitString(climb_rate_string, 0, 9);

	if (isnan(alt_controller.target_altitude))
		MAX7456_PrintDigitString(alt_controller.target_climb_rate>0 ? "+":"-", 0, 10);
	else
		MAX7456_PrintDigitString(" ", 0, 10);



	#endif
	return 0;
}


int64_t land_detect_us = 0;
int land_detector()
{
	if (rc[2] < 0.1f				// low throttle
		&& fabs(alt_estimator.state[1]) < (quadcopter_max_descend_rate/4.0f)			// low climb rate : 25% of max descend rate should be reached in such low throttle, or ground was touched
// 		&& fabs(alt_estimator.state[2] + alt_estimator.state[3]) < 0.5f			// low acceleration
	)
	{
		land_detect_us = land_detect_us == 0 ? getus() : land_detect_us;

		if (getus() - land_detect_us > (airborne ? 1000000 : 3000000))		// 2 seconds for before take off, 1 senconds for landing
		{
			set_mode(_shutdown);
			LOGE("landing detected");
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

	float gforce = vector_length(&accel_delta);
	if (gforce > 1.75f)
	{
		TRACE("high G force (%.2f) detected\n", gforce);
		collision_detected = getus();
	}

	// forced shutdown if >3g external force
	if (gforce > 3.0f)
	{
		TRACE("very high G force (%.2f) detected (%.0f,%.0f,%.0f)\n", gforce, accel.array[0], accel.array[1], accel.array[2]);
		//set_mode(_shutdown);
	}

	int prot = (float)::crash_protect;

	// tilt detection
	if (rc[2] < 0.1f || prot & CRASH_TILT_IMMEDIATE)
	{
		float tilt = sqrt(euler[0]*euler[0] + euler[1]*euler[1]);
		if (vector_angle(&ground, &estAccGyro) < 0.33f)		// around 70 degree
			tilt_us = tilt_us > 0 ? tilt_us : getus();
		else
			tilt_us = 0;
	}

	if (((collision_detected > 0 && getus() - collision_detected < 5000000) && (rc[2] < 0.1f || prot & CRASH_COLLISION_IMMEDIATE)) 
		|| (tilt_us> 0 && getus()-tilt_us > 1000000))	// more than 1 second
	{
		TRACE("crash landing detected(%s)\n", (collision_detected > 0 && getus() - collision_detected < 5000000) ? "collision" : "tilt");

		set_mode(_shutdown);
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

int read_advsensor_packets()
{
#ifdef STM32F4

	byte_count = UART4_ReadPacket(packets, sizeof(packets));
	if (byte_count > 0 && byte_count > sizeof(packets[0]))
	{
		int count = byte_count/sizeof(packets[0]);
		//LOGE("count=%d\n", byte_count);
		for(int i=0; i<count; i++)
		{
			int crc = crc32(0, &packets[i], sizeof(packets[i].data));

			if (crc == packets[i].crc)
			{
				last_imu_packet_time = getus();
				imu_packet_counter++;
				gyro620 = (packets[i].data[1]-2.50f)/0.006f;
				//LOGE("\rg:%.3f,a:%.4fg, ", gyro620, packets[i].data[14]-2.50f);
				imupacket = packets[0];
			}
			else
			{
				//LOGE("bad packet found\n");
				imu_packet_counter++;
			}
		}
	}

#endif
	
	return 0;
}

int handle_uart4_cli()
{
#ifdef STM32F4
	char line[1024];
	char out[1024];
	byte_count = UART4_ReadPacket(line, sizeof(line));
	if (byte_count <= 0)
		return 0;
	
	line[byte_count-1] = 0;
	
	int out_count = parse_command_line(line, out);
	if (out_count>0)
		UART4_SendPacket(out, out_count);
	
	printf("%s,%s\n", line, out);
#endif

	return 0;
}

int handle_uart4_controll()
{
#ifdef STM32F4
	char line[1024];
	byte_count = UART4_ReadPacket(line, sizeof(line));
	if (byte_count <= 0)
		return 0;

	int len = strlen(line);
	const char * keyword = ",blue\n";
	const char * keyword2 = ",stick\n";

	if (strstr(line, keyword) == (line+len-strlen(keyword)))
	{
		char * p = (char*)strstr(line, ",");
		if (!p)
			return -1;

		*p = NULL;
		p++;
		
		bluetooth_roll = atof(line) * PI / 180;
		bluetooth_pitch = atof(p) * PI / 180;

		bluetooth_roll = limit(bluetooth_roll, -25*PI/180, 25*PI/180);
		bluetooth_pitch = limit(bluetooth_pitch, -25*PI/180, 25*PI/180);

		TRACE("%f,%f\n", bluetooth_roll * PI180, bluetooth_pitch * PI180);
		
		bluetooth_last_update = getus();
	}
	else if (strstr(line, keyword2) == (line+len-strlen(keyword2)))
	{
		if (sscanf(line, "%f,%f,%f,%f", &rc_mobile[0], &rc_mobile[1], &rc_mobile[2], &rc_mobile[3] ) == 4)
		{
			mobile_last_update = getus();
		}
	}

	else
	{
		// invalid packet
	}
#endif

	return 0;
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
		rc[i] = ppm2rc(g_pwm_input[i], rc_setting[i][0], rc_setting[i][1], rc_setting[i][2], rc_setting[i][3] > 0);
		TRACE("%.2f,", rc[i]);
	}

	rc[2] = (rc[2]+1)/2;


	// usb
	usb_lock();	// lock the system if usb transfer occurred


	led_all_off();
	
	if (getus() - tic > 1000000)
	{
		tic = getus();
		LOGE("speed: %d/%d\r\n", cycle_counter, imu_packet_counter);
		loop_hz = cycle_counter;
		cycle_counter = 0;
		imu_packet_counter = 0;
	}

	cycle_counter++;

	// flashlight
	#ifndef LITE
	time = getus();
	int time_mod_1500 = (time%1500000)/1000;
	if (time_mod_1500 < 150 || (time_mod_1500 > 200 && time_mod_1500 < 350) || (time_mod_1500 > 400 && time_mod_1500 < 550 && log_ready))
		flashlight_on();
	else
		flashlight_off();
	#endif

	// RC modes and RC fail detection
	check_mode();

	// read sensors and update altitude if new air pressure data arrived.
	int64_t us = getus();
	read_sensors();
	int64_t us2 = getus();
	if (read_px4flow(&frame) < 0)
		sonar_distance = NAN;
	else
		sonar_distance = frame.ground_distance <= 0.30f ? NAN : frame.ground_distance;
	TRACE("\rreading sensors cost %d+%dms, ground = %f m, roll=%.2f", int(us2-us), int(getus()-us2), frame.ground_distance/1000.0f, euler[0] * PI180);
//  	read_advsensor_packets();
//  	read_advsensor_packets();
//  	read_advsensor_packets();
//	handle_uart4_controll();
 	handle_uart4_cli();

	// attitude and  heading
	calculate_attitude();

	// gps		
#ifndef LITE
	if (GPS_ParseBuffer() > 0)
	{
		last_gps_tick = getus();
		gps_id++;
		nmeaINFO *info = GPS_GetInfo();

		if (info->HDOP > 0 && info->HDOP < (estimator.healthy ? 3.5f : 2.5f) && info->fix>=3)
		{
			estimator.update_gps(COORDTIMES * NDEG2DEG(info->lat), COORDTIMES * NDEG2DEG(info->lon), info->HDOP, getus());
		}
// 		else
		{
			//estimator.reset();
			//estimator2.reset();
		}

		float yaw_gps = info->direction * PI / 180;
		if (yaw_gps > PI)
			yaw_gps -= 2 * PI;

		ground_speed_east = sin(yaw_gps) * info->speed / 3.6f;
		ground_speed_north = cos(yaw_gps) * info->speed / 3.6f;

		// TODO: compute acceleration and use it to compensate roll and pitch
	}

	static int ii = 0;
	if (ii == 0)
	{
// 		estimator.update_gps(ii++, ii++, getus());
// 		estimator2.update_gps(ii++, ii++, getus());
// 		estimator3.update_gps(ii++, ii++, getus());
		//ii++;
	}

	estimator.update_accel(accel_earth_frame.array[0], accel_earth_frame.array[1], getus());

#endif


// 	osd();
	prepare_pid();
	pid();
	output();
#ifndef LITE
	save_logs();
#endif

	if (mode == quadcopter)
	{
		land_detector();
// 		crash_detector();
	}
	else
		land_detect_us = 0;



	TRACE("\rroll,pitch,yaw/yaw2 = %f,%f,%f,%f, target roll,pitch,yaw = %f,%f,%f, error = %f,%f,%f", roll*PI180, pitch*PI180, yaw_est*PI180, yaw_gyro*PI180, target[0]*PI180, target[1]*PI180, target[2]*PI180,
		error_pid[0][0]*PI180, error_pid[1][0]*PI180, error_pid[2][0]*PI180);

	TRACE("time=%.2f,inte=%.4f,out= %d, %d, %d, %d, input=%f,%f,%f,%f\n", getus()/1000000.0f, interval, g_ppm_output[0], g_ppm_output[1], g_ppm_output[2], g_ppm_output[3], g_pwm_input[0], g_pwm_input[1], g_pwm_input[3], g_pwm_input[5]);
	TRACE ("\r mag=%.2f,%.2f,%.2f  acc=%.2f,%.2f,%.2f ", estMagGyro.V.x, estMagGyro.V.y, estMagGyro.V.z, estAccGyro.V.x, estAccGyro.V.y, estAccGyro.V.z);
	TRACE ("\racc=%d,%d,%d ", p->accel[0], p->accel[1], p->accel[2]);
	TRACE("\rinput= %.2f, %.2f, %.2f, %.2f,%.2f,%.2f", g_pwm_input[0], g_pwm_input[1], g_pwm_input[2], g_pwm_input[3], g_pwm_input[4], g_pwm_input[5]);
	TRACE("\rinput:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f, ADC=%.2f", (float)g_ppm_output[0], (float)g_ppm_output[1], (float)g_ppm_output[2], (float)g_ppm_output[3], (float)g_ppm_output[0], (float)g_ppm_output[1], p->voltage/1000.0 );

	TRACE("\ryaw=%.2f, yt=%.2f, mag=%d,%d,%d           ", yaw_est *PI180, yaw_launch*PI180, p->mag[0], p->mag[1], p->mag[2]);

	TRACE("\rv/a=%dmV, %dmA, %.1f mah, %.1f Wh   ", p->voltage, p->current, mah_consumed, wh_consumed);
	TRACE("\rgyroI=%.2f", yaw_est * PI180);

	led_all_on();

	#ifndef LITE
	if (log_ready)
	{
		// flash one of the LED(A4) at 10hz
		if ((getus() % 100000) < 50000)
			GPIO_SetBits(GPIOC, GPIO_Pin_4);
	}
	#endif

	if (ms5611_result == 0)
	{
// 		LOGE("%.2f    %.2f    %.2f    %.2f\n", getus()/1000000.0f, a_raw_altitude, alt_estimator.state[0], mpu6050_temperature);
	}

	return 0;
}

#ifndef LITE
typedef struct  
{
	int isAccel;
	COORDTYPE lat;
	COORDTYPE lon;
	float alat;
	float alon;
	int64_t timestamp;
} sanity_test_packet;

int sanity_test()
{
	FIL file;
	FIL *f = &file;
	FRESULT res = f_open(f, "sanity.dat", FA_OPEN_EXISTING | FA_WRITE | FA_READ);

	pos_estimator est;

	sanity_test_packet packet;
	printf("sizeof(packet)=%d", sizeof(packet));
	while(true)
	{
		UINT br = 0;
		f_read(f, &packet, sizeof(packet), &br);
		if (br != sizeof(packet))
		{
			f_close(f);
			break;
		}
		if (packet.isAccel)
			est.update_accel(packet.alat, packet.alon, packet.timestamp);
		else
		{
			est.update_gps(packet.lat, packet.lon, 2.0f, packet.timestamp);
			printf("err=%f,%f", est.error_lat, est.error_lon);
		}
	}



	return 0;
}

#endif

int test_sensors()
{

// 	LOGE("init=%d\n", init_px4flow());
// 	while(0)
// 	{
// 		px4_frame frame = {0};
// 
// 		int us = getus();
// 		read_px4flow(&frame);
// 		us = getus() - us;
// 
// 
// 		LOGE("%f,%d, %d, %dms\n", frame.ground_distance/1000.0f, frame.frame_count, frame.qual, us);
// 		delayms(1);
// 	}

// 	init_hp203b();
// 	while(1)
// 	{
// 		int result[2];
// 		int res = read_hp203b(result);
// 		printf("\r%d,%d,%d                  ", res, result[0], result[1]);
// 
// 		delayms(10);
// 	}


// 	init_MPU9250spi();
// 	while(1)
// 	{
// 		short result16[8] = {0};
// 		int res = read_MPU9250spi(result16);
// 		float temp = (result16[3]-521)  / 340.0f + 21;
// 		printf("\r%d,%d,%d,%d,%d,%d,%d,%d, %.3fdeg                  ", res, result16[0], result16[1], result16[2], result16[3], result16[4], result16[5], result16[6], temp);
// 
// 		delayms(10);
// 	}

// 	init_HMC5983();
// 	while(1)
// 	{
// 		int result[2];
// 		short result16[8] = {0};
// 		int res = read_HMC5983(result16);
// 		printf("\r%d,%d,%d,%d                  ", res, result16[0], result16[1], result16[2]);
// 		
// 		delayms(10);
// 	}
// 	
// 	init_MS5611spi();
// 	while(1)
// 	{
// 		int result[2];
// 		int res = read_MS5611spi(result);
// 		printf("\r%d,%d,%d                  ", res, result[0], result[1]);
// 		
// 		delayms(10);
// 	}

// 	adis16405_init();
// 
// 	// configure adis16405 filter register
// 	unsigned short test2;
// 	adis16405_read_register(sens_avg, &test2);
// 	if (test2 != 0x402)
// 	{
// 		adis16405_write_register(sens_avg, 0x402);
// 	}
// 
// 	// read all register
// 	for(int i=0; i<aux_dac; i+=2)
// 	{
// 		short test;
// 		unsigned short test2;
// 		delayms(10);
// 		test = ReadFromADIS16405ViaSpi(i);
// 		delayms(10);
// 		adis16405_read_register(i, &test2);
// 		LOGE("%x=%x(%d).%x(%d)\n", i, test, test, test2, test2);
// 	}

	return 0;
}

int main(void)
{
#ifdef LITE
	//if (FLASH_GetReadOutProtectionStatus() != SET)
	//	FLASH_ReadOutProtection(ENABLE);
#endif

	//Basic Initialization
	init_timer();
	SysClockInit();
	#ifndef LITE
// 	sdcard_speed_test();
	#endif

	// priority settings
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	

	// USB
#if defined(STM32F1)
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

#ifdef QUADCOPTER
	for(int i=0; i<8; i++)
		g_ppm_output[i] = THROTTLE_STOP;
#else
	for(int i=0; i<8; i++)
		g_ppm_output[i] = i == 2 ? THROTTLE_STOP : RC_CENTER;
#endif


	ADC1_Init();
	SysTick_Config(720);
	PPM_init();
	printf_init();
	I2C_init(0x30);
	debugpin_init();
	log_init();
	if (init_MPU6050() < 0)
		critical_errors |= error_accelerometer | error_gyro;
	
// 	if (init_MPU9250() < 0)
// 		critical_errors |= error_accelerometer | error_gyro;
	
	#ifndef LITE
	estimator.set_gps_latency(0);
	adxrs453_init();
	if (init_MS5611() < 0)
		critical_errors |= error_baro;
	if (init_HMC5883() < 0)
		critical_errors |= error_magnet;	
	GPS_Init(115200);
#ifdef STM32F4
	UART4_Init(115200, 0);
#endif
	NRF_Init();
 	//MAX7456_SYS_Init();
 	Max7456_Set_System(1);
	//sonar_init();
	#else
	if (init_BMP085() < 0)
		critical_errors |= error_baro;
	#endif
	flashlight_on();

	#if PCB_VERSION == 3 && !defined(LITE)
	ads1115_init();	
 	ads1115_new_work(ads1115_speed_860sps, ads1115_channnel_AIN0, ads1115_gain_4V, &ads1115_voltage);
 	ads1115_new_work(ads1115_speed_860sps, ads1115_channnel_AIN1_AIN3, ads1115_gain_4V, &ads1115_current);
 	ads1115_new_work(ads1115_speed_860sps, ads1115_channnel_AIN2_AIN3, ads1115_gain_2V, &ads1115_airspeed);
 	ads1115_new_work(ads1115_speed_860sps, ads1115_channnel_AIN3, ads1115_gain_4V, &ads1115_2_5V);	
	#endif
		
	p->voltage = -32768;
	p->current = -32768;






#ifndef LITE
	test_sensors();

	//magnet_calibration();
#endif
	sensor_calibration();

	has_5th_channel = g_pwm_input_update[4] > getus()-500000;
	has_6th_channel = g_pwm_input_update[5] > getus()-500000;

	// check critical errors
	if (critical_errors != 0)
	{
		LOGE("critical_errors : %x (", critical_errors);
		for(int i=0; (1<<i)<error_MAX; i++)
		{
			if ((1<<i) & critical_errors)
				LOGE("%s %s", i==0?"":" | ", critical_error_desc[i]);
		}
		LOGE(" )\n");

		led_all_off();
		flashlight_off();

		while (1)
		{
			// blink error code!
			for(int i=1; i<error_MAX; i<<=1)
			{
				led_all_on();
				flashlight_on();

				handle_uart4_cli();

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
#if __GNUC__ > 0
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
	TIM_TimeBaseStructure.TIM_Period=6000-1;
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

int64_t tick;
void TIM8_BRK_TIM12_IRQHandler(void)
{

#ifndef LITE

	int64_t t = getus();
	int dt = t-tick;
	if (dt > 15000)
		TRACE("long log interval:%d\n", dt);


	int64_t starttick = getus();
	int res = log_flush();
	starttick = getus() - starttick;

	if (starttick > 10000)
		TRACE("long log time:%d\n", int(starttick));

	if (res == 0)
		tick = t;
#endif
	TIM_ClearITPendingBit(TIM12 , TIM_FLAG_Update);
}
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
