#ifndef __RFDATA_H__
#define __RFDATA_H__

typedef struct
{
	short mag[3];
	short accel[3];
	short temperature1;		// raw temperature by MPU6050
	short gyro[3];			// roll, pitch, yaw	
} sensor_data;

typedef struct
{
	short estAccGyro[3];			// 6 byte
	short estGyro[3];				// 6 byte
	short estMagGyro[3];			// 6 byte
	unsigned short temperature;		// 2 byte
	int pressure;					// 4 byte
} imu_data;

typedef struct
{
	unsigned char fly_mode;					// 1 byte
	short error[3];							// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
	short target[3];						// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
	int altitude;							// 4 byte, unit base: 0.01 meter relative to launch ground.
	short rc[3];							// 6 byte, roll, pitch, yaw channel, switch is ignored.
} pilot_data;

typedef struct
{
	int64_t time;			// 8 byte, the top 1byte is tag
	union
	{
		sensor_data sensor;	// 20 byte
		imu_data imu;		// 24 byte
		pilot_data pilot;	// 23 byte
	}data;
} rf_data;

#define TAG_SENSOR_DATA	0x1200000000000000
#define TAG_IMU_DATA	0x8700000000000000
#define TAG_PILOT_DATA	0x6500000000000000
#define TAG_MASK		0xff00000000000000

#endif
