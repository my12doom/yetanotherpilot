#ifndef __RFDATA_H__
#define __RFDATA_H__

typedef struct
{
	short mag[3];
	short accel[3];
	short temperature1;		// raw temperature by MPU6050
	short gyro[3];			// roll, pitch, yaw
	int voltage;	// unit base: mV
} sensor_data;

typedef struct
{
	unsigned short temperature;		// 2 byte
	int pressure;					// 4 byte
	short estAccGyro[3];			// 6 byte
	short estGyro[3];				// 6 byte
	short estMagGyro[3];			// 6 byte
} imu_data;

typedef struct
{
	int altitude;							// 4 byte, unit base: 0.01 meter relative to launch ground.
	short error[3];							// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
	short target[3];						// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
	unsigned char fly_mode;					// 1 byte
} pilot_data;

typedef struct
{
	short in[6];
	short out[6];
} ppm_data;

typedef struct
{
	int64_t time;			// 8 byte, the top 1byte is tag
	union
	{
		sensor_data sensor;	// 20 byte
		imu_data imu;		// 24 byte
		pilot_data pilot;	// 23 byte
		ppm_data ppm;
	}data;
} rf_data;

#define TAG_SENSOR_DATA	0x1200000000000000
#define TAG_IMU_DATA	0x8700000000000000
#define TAG_PILOT_DATA	0x6500000000000000
#define TAG_MASK		0xff00000000000000
#define TAG_PPM_DATA	0x3300000000000000


#endif
