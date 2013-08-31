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
	int64_t time;			// 8 byte, the top 1byte is tag
	union
	{
		sensor_data sensor;	// 20 byte
		imu_data imu;		// 24 byte
	}data;
} rf_data;

#define TAG_SENSOR_DATA	0x1200000000000000
#define TAG_IMU_DATA	0x8700000000000000
#define TAG_MASK		0xff00000000000000

#endif
