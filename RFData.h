#ifndef __RFDATA_H__
#define __RFDATA_H__

typedef struct
{
	short mag[3];
	short accel[3];
	short temperature1;		// raw temperature by MPU6050
	short gyro[3];			// roll, pitch, yaw
	short voltage;			// unit base: mV
	short current;			// unit base: mA
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
	unsigned short DOP[3];				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
	float longitude;				// longitude in NDEG - +/-[degree][min].[sec/60]
	float latitude;					// latitude in NDEG - +/-[degree][min].[sec/60]
	float altitude;					// meter
	short speed;					// unit: cm/s
	unsigned satelite_in_view 	: 4;
	unsigned satelite_in_use	: 4;
	unsigned sig				: 4;// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
	unsigned fix				: 4;// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
} gps_data;

typedef struct
{
	int altitude;							// 4 byte, unit base: 0.01 meter relative to launch ground.
	short error[3];							// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
	short target[3];						// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
	unsigned char fly_mode;					// 1 byte
} pilot_data;

typedef struct
{
	int I[3];								// 12 byte, I of PID, unit base: 0.01 degree * x(undefined time unit, current implementation is ~8ms)
	int D[3];								// 12 byte, D of PID, unit base: 0.01 degree / x(undefined time unit, same as I)
} pilot_data2;

typedef struct
{
	short in[6];
	short out[6];
} ppm_data;

typedef struct
{
	int cmd;
	int reg;
	int value;
} controll_data;

typedef struct
{
	int64_t time;			// 8 byte, the top 1byte is tag
	union
	{
		sensor_data sensor;	// 20 byte
		imu_data imu;		// 24 byte
		pilot_data pilot;	// 23 byte
		pilot_data2 pilot2;	// 24 byte
		ppm_data ppm;		// 24 byte
		controll_data controll; // 12 byte
		gps_data gps;		// 
	}data;
} rf_data;

#define TAG_SENSOR_DATA	0x1200000000000000
#define TAG_IMU_DATA	0x8700000000000000
#define TAG_PILOT_DATA	0x6500000000000000
#define TAG_PILOT_DATA2	0x6600000000000000
#define TAG_MASK		0xff00000000000000
#define TAG_PPM_DATA	0x3300000000000000
#define TAG_CTRL_DATA	0x3400000000000000
#define TAG_GPS_DATA	0x3500000000000000

#define CTRL_CMD_SET_VALUE 0
#define CTRL_CMD_GET_VALUE 1
#define CTRL_CMD_GO 2

#endif
