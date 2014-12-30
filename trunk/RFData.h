#ifndef __RFDATA_H__
#define __RFDATA_H__

#ifdef WIN32
typedef __int64 int64_t;
#endif

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
} imu_data_v1;

typedef struct
{
	int pressure;					// 4 byte
	unsigned short temperature;		// 2 byte
	short estAccGyro[3];			// 6 byte
	short estGyro[3];				// 6 byte
	short estMagGyro[3];			// 6 byte
} imu_data;

typedef struct
{
	unsigned short DOP[3];			// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
	float longitude;				// longitude in NDEG - +/-[degree][min].[sec/60]
	float latitude;					// latitude in NDEG - +/-[degree][min].[sec/60]
	float altitude;					// meter
	short speed;					// unit: cm/s
	unsigned satelite_in_view 	: 4;
	unsigned satelite_in_use	: 4;
	unsigned sig				: 4;// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
	unsigned fix				: 4;// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
} gps_data_v1;

typedef struct
{
	unsigned short DOP[3];				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
	short speed;					// unit: cm/s
	float longitude;				// longitude in NDEG - +/-[degree][min].[sec/60]
	float latitude;					// latitude in NDEG - +/-[degree][min].[sec/60]
	float altitude;					// meter
	unsigned satelite_in_view 	: 4;
	unsigned satelite_in_use	: 4;
	unsigned sig				: 4;// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
	unsigned fix				: 4;// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
} gps_data_v2;

typedef struct
{
	unsigned short DOP[3];			// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
	short speed;					// unit: cm/s
	int longitude;					// longitude in 1/10000000 degree
	int latitude;					// latitude in 1/10000000 degree
	float altitude;					// meter
	unsigned satelite_in_view 	: 4;
	unsigned satelite_in_use	: 4;
	unsigned sig				: 4;// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
	unsigned fix				: 4;// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
	unsigned id					: 16;// incremental id for un-guaranteed telemetry channel
} gps_data_v3;

typedef struct
{
	unsigned short DOP[3];			// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
	short speed;					// unit: cm/s
	int longitude;					// longitude in 1/10000000 degree
	int latitude;					// latitude in 1/10000000 degree
	float altitude;					// meter
	unsigned satelite_in_view 	: 4;
	unsigned satelite_in_use	: 4;
	unsigned sig				: 4;// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
	unsigned fix				: 4;// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
	unsigned id					: 4;// incremental id for un-guaranteed telemetry channel
	unsigned direction			: 12;// Track angle in degrees True, 0-360 degree, 0: north, 90: east, 180: south, 270: west, 359:almost north
} gps_data;

typedef struct
{
	char data[24];
} raw_packet;

typedef struct
{
	int altitude;							// 4 byte, unit base: 0.01 meter relative to launch ground.
	float airspeed;							// unit base: pascal. not a speed unit but a differencial pressure.
	short error[3];							// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
	short target[3];						// 6 byte, unit base: 0.01 degree, range: -18000 ~ 18000
	unsigned char fly_mode;					// 1 byte
	unsigned short mah_consumed;			// mah power consumed
} pilot_data;

typedef struct
{
	int I[3];								// 12 byte, I of PID, unit base: 0.01 degree * second
	int D[3];								// 12 byte, D of PID, unit base: 0.01 degree / second
} pilot_data2;

typedef struct
{
	short in[6];
	short out[6];
} ppm_data;

typedef struct
{
	short id;
	short accel_NED2[3];
	int lat;
	int lon;
	float error_lat;
	float error_lon;
} ned_data;

typedef struct
{
	int cmd;
	int reg;
	int value;
	int data[3];
} controll_data;

typedef struct
{
	short angle_pos[3];
	short angle_target[3];
	short speed[3];
	short speed_target[3];
} quadcopter_data;

typedef struct
{
	short climb_rate;		// high-frequency low pass filtered climb rate, unit: cm/s
	bool airborne;
	short altitude_inertia;	// unit: cm
	short accel_z_kalman;	// unit: cm/s
	short altitude_baro_raw;	// unit: cm
	short accel_z;			// unit: cm/s^2
	short loop_hz;
	short throttle_result;	// throttle result pwm from auto throttle controller
} quadcopter_data2;

typedef struct
{
	short altitude_target;
	short altitude;
	short climb_target;
	short climb;
	short accel_target;
	short accel;
	short throttle_result;
	short yaw_launch;
	short yaw_est;
	short throttle_real_crusing;
	unsigned short ultrasonic;
	short accel_I;
} quadcopter_data3;

typedef struct
{
	float data[6];
} adv_sensor_data;

typedef struct
{
	int64_t time;			// 8 byte, the top 1byte is tag
	union
	{
		sensor_data sensor;	// 24 bytes
// 		imu_data_v1 imu_v1;		// 24 bytes
		adv_sensor_data adv_sensor;		// 24 bytes
		imu_data imu;		// 24 bytes
		ned_data ned;
		pilot_data pilot;	// 23 bytes
		pilot_data2 pilot2;	// 24 bytes
		ppm_data ppm;		// 24 bytes
		controll_data controll; // 24 bytes
		//gps_data_v1 gps_v1;		// 22 bytes
 		gps_data_v2 gps_v2;		// 22 bytes
		gps_data_v3 gps_v3;		// 24 bytes
		gps_data gps;		// 24 bytes
		quadcopter_data quadcopter;	// 24 byte
		quadcopter_data2 quadcopter2;
		quadcopter_data3 quadcopter3;
	}data;
} rf_data;

#define TAG_SENSOR_DATA	0x1200000000000000
#define TAG_IMU_DATA_V1	0x8700000000000000
#define TAG_PILOT_DATA	0x6500000000000000
#define TAG_PILOT_DATA2	0x6600000000000000
#define TAG_MASK		0xff00000000000000
#define TAG_PPM_DATA	0x3300000000000000
#define TAG_CTRL_DATA	0x3400000000000000
#define TAG_GPS_DATA_V1	0x3500000000000000
#define TAG_GPS_DATA_V2	0x3600000000000000
#define TAG_GPS_DATA_V3	0x3C00000000000000
#define TAG_QUADCOPTER_DATA	0x3700000000000000
#define TAG_QUADCOPTER_DATA2	0x3800000000000000
#define TAG_QUADCOPTER_DATA3	0x3900000000000000
#define TAG_IMU_DATA	0x3A00000000000000
#define TAG_NED_DATA	0x3B00000000000000
#define TAG_ADV_SENSOR_DATA1	0x3D00000000000000
#define TAG_ADV_SENSOR_DATA2	0x3E00000000000000
#define TAG_ADV_SENSOR_DATA3	0x3F00000000000000
#define TAG_GPS_DATA	0x4000000000000000
#define TAG_RAW_DATA	0x4100000000000000

#define CTRL_CMD_SET_VALUE 0
#define CTRL_CMD_GET_VALUE 1
#define CTRL_CMD_GO 2
#define CTRL_CMD_FEEDBACK 3

#define CTRL_REG_MAGNET 0x1000

#endif
