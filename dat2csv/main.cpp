#include <Windows.h>
#include <stdio.h>
#include <conio.h>
#include <math.h>

typedef __int64 int64_t;
#include "..\RFData.h"
#include "..\common\vector.h"
#include "..\common\common.h"

int main(int argc, char **argv)
{
	if (argc<=1)
	{
		printf("dat2csv file\r\n");

		getch();

		return -1;
	}

	rf_data rf;
	pilot_data pilot = {0};
	imu_data imu = {0};
	sensor_data sensor = {0};
	ppm_data ppm = {0};


	FILE * f = fopen(argv[1], "rb");
	FILE * fo = fopen("out.csv", "wb");
	int n = 0;
	int packet = 0;
	int file = 0;
	char tmp[200];
	int64_t lasttime = 9999999999;
	while (fread(&rf, 1, 32, f) == 32)
	{
		int64_t time = rf.time & ~TAG_MASK;
		if ((rf.time & TAG_MASK) ==  TAG_IMU_DATA)
			imu = rf.data.imu;
		else if ((rf.time & TAG_MASK) ==  TAG_PILOT_DATA)
			pilot = rf.data.pilot;
		else if ((rf.time & TAG_MASK) ==  TAG_SENSOR_DATA)
			sensor = rf.data.sensor;
		else if ((rf.time & TAG_MASK) ==  TAG_PPM_DATA)
			ppm = rf.data.ppm;
		else
		{
			printf("unknown data %x, skipping\r\n", int((rf.time & TAG_MASK)>>56));
		}

		if (time < lasttime)
		{
			fclose(fo);
			file ++;
			sprintf(tmp, "out%d.csv", file);
			fo = fopen(tmp, "wb");
			fprintf(fo, "time,altitude,accel[0],aceel[1],accel[2],error[0],error[1],error[2],roll,pitch,yaw_gyro,roll_t,pitch_t,yaw_t,throttle\r\n");
		}

		lasttime = time;

		vector estAccGyro = {imu.estAccGyro[0], imu.estAccGyro[1], imu.estAccGyro[2]};
		vector estMagGyro = {imu.estMagGyro[0], imu.estMagGyro[1], imu.estMagGyro[2]};
		vector estGyro = {imu.estGyro[0], imu.estGyro[1], imu.estGyro[2]};
		vector estAccGyro16 = estAccGyro;
		vector_divide(&estAccGyro16, 16);
		float roll = radian_add(atan2(estAccGyro.V.x, estAccGyro.V.z), PI);
		float pitch = atan2(estAccGyro.V.y, (estAccGyro.V.z > 0 ? 1 : -1) *sqrt(estAccGyro.V.x*estAccGyro.V.x + estAccGyro.V.z * estAccGyro.V.z));			
		pitch = radian_add(pitch, PI);
		float xxzz = (estAccGyro16.V.x*estAccGyro16.V.x + estAccGyro16.V.z * estAccGyro16.V.z);
		float G = sqrt(xxzz+estAccGyro16.V.y*estAccGyro16.V.y);
		float yaw_est = atan2(estMagGyro.V.z * estAccGyro16.V.x - estMagGyro.V.x * estAccGyro16.V.z,
			(estMagGyro.V.y * xxzz - (estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);
		float yaw_gyro = atan2(estGyro.V.z * estAccGyro16.V.x - estGyro.V.x * estAccGyro16.V.z,
			(estGyro.V.y * xxzz - (estGyro.V.x * estAccGyro16.V.x + estGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);


		double ground_pressure = 1012.9;
		double ground_temperature = 34.28;
		double pressure = (imu.temperature + 0x10000)/100.0;
		double scaling = (double)pressure / ground_pressure;
		double temp = ((double)ground_temperature) + 273.15f;
		double altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));


// 		if (n++ %50 == 0)
 		fprintf(fo, "%.2f,%.2f,"
					"%d,%d,%d,%d,%d,%d,"
					"%f,%f,%f,%d,%d,%d,%d\r\n",
				float(time/1000000.0f), altitude*100,
 				sensor.accel[0], sensor.accel[1], sensor.accel[2], pilot.error[0], pilot.error[1], pilot.error[2],
				roll*18000/PI, pitch*18000/PI, yaw_gyro*18000/PI, pilot.target[0], pilot.target[1], pilot.target[2], 
				ppm.in[2]);
// 		fprintf(fo, "%.2f,%d,%d,%d,%d\r\n", float(time/1000000.0f), ppm.in[0], ppm.in[1], ppm.in[2], ppm.in[3]);
// 				// accel[0]前进方向，机尾方向为正
// 				// accel[1]机翼方向，右机翼方向为正
// 				// accel[2]垂直方向，往上为正
	}

	fclose(fo);
	fclose(f);

	return 0;
}