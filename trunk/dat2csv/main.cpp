#include <Windows.h>
#include <stdio.h>
#include <conio.h>

typedef __int64 int64_t;
#include "..\RFData.h"

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


	FILE * f = fopen(argv[1], "rb");
	FILE * fo = fopen("out.csv", "wb");
	int n = 0;
	int packet = 0;
	int file = 0;
	char tmp[200];
	int64_t lasttime = -1;
	while (fread(&rf, 1, 32, f) == 32)
	{
		int64_t time = rf.time & ~TAG_MASK;
		if ((rf.time & TAG_MASK) ==  TAG_IMU_DATA)
			imu = rf.data.imu;
		else if ((rf.time & TAG_MASK) ==  TAG_PILOT_DATA)
			pilot = rf.data.pilot;
		else if ((rf.time & TAG_MASK) ==  TAG_SENSOR_DATA)
			sensor = rf.data.sensor;
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
		}

		lasttime = time;


		if (n++ %50 == 0)
		fprintf(fo, "%.2f,%.2f,%d,%d,%d,%d\r\n", float(time/1000000.0f), pilot.altitude*10.0,
				sensor.accel[0], sensor.accel[1], pilot.error[2], sensor.accel[2]);
				// accel[0]前进方向，机尾方向为正
				// accel[1]机翼方向，右机翼方向为正
				// accel[2]垂直方向，往上为正
	}

	fclose(fo);
	fclose(f);

	return 0;
}