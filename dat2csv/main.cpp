#include <stdio.h>
#include <conio.h>
#include <math.h>
#include <stdlib.h>

typedef __int64 int64_t;
#include "..\RFData.h"
#include "..\common\vector.h"
#include "..\common\common.h"
#include "..\common\matrix.h"
#include "..\common\fifo.h"

typedef struct
{
	double longtitude;
	double latitude;
	double vlongtitude;
	double vlatitude;
	int64_t time;
}position;

int max(int a, int b)
{
	return a>b?a:b;
}

float NDEG2DEG(float ndeg)
{
	int degree = ndeg / 100;
	int minute = int(floor(ndeg)) % 100;	

	return degree + minute/60.0f + modf(ndeg, &ndeg)/60.0f;
}

int main(int argc, char **argv)
{
	CircularQueue<position, 50> pos_hist;
	position gps_sample = {0};
	position current = {0};
	position abias = {0};
	position pbias = {0};
	position base = {0};
	position meter = {0};
	position meter_raw = {0};


	float o;
	int res;
	bool gps_ready = 0;

	
	CircularQueue<float, 5> q;
	for(int i=0; i<999; i++)
	{
	q.push(1.0f);
	q.push(2.0f);
	q.push(3.0f);
	q.push(4.0f);
	q.push(5.0f+i);
	q.push(6.0f);

	res = q.peak(2, &o);
	res = q.pop_n(3);
	res = q.peak(2, &o);

	res = q.pop(&o);
	res = q.pop(&o);
	res = q.pop(&o);
	res = q.pop(&o);
	res = q.pop(&o);
	res = q.pop(&o);
	}


	double a_raw_altitude;
	{
		double scaling = (double)1014.001 / 1014;
		float temp = ((float)25) + 273.15f;
		a_raw_altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));

	}

	FILE * ff = fopen("Z:\\out.csv", "wb");
	fprintf(ff, "i,x,y,z\r\n");

 	for(int i=-100; i<=100; i++)
	{
		vector test = {1,1,1};
		vector test2 = test;
		vector gyro = {0,0,PI*i/100/2,};
		
		vector_rotate(&test, gyro.array);

		vector delta = vector_delta_angle(test, test2);

		fprintf(ff, "%f,%f,%f,%f\r\n", PI*i/100/2, (delta.array[0]), delta.array[1], delta.array[2]);
	}
	fclose(ff);
	if (argc<=1)
	{
		printf("dat2csv file\r\n");

		getch();

		return -1;
	}

	rf_data rf;
	pilot_data pilot = {0};
	pilot_data2 pilot2 = {0};
	imu_data imu = {0};
	sensor_data sensor = {0};
	ppm_data ppm = {0};
	gps_data_v1 gps_v1 = {0};
	gps_data gps = {0};
	quadcopter_data quad = {0};
	quadcopter_data2 quad2 = {0};
	quadcopter_data3 quad3 = {0};
	ned_data ned = {0};


	FILE * f = fopen(argv[1], "rb");
	FILE * fo = NULL;
	FILE * gpso = NULL;
	FILE * gyrof = fopen("gyro.dat", "wb");
	int n = 0;
	int m = 0;
	int packet = 0;
	int file = 0;
	char tmp[200];
	int64_t lasttime = 9999999999;
	int max_time_delta = 0;
	while (fread(&rf, 1, 32, f) == 32)
	{
		int64_t time = rf.time & ~TAG_MASK;
		if ((rf.time & TAG_MASK) ==  TAG_IMU_DATA)
			imu = rf.data.imu;
		else if ((rf.time & TAG_MASK) ==  TAG_PILOT_DATA)
			pilot = rf.data.pilot;
		else if ((rf.time & TAG_MASK) ==  TAG_PILOT_DATA2)
			pilot2 = rf.data.pilot2;
		else if ((rf.time & TAG_MASK) ==  TAG_GPS_DATA)
		{
			gps = rf.data.gps;
			position s = {NDEG2DEG(gps.longitude), NDEG2DEG(gps.latitude), 0, 0, time};
			gps_sample = s;

			if (gps.DOP[1] > 0 && gps.DOP[1] < 500 && !gps_ready)
			{
				gps_ready = true;
				base = current = s;
			}

		}
		else if ((rf.time & TAG_MASK) ==  TAG_NED_DATA)
			ned = rf.data.ned;
// 		else if ((rf.time & TAG_MASK) ==  TAG_GPS_DATA_V1)
// 			gps_v1 = rf.data.gps_v1;
		else if ((rf.time & TAG_MASK) ==  TAG_SENSOR_DATA)
		{
			sensor = rf.data.sensor;
			fwrite(sensor.gyro, 1, 6, gyrof);
		}
		else if ((rf.time & TAG_MASK) ==  TAG_QUADCOPTER_DATA)
		{
			quad = rf.data.quadcopter;
		}
		else if ((rf.time & TAG_MASK) ==  TAG_QUADCOPTER_DATA2)
		{
			quad2 = rf.data.quadcopter2;
		}
		else if ((rf.time & TAG_MASK) ==  TAG_QUADCOPTER_DATA3)
		{
			quad3 = rf.data.quadcopter3;
		}
		else if ((rf.time & TAG_MASK) ==  TAG_PPM_DATA)
			ppm = rf.data.ppm;
		else
		{
			printf("unknown data %x, skipping\r\n", int((rf.time & TAG_MASK)>>56));
		}

		// test

		static int64_t last_time = time;
		double dt = (time - last_time) / 1000000.0f;
		last_time = time;

		if (gps_ready && dt >0 && dt < 1)
		{
			double _time_constant_xy = 3.5f;
			double _k1_xy = 3 / _time_constant_xy;
			double _k2_xy = 3 / (_time_constant_xy*_time_constant_xy);
			double _k3_xy = 1 / (_time_constant_xy*_time_constant_xy*_time_constant_xy);

			double longtitude_to_meter = (40007000.0f/360*cos(current.latitude * PI / 180));
			double latitude_to_meter = (40007000.0f/360);
			//double 
			double accel_x = ned.accel_NED2[1] / 1000.0f / longtitude_to_meter;
			double accel_y = ned.accel_NED2[0] / 1000.0f / latitude_to_meter;

			double error_x = (gps_sample.longtitude - (current.longtitude + pbias.longtitude));
			double error_y = (gps_sample.latitude - (current.latitude + pbias.latitude));

			abias.longtitude += error_x * _k3_xy * dt;		// accel bias
			abias.latitude += error_y * _k3_xy * dt;		// accel bias

			current.vlongtitude += error_x * _k2_xy * dt;
			current.vlatitude += error_y * _k2_xy * dt;

			pbias.longtitude += error_x * _k1_xy * dt;
			pbias.latitude += error_y * _k1_xy * dt;

			double v_increase_x = (accel_x + abias.longtitude) * dt;
			double v_increase_y = (accel_y + abias.latitude) * dt;

			current.longtitude += (current.vlongtitude + v_increase_x*0.5) * dt;
			current.latitude += (current.vlatitude + v_increase_y*0.5) * dt;

			current.vlongtitude += v_increase_x;
			current.vlatitude += v_increase_y;

			static int i = 0;
			if (i++%150==0)
				printf("v=%.2f/%.2f,time%.2f,p=%f / %f\n", current.vlongtitude*longtitude_to_meter, current.vlatitude*latitude_to_meter, time/1000000.0f, current.longtitude, current.latitude);

			meter.longtitude = (current.longtitude + pbias.longtitude - base.longtitude) * longtitude_to_meter;
			meter.latitude = (current.latitude + pbias.latitude - base. latitude) * latitude_to_meter;
			meter_raw.longtitude = (gps_sample.longtitude - base.longtitude) * longtitude_to_meter;
			meter_raw.latitude = (gps_sample.latitude - base. latitude) * latitude_to_meter;
		}



		if (time < lasttime)
		{
			if (fo)
				fclose(fo);
			if (gpso)
				fclose(gpso);
			file ++;
			sprintf(tmp, "out%d.csv", file);
			fo = fopen(tmp, "wb");
			fprintf(fo, "time,voltage,current,airspeed,altitude,accel[0],aceel[1],accel[2],gyro[0](-roll_rate),gyro[1](-pitch_rate),gyro[2],error[0],error[1],error[2],errorI[0],errorD[0],roll,yaw,yaw_gyro,roll_t,pitch_t,yaw_t,throttle, mode,ppmi[0],ppmi[1],ppmi[2],ppmi[3],ppmo[0],ppmo[1],ppmo[2],ppmo[3],est[0],est[1],est[2],gyro[0],gyro[1],gyro[2]\r\n");
			sprintf(tmp, "gps%d.csv", file);
			gpso = fopen(tmp, "wb");
			fprintf(gpso, "time,angle[1],angle_target[1],speed[1],speed_target[1],angle[0],angle_target[0],speed[0],speed_target[0],climb,alt,alt_t,climb_t,cf,accelz\r\n");
		}
		else
		{
			max_time_delta = max((int)time - lasttime, max_time_delta);
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


		static double ground_pressure = -1012.9;
		static double ground_temperature = 34.28;

		if (imu.pressure < 10000)
		{
			int pressure = imu.temperature + 0x10000;
			int temp = imu.pressure;

			imu.pressure = pressure;
			imu.temperature = temp;
		}

		double pressure = (imu.pressure)/100.0;

		if (ground_pressure <0 && pressure > 900)
		{
			ground_pressure = pressure;
			ground_temperature = imu.temperature / 100.0;
		}

		double scaling = (double)pressure / ground_pressure;
		double temp = ((double)ground_temperature) + 273.15f;
		double altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));
		double overload = sqrt((double)sensor.accel[0]*sensor.accel[0]+ sensor.accel[1]*sensor.accel[1]+ sensor.accel[2]*sensor.accel[2]) / 2048;
		double throttle = (ppm.in[5]-1000)/520.0;

		float airspeed = pilot.airspeed<0?0:  sqrt ( 5 * 1.403 * 287.05287 * (20+273.15) *  (pow((pilot.airspeed/1000.0)/(100.0)+1.0, 1/3.5 ) - 1.0) );
		float mpu6050_temperature = sensor.temperature1/340.0f+36.53f;

		if (yaw_gyro<0)
			yaw_gyro += 2*PI;
		if (yaw_est<0)
			yaw_est += 2*PI;

// 		if (time > 300000000 && time < 400000000)
		if (n++ %54 == 0)
// 		if (time > 350000000 && time < 370000000)
 		fprintf(fo, "%.4f,%.2f,%.2f,%2f,%.2f,"
					"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
					"%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d,%d,%d,%d\r\n",
				float(time/1000000.0f), sensor.voltage/1000.0f, sensor.current/1000.0f, pilot.mah_consumed/1.0f, imu.temperature / 100.0f,
 				ned.accel_NED1[0], ned.accel_NED1[1], ned.accel_NED1[2], ned.accel_NED2[0], ned.accel_NED2[1], ned.accel_NED2[2], pilot.error[0], pilot.error[1], pilot.error[2], pilot2.I[0], pilot2.D[0],
				roll*180/PI, pitch*180/PI, yaw_est*180/PI, pilot.target[0]/100.0, pilot.target[1]/100.0, pilot.target[2]/100.0, 
				(ppm.in[2]-1113)/50, pilot.fly_mode,
				ppm.in[0], ppm.in[1], ppm.in[2], ppm.in[3], ppm.out[0], ppm.out[1], ppm.out[2], ppm.out[3],
				estAccGyro.V.x, estAccGyro.V.y, estAccGyro.V.z,
				sensor.gyro[0],sensor.gyro[1],sensor.gyro[2], sensor.mag[0]);
// 		fprintf(fo, "%.2f,%d,%d,%d,%d\r\n", float(time/1000000.0f), ppm.in[0], ppm.in[1], ppm.in[2], ppm.in[3]);
// 				// accel[0]前进方向，机尾方向为正
// 				// accel[1]机翼方向，右机翼方向为正
// 				// accel[2]垂直方向，往上为正
				// estAcc[0], 机翼方向，右机翼方向为正
				// estAcc[1], 前进方向，机头方向为正
				// estAcc[2], 垂直方向，往上为正

		if ((rf.time & TAG_MASK) ==  TAG_QUADCOPTER_DATA || (rf.time & TAG_MASK) ==  TAG_GPS_DATA || (rf.time & TAG_MASK) ==  TAG_PILOT_DATA || (rf.time & TAG_MASK) ==  TAG_PILOT_DATA2)
		{
//  			if (
// 				1 &&
// 				pilot.fly_mode == quadcopter
// 				gps.fix>1 && gps.longitude > 0 && gps.latitude > 0
//  				)
// 			if (time > 200000000 && time < 500000000)
// 			if (m++ %3 == 0 && quad3.ultrasonic != 0xffff)
 			if (m++ %50 == 0)
			{
				fprintf(gpso, "%.4f", float(time/1000000.0f));
				fprintf(gpso, ",%d,%d,%d,%d", quad.angle_pos[2], quad.angle_target[2], quad.speed[2], quad.speed_target[2]);
				fprintf(gpso, ",%d,%d,%d,%d,%d,%f,%f,", quad.angle_pos[0],quad.angle_target[0],quad.speed[0],quad.speed_target[0], quad2.climb_rate, meter_raw.longtitude, meter_raw.latitude);
				fprintf(gpso, "%f,%f,%f", gps.DOP[0]/100.0f, meter.longtitude, meter.latitude);
				fprintf(gpso, "\r\n");
			}
		}
	}

	fclose(gyrof);
	fclose(fo);
	fclose(f);
	fclose(gpso);

	printf("max time delta: %d\n", max_time_delta);

	return 0;
}