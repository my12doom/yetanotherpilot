#include <Windows.h>
#include <stdio.h>
#include <conio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

typedef __int64 int64_t;
#include "..\RFData.h"
#include "..\common\vector.h"
#include "..\common\common.h"
#include "..\common\matrix.h"
#include "..\common\fifo.h"

#include "../pos_estimator.h"

#define TIMES 1000000000

typedef struct
{
	int64_t longtitude;			// unit: 1/10000000 degree
	int64_t latitude;			// unit: 1/10000000 degree
	int64_t vlongtitude;		// unit: 1/10000000 degree/s
	int64_t vlatitude;			// unit: 1/10000000 degree/s, ~1cm/s
	int64_t time;
}position2;

typedef struct  
{
	int isAccel;
	COORDTYPE lat;
	COORDTYPE lon;
	float alat;
	float alon;
	int64_t timestamp;
} sanity_test_packet;

double NDEG2DEG(double ndeg)
{
	int degree = ndeg / 100;
	int minute = int(floor(ndeg)) % 100;	

	return degree + minute/60.0 + modf(ndeg, &ndeg)/60.0;
}

int sanity_test()
{
	FILE * sanity = fopen("Z:\\sanity.dat", "rb");;

	pos_estimator estimator;

	sanity_test_packet packet;
	while(fread(&packet, 1, sizeof(packet), sanity) == sizeof(packet))
	{
		if (packet.isAccel)
			estimator.update_accel(packet.alat, packet.alon, packet.timestamp);
		else
			estimator.update_gps(packet.lat, packet.lon, packet.timestamp);
	}

	fclose(sanity);


	return 0;
}

float sqr(float a, float b)
{
	return sqrt(a*a+b*b);
}

int main(int argc, char **argv)
{
// 	sanity_test();

	FILE * sanity = fopen("Z:\\sanity.dat", "wb");;

	CircularQueue<position2, 50> pos_hist;
	position2 gps_sample = {0};
	position2 est = {0};
	position2 abias = {0};
	position2 pbias = {0};
	position2 home = {0};
	position_meter meter = {0};
	position_meter meter_raw = {0};
	position_meter meter_est = {0};
	position_meter meter_est2 = {0};

	pos_estimator estimator;
	pos_estimator estimator2;
	estimator2.set_gps_latency(200);
	for(int i=0; i<10000; i++)
	{
		static int ii = 0;
		estimator.update_gps(ii++, ii++, (int64_t)i*1000000/333);
		estimator2.update_gps(ii++, ii++, (int64_t)i*1000000/333);

		estimator.update_accel(0, 0, (int64_t)i*1000000/333);
		estimator2.update_accel(0, 0, (int64_t)i*1000000/333);
	}


	float o;
	int res;
	bool home_set = 0;

	
	CircularQueue<float, 5> q;
	for(int i=0; i<999; i++)
	{
	q.push(1.0f);
	q.push(2.0f);
	q.push(3.0f);
	q.push(4.0f);
	q.push(5.0f+i);
	q.push(6.0f);

	res = q.peek(2, &o);
	res = q.pop_n(3);
	res = q.peek(2, &o);

	res = q.pop(&o);
	res = q.pop(&o);
	res = q.pop(&o);
	res = q.pop(&o);
	res = q.pop(&o);
	res = q.pop(&o);
	}


	double a_raw_altitude;
	{
		double scaling = (double)1014.01 / 1014;
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
	gps_data_v2 gps_v2 = {0};
	gps_data_v1 gps_v1 = {0};
	gps_data gps = {0};
	quadcopter_data quad = {0};
	quadcopter_data2 quad2 = {0};
	quadcopter_data3 quad3 = {0};
	ned_data ned[3] = {0};
	ned_data &ned0 = ned[0];
	ned_data &ned1 = ned[1];
	ned_data &ned2 = ned[2];


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
	estimator.reset();
	estimator2.reset();
// 	estimator.set_gps_latency(0);
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
			position2 s = {gps.longitude*double(TIMES/10000000), gps.latitude*double(TIMES/10000000), 0, 0, time};
			gps_sample = s;

			if (gps.DOP[1] > 0 && gps.DOP[1] < 500 && gps.fix>1 && !home_set)
			{
				home_set = true;
				home = est = s;
			}

		}
		else if ((rf.time & TAG_MASK) ==  TAG_NED_DATA)
		{
			if (rf.data.ned.id > 0 && rf.data.ned.id<3)
				ned[rf.data.ned.id] = rf.data.ned;
			else
				ned0 = rf.data.ned;

		}
// 		else if ((rf.time & TAG_MASK) ==  TAG_GPS_DATA_V1)
// 			gps_v1 = rf.data.gps_v1;
		else if ((rf.time & TAG_MASK) ==  TAG_GPS_DATA_V2)
		{
			gps_v2 = rf.data.gps_v2;

			position2 s = {NDEG2DEG(gps_v2.longitude)*TIMES, NDEG2DEG(gps_v2.latitude)*TIMES, 0, 0, time};
			gps_sample = s;

			if (gps_v2.DOP[1] > 0 && gps_v2.DOP[1] < 500 && !home_set)
			{
				home_set = true;
				home = est = s;
			}

		}
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
		float dt = (time - last_time) / 1000000.0f;
		last_time = time;

		if (home_set && dt >0 && dt < 1)
		{
			if (gps.DOP[1] > 0 && gps.DOP[1] < 500 && gps.fix>1)
			{

			}
			else
			{
				printf("WTF");
// 				ExitProcess(0);
			}


			float _time_constant_xy = 2.5f;
			float _k1_xy = 3 / _time_constant_xy;
			float _k2_xy = 3 / (_time_constant_xy*_time_constant_xy);
			float _k3_xy = 1 / (_time_constant_xy*_time_constant_xy*_time_constant_xy);

			float longtitude_to_meter = (40007000.0f/TIMES/360*cos(est.latitude * PI/TIMES / 180));
			float latitude_to_meter = (40007000.0f/TIMES/360);

			//float 
			float accel_lon = ned0.accel_NED2[1] / 1000.0f / longtitude_to_meter;
			float accel_lat = ned0.accel_NED2[0] / 1000.0f / latitude_to_meter;

			float error_lon = (gps_sample.longtitude - (est.longtitude + pbias.longtitude));
			float error_lat = (gps_sample.latitude - (est.latitude + pbias.latitude));

			abias.longtitude += error_lon * _k3_xy * dt;		// accel bias
			abias.latitude += error_lat * _k3_xy * dt;		// accel bias

			est.vlongtitude += error_lon * _k2_xy * dt;
			est.vlatitude += error_lat * _k2_xy * dt;

			pbias.longtitude += error_lon * _k1_xy * dt;
			pbias.latitude += error_lat * _k1_xy * dt;

			float v_increase_lon = (accel_lon + abias.longtitude) * dt;
			float v_increase_lat = (accel_lat + abias.latitude) * dt;

			est.longtitude += (est.vlongtitude + v_increase_lon*0.5) * dt;
			est.latitude += (est.vlatitude + v_increase_lat*0.5) * dt;

			est.vlongtitude += v_increase_lon;
			est.vlatitude += v_increase_lat;

			static int bid = 0;
			if (bid != gps.id)
			{
				estimator.update_gps(gps_sample.latitude*double(COORDTIMES/TIMES), gps_sample.longtitude*double(COORDTIMES/TIMES), time);
				// 			estimator2.update_gps(gps_sample.latitude*double(COORDTIMES/TIMES), gps_sample.longtitude*double(COORDTIMES/TIMES), time);
				bid = gps.id;


				sanity_test_packet packet = {0, gps_sample.latitude*double(COORDTIMES/TIMES), gps_sample.longtitude*double(COORDTIMES/TIMES),0,0,time};
				fwrite(&packet, 1, sizeof(packet), sanity);
				fflush(sanity);
			}
			estimator.update_accel(ned0.accel_NED2[0] / 1000.0f, ned0.accel_NED2[1] / 1000.0f, time);
			// 			estimator2.update_accel(ned.accel_NED2[0] / 1000.0f, ned.accel_NED2[1] / 1000.0f, time);
			sanity_test_packet packet = {1, 0, 0,ned0.accel_NED2[0] / 1000.0f,ned0.accel_NED2[1] / 1000.0f,time};
			fwrite(&packet, 1, sizeof(packet), sanity);
			fflush(sanity);

			static int i = 0;
			if (i++%150==0)
				printf("v=%.2f/%.2f,time%.2f,p=%f / %f\n", est.vlongtitude*longtitude_to_meter, est.vlatitude*latitude_to_meter, time/1000000.0f, est.longtitude, est.latitude);

			meter.vlongtitude = (est.vlongtitude) * longtitude_to_meter;
			meter.vlatitude = (est.vlatitude) * latitude_to_meter;

			meter_raw.longtitude = (gps_sample.longtitude - home.longtitude) * longtitude_to_meter;
			meter_raw.latitude = (gps_sample.latitude - home. latitude) * latitude_to_meter;

			meter.longtitude = (est.longtitude + pbias.longtitude - home.longtitude) * longtitude_to_meter;
			meter.latitude = (est.latitude + pbias.latitude - home. latitude) * latitude_to_meter;

			position est = estimator.get_estimation();
			meter.longtitude = (est.longtitude - home.longtitude) * longtitude_to_meter;
			meter.latitude = (est.latitude - home. latitude) * latitude_to_meter;

			meter_est.longtitude = (ned0.lon/10000000.0 * TIMES- home.longtitude) * longtitude_to_meter;
			meter_est.latitude = (ned0.lat/10000000.0  * TIMES- home. latitude) * latitude_to_meter;
// 			meter_est2.longtitude = (ned.lon2/10000000.0 * TIMES- home.longtitude) * longtitude_to_meter;
// 			meter_est2.latitude = (ned.lat2/10000000.0  * TIMES- home. latitude) * latitude_to_meter;

// 			printf("%.7f,%.8f\n", meter_raw.longtitude, gps_sample.longtitude);
		}

// 		float longtitude_to_meter = (40007000.0f/TIMES/360);
// 		float latitude_to_meter = (40007000.0f/TIMES/360);
// 		meter_est.longtitude = (ned.lon/10000000.0 * TIMES- home.longtitude) * longtitude_to_meter;
// 		meter_est.latitude = (ned.lat/10000000.0  * TIMES- home. latitude) * latitude_to_meter;
// 		meter_est2.longtitude = (ned.lon2/10000000.0 * TIMES- home.longtitude) * longtitude_to_meter;
// 		meter_est2.latitude = (ned.lat2/10000000.0  * TIMES- home. latitude) * latitude_to_meter;

		int64_t a = 0;
		float b = 1;
// 		std::cout << typeid(a+(COORDTYPE)b).name();

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

		float mag_size = sqrt((double)sensor.mag[0]*sensor.mag[0]+sensor.mag[1]*sensor.mag[1]+sensor.mag[2]*sensor.mag[2]);

// 		if (time > 300000000 && time < 400000000)
 		if (n++ %24 == 0)
//		if (time > 160000000 && time < 190000000)
 		fprintf(fo, "%.4f,%.2f,%.2f,%2f,%.2f,"
					"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
					"%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d,%d,%d,%d\r\n",
				float(time/1000000.0f), sensor.voltage/1000.0f, sensor.current/1000.0f, mag_size, imu.temperature / 100.0f,
 				sensor.accel[0], sensor.accel[1], sensor.accel[2], ned1.accel_NED2[0], ned1.accel_NED2[1], ned1.accel_NED2[2], pilot.error[0], pilot.error[1], pilot.error[2], pilot2.I[0], pilot2.D[0],
				roll*180/PI, pitch*180/PI, yaw_est*180/PI, pilot.target[0]/100.0, pilot.target[1]/100.0, pilot.target[2]/100.0, 
				(ppm.in[2]-1113)/50, pilot.fly_mode,
				ppm.in[0], ppm.in[1], ppm.in[2], ned0.accel_NED2[2], ppm.out[0], ppm.out[1], ppm.out[2], ppm.out[3],
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
//  			if (time > 254000000 && time < 264000000)
// 			if (m++ %3 == 0 && quad3.ultrasonic != 0xffff)
// 			if (home_set)
 			if (m++ %10 == 0)
			{
				fprintf(gpso, "%.4f", float(time/1000000.0f));
				fprintf(gpso, ",%d,%d,%d,%d", quad.angle_pos[2], quad.angle_target[2], quad.angle_pos[0], quad.angle_target[0]);
				fprintf(gpso, ",%d,%d,%d,%f,%f,%f,%f,", quad.angle_pos[0],quad.angle_target[0],quad.speed[0], meter_raw.longtitude, meter_raw.latitude, meter.longtitude, meter.latitude);
				fprintf(gpso, "%.1f/%d/%d/%d,%f,%f,%f,%f,%f", gps.DOP[0]/100.0f, gps.satelite_in_use, gps.fix, gps.id, quad3.altitude_target/100.01f, quad3.altitude/100.01f, sqr(ned0.error_lat,ned0.error_lon), sqr(ned1.error_lat,ned1.error_lon), sqr(ned2.error_lat,ned2.error_lon));
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