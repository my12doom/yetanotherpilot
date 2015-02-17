#include <Windows.h>
#include <stdio.h>
#include <conio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

typedef __int64 int64_t;
#include "..\..\common\RFData.h"
#include "..\..\common\vector.h"
#include "..\..\common\common.h"
#include "..\..\common\matrix.h"
#include "..\..\common\fifo.h"

#include "../../library/pos_estimator.h"
#include "../../library/ahrs.h"

#define TIMES 1000000000
#define countof(x) (sizeof(x)/sizeof(x[0]))

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
			estimator.update_gps(packet.lat, packet.lon, 2.0, packet.timestamp);
	}

	fclose(sanity);


	return 0;
}


float still_start_time;
float still_start_accel[3];
float still_accel_avg[3];
float still_temp_avg;
int still_counter;


int point_count = 0;
const int max_point = 8192;
float points[max_point][4];

vector acc;

typedef struct
{
	double parameter[15];
} func_vector;

double func_3d(func_vector v);
double func_accel_matrix(func_vector v);

int fitting_accel_3d()
{

	for(int i=0; i<point_count; i++)
	{
		points[i][0] /= 2048;
		points[i][1] /= 2048;
		points[i][2] /= 2048;
	}

	func_vector v = {0,0,0,1,1,1};
	const double step = 0.0001;

	double delta[6];
	for(int j=0; j<100000; j++)
	{
		double result = func_3d(v);
		double delta_length = 0;
		for(int i=0; i<6; i++)
		{
			func_vector v2 = v;
			v2.parameter[i] += step;

			delta[i] = func_3d(v2) - result;
			delta_length += delta[i] * delta[i];
		}

		delta_length = sqrt(delta_length);
		for(int i=0; i<6; i++)
		{
			delta[i] /= delta_length;
			v.parameter[i] -= step * delta[i];
		}
	}

	for(int i=0; i<point_count; i++)
	{
		printf("I:%f,%f,%f(%f)", points[i][0], points[i][1], points[i][2], sqrt(points[i][0]*points[i][0]+points[i][1]*points[i][1]+points[i][2]*points[i][2]));
		printf("\n");

	}

	for(int i=0; i<point_count; i++)
	{
		float a0 = (points[i][0] + v.parameter[0]) * v.parameter[3];
		float a1 = (points[i][1] + v.parameter[1]) * v.parameter[4];
		float a2 = (points[i][2] + v.parameter[2]) * v.parameter[5];

		printf("O:%f,%f,%f(%f)", a0, a1, a2, sqrt(a0*a0+a1*a1+a2*a2));
		printf("\n");
	}

	printf("bias:");
	for(int i=0; i<3; i++)
	{
		printf("%.2f(%f),", v.parameter[i]* 2048, v.parameter[i]);
	}

	printf("\nscale:");
	for(int i=0; i<3; i++)
		printf("%f,", v.parameter[i+3]);
	printf("\n");


	return 0;
}

int fitting_mag_3d()
{
	float mins[3] = {9999,9999,9999};
	float maxs[3] = {-9999,-9999,-9999};

	for(int i=0; i<point_count; i++)
	{
		for(int j=0; j<3; j++)
		{
			points[i][j] /= 500;
			mins[j] = min(mins[j], points[i][j]);
			maxs[j] = max(maxs[j], points[i][j]);
		}
	}

	func_vector v = {0,0,0,1,1,1};
	for(int i=0; i<3; i++)
	{
		v.parameter[i] = (maxs[i] + mins[i])/2;
		v.parameter[i+3] = 1/((maxs[i] - mins[i])/2);
	}
	func_vector v_initial = v;
	const double step = 0.001;

	double delta[6];
	for(int j=0; j<1000000; j++)
	{
		double result = func_3d(v);
		double delta_length = 0;
		for(int i=0; i<6; i++)
		{
			func_vector v2 = v;
			v2.parameter[i] += step;

			delta[i] = func_3d(v2) - result;
			delta_length += delta[i] * delta[i];
		}

		delta_length = sqrt(delta_length);
		for(int i=0; i<6; i++)
		{
			delta[i] /= delta_length;
			v.parameter[i] -= step * delta[i];

// 			if (i<3)
// 				v.parameter[i] = limit(v.parameter[i], -2, 2);
// 			else
// 				v.parameter[i] = limit(v.parameter[i], 0.5, 2.0);

		}
	}

	for(int i=0; i<min(point_count,999); i++)
	{
		printf("I:%f,%f,%f(%f)", points[i][0], points[i][1], points[i][2], sqrt(points[i][0]*points[i][0]+points[i][1]*points[i][1]+points[i][2]*points[i][2]));
		printf("\n");

	}

	FILE * f = fopen("Z:\\test.csv", "wb");
	fprintf(f, "a1,a2,a3,o1,o2,o3,\r\n");

	for(int i=0; i<point_count; i++)
	{
		float a0 = (points[i][0] + v_initial.parameter[0]) * v_initial.parameter[3];
		float a1 = (points[i][1] + v_initial.parameter[1]) * v_initial.parameter[4];
		float a2 = (points[i][2] + v_initial.parameter[2]) * v_initial.parameter[5];

		float b0 = (points[i][0] + v.parameter[0]) * v.parameter[3];
		float b1 = (points[i][1] + v.parameter[1]) * v.parameter[4];
		float b2 = (points[i][2] + v.parameter[2]) * v.parameter[5];

// 		printf("O:%f,%f,%f(%f)", a0, a1, a2, sqrt(a0*a0+a1*a1+a2*a2));
// 		printf("\n");
		printf("OB:%f,%f,%f(%f)", b0, b1, b2, sqrt(b0*b0+b1*b1+b2*b2));
		printf("\n");
		fprintf(f, "%f,%f,%f,%f,%f,%f\r\n", points[i][0], points[i][1], points[i][2], b0, b1, b2);
	}

	fclose(f);

	printf("bias:");
	for(int i=0; i<3; i++)
	{
		v.parameter[i] *= 500;
		printf("%.2f,", v.parameter[i]);
	}

	printf("\nscale:");
	for(int i=0; i<3; i++)
		printf("%f,", v.parameter[i+3]);
	printf("\n");


	return 0;
}

int fitting_accel_matrix()
{

	for(int i=0; i<point_count; i++)
	{
		points[i][0] /= 2048;
		points[i][1] /= 2048;
		points[i][2] /= 2048;
	}

	func_vector v = 
	{
		1,0,0,
		0,1,0,
		0,0,1,
		0,0,0};

// 	func_vector v = 
// 	{
// 		0.998976, -0.006189, -0.015724,
// 		0.005724, 1.004099, -0.018982,
// 		0.016394, 0.020874, 0.987771,
// 
// 		-0.000405, -0.003841, -0.022524};

	const double step = 0.000001;

	double delta[15];
	double delta_length = 0;
	for(int j=0; j<100000; j++)
	{
		double result = func_accel_matrix(v);
		delta_length = 0;
		for(int i=0; i<12; i++)
		{
			func_vector v2 = v;
			v2.parameter[i] += step;

			delta[i] = func_accel_matrix(v2) - result;
			delta_length += delta[i] * delta[i];
		}

		if (delta_length <= 1e-25)
			break;

		delta_length = sqrt(delta_length);
		for(int i=0; i<12; i++)
		{
			delta[i] /= delta_length;
			v.parameter[i] -= step * delta[i];
		}
	}

	for(int i=0; i<point_count; i++)
	{
		printf("I:%f,%f,%f(%f)", points[i][0], points[i][1], points[i][2], sqrt(points[i][0]*points[i][0]+points[i][1]*points[i][1]+points[i][2]*points[i][2]));
		printf("\n");

	}

	for(int i=0; i<point_count; i++)
	{
		float a0 = (points[i][0] + v.parameter[0]) * v.parameter[3];
		float a1 = (points[i][1] + v.parameter[1]) * v.parameter[4];
		float a2 = (points[i][2] + v.parameter[2]) * v.parameter[5];

		a0 = (points[i][0]*v.parameter[0] + points[i][1]*v.parameter[1] + points[i][2]*v.parameter[2] + v.parameter[9]);
		a1 = (points[i][0]*v.parameter[3] + points[i][1]*v.parameter[4] + points[i][2]*v.parameter[5] + v.parameter[10]);
		a2 = (points[i][0]*v.parameter[6] + points[i][1]*v.parameter[7] + points[i][2]*v.parameter[8] + v.parameter[11]);

		printf("O:%f,%f,%f(%f)", a0, a1, a2, sqrt(a0*a0+a1*a1+a2*a2));
		printf("\n");
	}


	printf("coeff:\n");
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
			printf("%llf,", v.parameter[i*3+j]);
		printf("   %f\n", v.parameter[9+i]);
	}

	for(int i=0; i<point_count; i++)
	{
		points[i][0] *= 2048;
		points[i][1] *= 2048;
		points[i][2] *= 2048;
	}



	return 0;
}

int motion_detect_accel(sensor_data sensor, float time)
{
	float mpu6050_temperature = sensor.temperature1  / 340.0f + 36.53f;

	int mounted[3] = {sensor.accel[1], sensor.accel[0], -sensor.accel[2]};
	for(int i=0; i<3; i++)
		sensor.accel[i] = mounted[i];

	bool still = true;
	for(int i=0; i<3; i++)
	{
		if (abs(still_start_accel[i] - sensor.accel[i]) > 55)
			still = false;
	}

	if (still)
	{
		if (still_counter == 0)
		{
			for(int i=0; i<3; i++)
				still_accel_avg[i] = sensor.accel[i];
			still_start_time = time;
			still_temp_avg = mpu6050_temperature;

		}
		else
		{
			still_temp_avg += mpu6050_temperature;
			for(int i=0; i<3; i++)
				still_accel_avg[i] += sensor.accel[i];
		}

		still_counter ++;
	}
	else
	{
		if (time - still_start_time > 4 && still_counter > 10)
		{
			printf("STILL:");
			for(int i=0; i<3; i++)
				printf("%.1f,", still_accel_avg[i]/still_counter);
			printf("%.2f°C, %.2fs\n", still_temp_avg/still_counter, time - still_start_time);

			for(int i=0; i<3; i++)
				points[point_count][i] = still_accel_avg[i]/still_counter;
			points[point_count][3] = still_temp_avg/still_counter;

			point_count++;
		}

		for(int i=0; i<3; i++)
			still_start_accel[i] = sensor.accel[i];
		still_counter = 0;
		still_start_time = 999999;
	}

	return 0;
}

int motion_detect_accel_16405(double_sensor_data double_sensor, float time)
{
	bool still = true;
	for(int i=0; i<3; i++)
	{
		if (abs(still_start_accel[i] - double_sensor.acc2[i]) > 55)
			still = false;
	}

	if (still)
	{
		if (still_counter == 0)
		{
			for(int i=0; i<3; i++)
				still_accel_avg[i] = double_sensor.acc2[i];
			still_start_time = time;
			still_temp_avg = 0;

		}
		else
		{
			still_temp_avg += 0;
			for(int i=0; i<3; i++)
				still_accel_avg[i] += double_sensor.acc2[i];
		}

		still_counter ++;
	}
	else
	{
		if (time - still_start_time > 4 && still_counter > 10)
		{
			printf("STILL:");
			for(int i=0; i<3; i++)
				printf("%.1f,", still_accel_avg[i]/still_counter);
			printf("%.2f°C, %.2fs\n", still_temp_avg/still_counter, time - still_start_time);

			for(int i=0; i<3; i++)
				points[point_count][i] = still_accel_avg[i]/still_counter;
			points[point_count][3] = still_temp_avg/still_counter;

			point_count++;
		}

		for(int i=0; i<3; i++)
			still_start_accel[i] = double_sensor.acc2[i];
		still_counter = 0;
		still_start_time = 999999;
	}

	return 0;
}

int motion_detect_mag(sensor_data sensor, float time)
{
	float mpu6050_temperature = sensor.temperature1  / 340.0f + 36.53f;

	int mounted[3] = {sensor.mag[0], sensor.mag[2], sensor.mag[1]};
	for(int i=0; i<3; i++)
		sensor.accel[i] = mounted[i];

	static float last_data[3] = {0};
	float min_sum = 99999;
	for(int n=0; n<point_count; n++)
	{
		float sum = 0;
		for (int i=0; i<3; i++)
			sum += (points[n][i]-sensor.accel[i])*(points[n][i]-sensor.accel[i]);
		min_sum = min(min_sum, sum);
	}

	if (min_sum > 5600)
	{
		for(int i=0; i<3; i++)
			last_data[i] = points[point_count][i] = sensor.accel[i];
		point_count++;
	}

	return 0;
}

double func_3d(func_vector v)
{
	double sum = 0;
	for(int i=0; i<point_count; i++)
	{
		double v2[3] = 
		{
			(points[i][0]+v.parameter[0])*v.parameter[3],
			(points[i][1]+v.parameter[1])*v.parameter[4],
			(points[i][2]+v.parameter[2])*v.parameter[5],
		};
		double delta = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2] - 1;
		sum += delta * delta;
	}

	return sqrt(sum);
}

double func_accel_matrix(func_vector v)
{
	double sum = 0;

	for(int i=0; i<point_count; i++)
	{
		double v2[3] = 
		{
			(points[i][0]*v.parameter[0] + points[i][1]*v.parameter[1] + points[i][2]*v.parameter[2] + v.parameter[9]),
			(points[i][0]*v.parameter[3] + points[i][1]*v.parameter[4] + points[i][2]*v.parameter[5] + v.parameter[10]),
			(points[i][0]*v.parameter[6] + points[i][1]*v.parameter[7] + points[i][2]*v.parameter[8] + v.parameter[11]),
		};

		double delta = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2] - 1;
		sum += delta * delta;
	}

	return sum;
}


int tobin()
{
	char line[1024];
	FILE * f = fopen("Z:\\test.txt", "rb");;
	FILE *o = fopen("Z:\\o.bin", "wb");

	int last_gyro = 0;
	int temp;
	int gyro1;
	float gyro2;

	int v1,v2,v3,v4,v5,v6;

	while(fscanf(f, "%f%d%f%d%d%d%d", &temp, &gyro1, &gyro2, &v3, &v4, &v5, &v6) == 7)
	{
// 		if (gyro != last_gyro)
		{
// 			gyro1 = 65.5*gyro1/80.0;
			short v = v5;
			fwrite(&v, 1, sizeof(v), o);
			v = v6 *65.5/80;
			fwrite(&v, 1, sizeof(v), o);
		}

	}

	/*
	const int order = 1;
	double errorD_lpf[order] = {0};			// variable for high order low pass filter, [order]

	static const float lpf_RC = 1.0f/(2*PI * 5.0f);


	for(int i=0; i<500*2000; i++)
	{
		double interval = 0.002f;
		double alpha = interval / (interval + lpf_RC);

		float v = ((rand() & 0xff) << 8) | (rand()&0xff);
		v/=65535.0f;

		for(int j=0; j<order; j++)
			errorD_lpf[j] = errorD_lpf[j] * (1-alpha) + alpha * (j==0?v:errorD_lpf[j-1]);

		float v2 = errorD_lpf[order-1];
		fwrite(&v, 1, sizeof(v), o);
		fwrite(&v2, 1, sizeof(v2), o);

	}
	*/

	fclose(f);
	fclose(o);


	return 0;
}

float avg(float *data, int count)
{
	float o = 0;
	for(int i=0; i<count; i++)
		o += data[i];
	return o/count;
}

float std_dev(float *data, int count)
{
	float avg = ::avg(data, count);
	float sum = 0;
	for(int i=0; i<count; i++)
		sum += (data[i]-avg)*(data[i]-avg);
	return sqrt(sum/count);
}

int allan_variance(float *input, int count, float *out)
{
	int max_bin_size = count/5;

#pragma omp parallel for
	for(int bin_size = 2; bin_size<max_bin_size; bin_size+=5)
	{
		float *tmp = new float[count/2];
		int bin_count = count / bin_size;

		for(int i=0; i<bin_count; i++)
		{
			tmp[i] = 0;
			for(int j=0; j<bin_size; j++)
				tmp[i] += input[bin_size*i+j];
			tmp[i] /= bin_size;
		}

		out[bin_size-2] = std_dev(tmp, bin_count);

		if (bin_size % 8 == 0)
			printf("%d/%d\n", bin_size, max_bin_size);
		delete [] tmp;
	}


	return max_bin_size;
}


float sqr(float a, float b)
{
	return sqrt(a*a+b*b);
}

int main(int argc, char **argv)
{
// 	sanity_test();
	int ssize = sizeof(quadcopter_data2);
	tobin();

	int size =  sizeof(rf_data);
	int gyros_counter = 0;
	float *gyros[3];
	for(int i=0; i<3; i++)
		gyros[i] = new float[1024*10240];


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
		estimator.update_gps(ii++, ii++, 2.0, (int64_t)i*1000000/333);
		estimator2.update_gps(ii++, ii++, 2.0, (int64_t)i*1000000/333);

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
	adv_sensor_data adv_sensor[3] = {0};
	pos_controller_data posc_data1;
	pos_controller_data2 posc_data2;
	double_sensor_data double_sensor = {0};
	px4_frame px4flow = {0};
	float flow_pos[2] = {0};
	float Rot_matrix[9];
	float euler[3];


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
	int64_t time;
	while (fread(&rf, 1, 32, f) == 32)
	{
		time = rf.time & ~TAG_MASK;
		if ((rf.time & TAG_MASK) ==  TAG_IMU_DATA)
			imu = rf.data.imu;
		else if ((rf.time & TAG_MASK) ==  TAG_ADV_SENSOR_DATA1)
			adv_sensor[0] = rf.data.adv_sensor;
		else if ((rf.time & TAG_MASK) ==  TAG_ADV_SENSOR_DATA2)
			adv_sensor[1] = rf.data.adv_sensor;
		else if ((rf.time & TAG_MASK) ==  TAG_ADV_SENSOR_DATA3)
			adv_sensor[2] = rf.data.adv_sensor;
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
 		else if ((rf.time & TAG_MASK) ==  TAG_GPS_DATA_V3)
 			;//do nothing
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
			//motion_detect_accel(sensor, time/1000000.0f);
			motion_detect_mag(sensor, time/1000000.0f);

			static int imu_counter = 0;
			static int ltime = -1;
			if (time > 3600000000)
			{
				if (imu_counter ++ % 5 == 0)
				{
					gyros[0][gyros_counter] = sensor.gyro[0] * 500.0f / 32767;
					gyros[1][gyros_counter] = sensor.gyro[1] * 500.0f / 32767;
					gyros[2][gyros_counter] = sensor.gyro[2] / 80.0f;

// 					gyros[0][gyros_counter] = sensor.accel[0] / 2048.0f;
// 					gyros[1][gyros_counter] = sensor.accel[1] / 2048.0f;
// 					gyros[2][gyros_counter] = sensor.accel[2] / 2048.0f;

					gyros_counter ++;
				}
			}

			float dt = (time - ltime)/1000000.0f;
			ltime = time;

			// AHRS offline testing
			if (dt > 0 && dt < 1)
			{
				float alpha = dt / (dt + 1.0f/(2*3.1415926 * 2.5f));
				vector newacc = {sensor.accel[1], sensor.accel[0], -sensor.accel[2]};
				vector mag = {(sensor.mag[0]), (sensor.mag[2]), (sensor.mag[1])};
				float GYRO_SCALE = 500.0f * PI / 180 / 32767;		// full scale: +/-2000 deg/s  +/-32767
				vector gyro = {-sensor.gyro[0], sensor.gyro[1], 0};
				vector_multiply(&gyro, GYRO_SCALE);
				vector_multiply(&newacc, alpha);
				vector_multiply(&acc, 1-alpha);
				vector_add(&acc, &newacc);

				vector acc_norm = acc;
				vector_multiply(&acc_norm, 9.8065f/2048.0f);
				float pix_acc[3] = {acc_norm.V.y, -acc_norm.V.x, -acc_norm.V.z};
				float pix_acc_g = vector_length(&acc)/ 2048;
				float pix_acc2[3] = {pix_acc[0], pix_acc[1], pix_acc[2]};
				if (pix_acc_g > 1.15f || pix_acc_g < 0.85f)
					pix_acc2[0] = pix_acc2[1] = pix_acc2[2] = 0;

				float pix_mag[3] = {0};
				NonlinearSO3AHRSupdate(
					gyro.array[0], gyro.array[1], -gyro.array[2],
					// 		0,0,0,
					pix_acc2[0], pix_acc2[1], pix_acc2[2],
					pix_mag[0], pix_mag[1], pix_mag[2], 
					0.1f, 0.01f, 0.1f, 0.001f, dt);

				float q0q0 = q0*q0;
				float q1q1 = q1*q1;
				float q2q2 = q2*q2;
				float q3q3 = q3*q3;


				Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
				Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
				Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
				Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
				Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
				Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
				Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
				Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
				Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

// 				memcpy(&NED2BODY, Rot_matrix, sizeof(float)*9);
// 				inverse_matrix3x3(NED2BODY, BODY2NED);
				euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	//! Roll
				euler[1] = -asinf(Rot_matrix[2]);	//! Pitch
				euler[2] = atan2f(-Rot_matrix[1], -Rot_matrix[0]);		//! Yaw, 0 = north, PI/-PI = south, PI/2 = east, -PI/2 = west

			}

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
		else if ((rf.time & TAG_MASK) ==  TAG_PX4FLOW_DATA)
		{

// 			if (rf.data.px4flow.ground_distance > 300 && rf.data.px4flow.ground_distance < 30000)
			{
				px4flow = rf.data.px4flow;
				

				static int64_t last_integral_time = 0;

				float dt = (time - last_integral_time) / 1000000.0f;

				if (dt > 0 && dt < 1)
				{
					flow_pos[0] += px4flow.flow_comp_m_x / 1000.0f * dt;
					flow_pos[1] += px4flow.flow_comp_m_y / 1000.0f * dt;
				}
				else
				{
					printf("..");
				}

				last_integral_time = time;
			}
		}
		else if ((rf.time & TAG_MASK) ==  TAG_POS_CONTROLLER_DATA1)
			posc_data1 = rf.data.pos_controller;
		else if ((rf.time & TAG_MASK) ==  TAG_POS_CONTROLLER_DATA2)
			posc_data2 = rf.data.pos_controller2;
		else if ((rf.time & TAG_MASK) ==  TAG_DOUBLE_SENSOR_DATA)
		{
			double_sensor = rf.data.double_sensor;
 			motion_detect_accel_16405(double_sensor, time/1000000.0f);
		}
		else if ((rf.time & TAG_MASK) ==  TAG_PPM_DATA)
			ppm = rf.data.ppm;
		else if ((rf.time & TAG_MASK) ==  TAG_CTRL_DATA)
			;// ignore controll data packets
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
				estimator.update_gps(gps_sample.latitude*double(COORDTIMES/TIMES), gps_sample.longtitude*double(COORDTIMES/TIMES), 2.0f, time);
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
		float yaw_est = atan2(estMagGyro.V.x * estAccGyro16.V.z - estMagGyro.V.z * estAccGyro16.V.x ,
			((estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y  - estMagGyro.V.y * xxzz )/G);
		float yaw_gyro = -atan2(estGyro.V.x * estAccGyro16.V.z - estGyro.V.z * estAccGyro16.V.x,
			((estGyro.V.x * estAccGyro16.V.x + estGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y - estGyro.V.y * xxzz )/G);
		float roll_acc = radian_add(atan2(acc.V.x, acc.V.z), PI);
		float pitch_acc = atan2(acc.V.y, (acc.V.z > 0 ? 1 : -1) *sqrt(acc.V.x*acc.V.x + acc.V.z * acc.V.z));
		pitch_acc = radian_add(pitch_acc, PI);


		static double ground_pressure = -1012.9;
		static double ground_temperature = 34.28;

		if (imu.pressure < 10000)
		{
			int pressure = imu.temperature + 0x10000;
			int temp = imu.pressure;

			imu.pressure = pressure;
			imu.temperature = temp;
		}

		if ((rf.time & TAG_MASK) !=  TAG_SENSOR_DATA)
			continue;

		double pressure = (imu.pressure)/100.0;
// 		pressure = 15000 + (adv_sensor[0].data[0]-adv_sensor[0].data[3]*0.05f)/(0.9f*adv_sensor[0].data[3])*100000.0f;
// 		pressure /= 100.0f;

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
		float gyro = (adv_sensor[0].data[1] - 2.50f)/0.006f;
		float a1 = (adv_sensor[1].data[5]-2.50f);
		float a2 = (adv_sensor[2].data[0]-2.50f);
		float vcc = adv_sensor[0].data[3];
		float a3 = (adv_sensor[1].data[4]-vcc/2)*5.0/vcc/0.312f;
		float a4 = (adv_sensor[1].data[4]-2.50)/0.312f;
		float temperature_620 = (adv_sensor[0].data[2] - 2.50f)/0.009f + 25;
		float g_16405 = sqrtf(double_sensor.acc2[0]*double_sensor.acc2[0]+double_sensor.acc2[1]*double_sensor.acc2[1]+double_sensor.acc2[2]*double_sensor.acc2[2]) / 2048.0f;
		int avg_output = (ppm.out[0]+ ppm.out[1]+ ppm.out[2]+ ppm.out[3])/4;

		static float pos = -9999;
		static int time_start = 0;
		if (time > 134000000)
		{
			if (pos != -9999)
			{
				float dt = (time - time_start)/1000000.0f;
				time_start = time;
				pos += sensor.gyro[2] * 100.0f / 80.0f * dt;
			}
			else
			{
				time_start = time;
				pos = quad.angle_pos[0];
			}
		}
		float latitude_to_meter = (40007000.0f/COORDTIMES/360);
//   		if (time > 45000000 && time < 55000000)
// 		if (time > 70000000 && time < 80000000)
// 		if (abs(a1) < 0.10 && abs(a2) < 0.10)
  		if (n++ %5 == 0)
//  		if (abs(adv_sensor[0].data[3]-5)<0.2)
//  		if ((time > 13500000 && time < 17500000) || (time > 25500000 && time < 30000000))
 		fprintf(fo, "%.4f,%.5f,%.5f,%2f,%f,"
					"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
					"%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d,%d,%d,%d\r\n",
				float(time/1000000.0f), float(quad3.altitude_target/100.0f), float(quad2.altitude_baro_raw/100.0f), float(quad3.altitude/100.0f), float(quad2.altitude_kalman/100.0f),
 				quad2.accel_z, quad2.accel_z_kalman, sensor.accel[2], px4flow.qual, pilot.error[0], pilot.error[1], pilot.error[2], pilot2.I[1], pilot2.D[0],
				roll*180/PI, pitch*180/PI, roll_acc * 180 / PI, pitch_acc*180/PI, pilot.target[0]/100.0, pilot.target[2]/100.0, 
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
		fflush(fo);

// 		if ((rf.time & TAG_MASK) ==  TAG_QUADCOPTER_DATA || (rf.time & TAG_MASK) ==  TAG_GPS_DATA || (rf.time & TAG_MASK) ==  TAG_PILOT_DATA || (rf.time & TAG_MASK) ==  TAG_PILOT_DATA2)
		{
//  			if (
// 				1 &&
// 				pilot.fly_mode == quadcopter
// 				gps.fix>1 && gps.longitude > 0 && gps.latitude > 0
//  				)
//  			if (time > 200000000 && time < 300000000)
// 			if (m++ %3 == 0 && quad3.ultrasonic != 0xffff)
// 			if (home_set)
// 			if (time > 220000000 && time < 230000000)
// 			if (time > 50000000 && time < 75000000)
//  			if (m++ %15 == 0)
			{
				float yaw = gps.direction * PI / 180;
				if (yaw > PI)
					yaw -= 2*PI;

				float speed_north = cos(yaw) * gps.speed;
				float speed_east = sin(yaw) * gps.speed;

				fprintf(gpso, "%.4f", float(time/1000000.0f));
				fprintf(gpso, ",%d,%d,%d,%d", quad.angle_pos[1], quad.angle_target[1], quad.speed[1], quad.speed_target[1]);
				fprintf(gpso, ",%d,%d,%d,%d,%f,%f,%f,", quad.angle_pos[0], quad.angle_target[0],quad.speed[0], quad.speed_target[0], quad2.gyro_bias[0]/100.0f, quad2.gyro_bias[1]/100.0f, quad2.gyro_bias[2]/100.0f);
				fprintf(gpso, "%.1f/%d/%d/%d,%f,%f,%f,%f,%f", gps.DOP[1]/100.0f, gps.satelite_in_use, gps.fix, gps.direction, quad2.accel_z_kalman/100.0f, ned[1].accel_NED2[2]/1000.0f, quad2.altitude_baro_raw/100.0f, quad3.altitude/100.00f, quad3.altitude_target/100.0f);
				fprintf(gpso, "\r\n");

				fflush(gpso);
			}
		}
	}

	fclose(gyrof);
	fclose(fo);
	fclose(f);
	fclose(gpso);

	memset(&sensor, 0, sizeof(sensor));
// 	motion_detect_mag(sensor, time/1000000.0f);

	/*
	printf("allan variance\n");

	float *allan[3];
	int count = 0;
	for(int i=0; i<3; i++)
	{
		allan[i] = new float[1024*10240];
		count = allan_variance(gyros[i], gyros_counter, allan[i]);
	}

	printf("allan variance OK\n");

	FILE * allanf = fopen("allan.csv", "wb");
	for(int i=0; i<count; i+=5)
		fprintf(allanf, "%f,%f,%f,%f\r\n", i*0.01f, allan[0][i], allan[1][i], allan[2][i]);

	fclose(allanf);
	*/

// 	printf("max time delta: %d\n", max_time_delta);

// 	fitting_accel_matrix();
// 	fitting_mag_3d();
// 	fitting_accel_3d();

	return 0;
}