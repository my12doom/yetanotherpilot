#include "param.h"
#include "../mcu.h"
#include <stdlib.h>
#include "build.h"
#include <string.h>
#include "printf.h"
#include "PPM.h"
#include "vector.h"
#include "space.h"

extern "C"
{
#ifdef STM32F1
#include "eeprom.h"
#endif
#ifdef STM32F4
#include "eepromF4.h"
#endif
}

#define PARAM_ENDL "\0\0\0\0"

int all_param_count = -1;
struct
{
	char fourcc[4];
	float v;
}all_params[MAX_PARAM_COUNT];

static int fourcclen(const char *p)
{
	for(int i=0; i<4; i++)
		if (p[i] == NULL)
			return i;
	return 4;
}

param::param()
{

}

void param::init(const char *fourcc, float default_value)
{
	// TODO : locks

	if (all_param_count<0)
		init_all();

	for(int i=0; i<all_param_count; i++)
	{
		if (strncmp(fourcc, all_params[i].fourcc, 4) == 0)
		{
			pv = &all_params[i].v;
			pos = i;
			return;
		}
	}

	pos = all_param_count;
	pv = &all_params[all_param_count].v;
	strncpy(all_params[all_param_count].fourcc, fourcc, 4);
	all_params[all_param_count].v = default_value;
	space_read(fourcc, fourcclen(fourcc), &all_params[all_param_count].v, 4, NULL);
	all_param_count++;

	
	save();
}

param::param(const char *fourcc, float default_value)
{
	init(fourcc, default_value);
}

param::~param()
{

}

void param::save()						// save to eeprom
{
	int res;
	float v;
	res = space_read(all_params[pos].fourcc, fourcclen(all_params[pos].fourcc), &v, 4, NULL);

	if (v != all_params[pos].v || res < 0)
		res = space_write(all_params[pos].fourcc, fourcclen(all_params[pos].fourcc), &all_params[pos].v, 4, NULL);
}
void param::init_all()
{
	space_init();
	all_param_count = 0;
}

float *param::find_param(const char *fourcc)
{
	for(int i=0; i<all_param_count; i++)
		if (strncmp(fourcc, all_params[i].fourcc, 4) == 0)
			return &all_params[i].v;
	return NULL;
}
float *param::enum_params(int pos)
{
	if (pos < 0 || pos >= all_param_count)
		return NULL;
	return &all_params[pos].v;
}


extern volatile vector imu_statics[2][4];		//	[accel, gyro][min, current, max, avg]
extern volatile int avg_count;
extern float mpu6050_temperature;

extern "C" int parse_command_line(const char *line, char *out)
{

	float roll,  pitch, yaw;
	//float p,i,d;
	int log_level;
	int i,j,k;

	if (line[0] == NULL)
		return 0;

	if (*line == '?')
	{
		float *v = param::find_param(line+1);
		if (v)
			sprintf(out, "%f\n", *v);
		else
			strcpy(out, "null\n");
		return strlen(out);
	}
	else if (char *p = (char*)strchr(line, '='))
	{
		*p = NULL;
		p++;

		float *v = param::find_param(line);
		if (v)
		{
			*v = atof(p);
			param(line, 0).save();
			strcpy(out, "ok\n");
			return 3;
		}
		else
		{
			strcpy(out, "fail\n");
			return 5;
		}
	}
	else if (strstr(line, "rcstates") == line)
	{
		int count = 0;
		count += sprintf(out, "rc:");
		for(i=0; i<6; i++)
		{
			volatile float p0 = ppm_static[i][0];
			volatile float p1 = g_ppm_input[i];
			volatile float p2 = ppm_static[i][1];
			count += sprintf(out+count, "%d,%d,%d,", (int)p0, (int)p1, (int)p2);
			if (count > 256)
				return count;
		}

		out[count++] = '\n';

		return count;
	}
	else if (strstr(line, "rcreset") == line)
	{
		PPM_reset_static();
		strcpy(out, "ok\n");
		return 3;
	}
	else if (strstr(line, "hello") == line)
	{
		strcpy(out, "yap1.0.0\n");
		return strlen(out);
	}
	else if (strstr(line, "imureset") == line)
	{
		avg_count = 0;
		for(i=0; i<2; i++)
		{
			for(j=0; j<3; j++)
			{
				imu_statics[i][0].array[j] = 99999;
				imu_statics[i][2].array[j] = -99999;
				imu_statics[i][3].array[j] = 0;
			}
		}
		strcpy(out, "ok\n");
		return 3;
	}
	else if (strstr(line, "imustates") == line)
	{
// 		printf("test:");
// 		printf("%.3f,%d,", mpu6050_temperature, avg_count);

		int count = 0;
		count += sprintf(out, "imu:");
		for(i=0; i<2; i++)
		{
			for(j=0; j<4; j++)
			{
				for(k=0; k<3; k++)
				{
					volatile int t = imu_statics[i][j].array[k] * 1000.0f / (j==3?avg_count:1);
					count += sprintf(out+count, "%d.%03d,",  t/1000,  t%1000);
					//if (count > 256)
					//	return count;
				}
			}
		}

		count += sprintf(out+count, "%d.%d,%d,", (int)mpu6050_temperature, ((int)(mpu6050_temperature*1000))%1000, avg_count);
		out[count++] = '\n';

		return count;
	}




	return 0;
}