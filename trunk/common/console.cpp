#include "../mcu.h"
#include "console.h"
#include "PPM.h"
#include "printf.h"
#include "vector.h"
#include <stdlib.h>
#include "build.h"
#include <string.h>
#include "param.h"
#include "space.h"

extern volatile vector imu_statics[2][4];		//	[accel, gyro][min, current, max, avg]
extern volatile int avg_count;
extern float mpu6050_temperature;


#define SIGNATURE_ADDRESS 0x0800E800

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
	#ifdef LITE
	else if (strstr(line, "id") == line)
	{
		strcpy(out, "id:");
		memcpy(out+3, (void*)(0x1ffff7e8), 12);
		memcpy(out+15, (void*)SIGNATURE_ADDRESS, 128);
		out[15+128] = '\n';

		return 144;
	}
	else if (strstr(line, "sig1:") == line || strstr(line, "sig2:") == line || strstr(line, "sig3:") == line)
	{
		if (line[3] == '1')
			FLASH_ErasePage(SIGNATURE_ADDRESS);

		int p = line[3] - '1';
		for(int i=0; i<48; i+=4)
			FLASH_ProgramWord(SIGNATURE_ADDRESS+i+48*p, *(unsigned int*)(line+5+i));

		strcpy(out, "ok\n");
		return 3;
	}
	#endif
	else if (strstr(line, "resetmc") == line)
	{
		space_init(true);
		NVIC_SystemReset();
	}




	return 0;
}