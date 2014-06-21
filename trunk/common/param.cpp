#include "param.h"
#include "../mcu.h"
#include <stdlib.h>
#include "build.h"
#include <string.h>
#include "printf.h"

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
	for(int i=0; i<sizeof(all_params[0]); i+=2)
		EE_WriteVariable(VirtAddVarTab[0]+ i/2 + EEPROM_CONFIG + pos* sizeof(all_params[0]), *(uint16_t*)(((uint8_t*)&all_params[pos])+i));

	if (pos == all_param_count-1)
	{
		strncpy(all_params[all_param_count].fourcc, PARAM_ENDL, 4);
		for(int i=0; i<sizeof(all_params[0]); i+=2)
			EE_WriteVariable(VirtAddVarTab[0]+ i/2 + EEPROM_CONFIG + all_param_count* sizeof(all_params[0]), *(uint16_t*)(((uint8_t*)&all_params[all_param_count])+i));
	}
}
void param::init_all()
{
	printf_init();
	FLASH_Unlock();
#ifdef STM32F4
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
#endif
	EE_Init();

	ERROR("all_param_count=%d\n", all_param_count);

	all_param_count = 0;
	int pos = EEPROM_CONFIG;
	while (all_param_count<MAX_PARAM_COUNT)
	{
		for(int i=0; i<sizeof(all_params[0]); i+=2)
		{
			if (0 != EE_ReadVariable(VirtAddVarTab[0]+i/2+pos, (uint16_t*)(((uint8_t*)&all_params[all_param_count])+i)))
				goto finish;
		}

		if (strncmp(all_params[all_param_count].fourcc, PARAM_ENDL, 4) == 0)
			break;

		pos += 8;
		all_param_count++;
	}
	
finish:
	ERROR("all_param_count=%d\n", all_param_count);
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


extern "C" int parse_command_line(const char *line, char *out)
{

	float roll,  pitch, yaw;
	float p,i,d;
	int log_level;

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
		}
		
	}




	return 0;
}


/*

	if (strstr(line, "pid3") == line && sscanf(line, "pid3 %f %f %f", &p, &i, &d) == 3)
	{
		printf("new pid2 roll & pitch :%f,%f,%f\r\n", p, i, d);
		// 		pid_factor2[0][0] = p;
		// 		pid_factor2[0][1] = i;
		// 		pid_factor2[0][2] = d;
		// 		pid_factor2[1][0] = p;
		// 		pid_factor2[1][1] = i;
		// 		pid_factor2[1][2] = d;
	}
	else if (strstr(line, "pid4") == line && sscanf(line, "pid4 %f %f %f", &p, &i, &d) == 3)
	{
		printf("new pid2 yaw :%f,%f,%f\r\n", p, i, d);
		// 		pid_factor2[2][0] = p;
		// 		pid_factor2[2][1] = i;
		// 		pid_factor2[2][2] = d;
	}
	else if (strstr(line, "pid2") == line && sscanf(line, "pid2 %f %f %f", &p, &i, &d) == 3)
	{
		printf("new pid yaw:%f,%f,%f\r\n", p, i, d);
		// 		pid_factor[2][0] = p;
		// 		pid_factor[2][1] = i;
		// 		pid_factor[2][2] = d;
	}
	else if (strstr(line, "pid") == line && sscanf(line, "pid %f %f %f", &p, &i, &d) == 3)
	{
		printf("new pid roll&pitch:%f,%f,%f\r\n", p, i, d);
		// 		pid_factor[0][0] = p;
		// 		pid_factor[0][1] = i;
		// 		pid_factor[0][2] = d;
		// 		pid_factor[1][0] = p;
		// 		pid_factor[1][1] = i;
		// 		pid_factor[1][2] = d;
	}
	else if (strstr(line, "trim") == line && sscanf(line, "trim %f %f %f", &roll, &pitch, &yaw) == 3)
	{
		printf("new trim:%f,%f,%f\r\n", roll, pitch, yaw);
#if QUADCOPTER == 1
		// 		quadcopter_trim[0] = roll;
		// 		quadcopter_trim[1] = pitch;
		// 		quadcopter_trim[2] = yaw;
#endif
	}
	else if (strstr(line, "log") == line && sscanf(line, "log %d", &log_level) == 1)
	{
		printf("new log level: 0x%x\r\n", log_level);
		LOG_LEVEL = log_level;
	}
	else
	{
		ERROR("unknown/invalid command: %s\r\n", line);
		return -1;
	}
*/
