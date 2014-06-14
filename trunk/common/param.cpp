#include "param.h"
#include "../mcu.h"
#include <string.h>
#include "config.h"

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
		if (memcmp(fourcc, all_params[i].fourcc, 4) == 0)
		{
			pv = &all_params[i].v;
			pos = i;
			return;
		}
	}

	pos = all_param_count;
	pv = &all_params[all_param_count].v;
	memcpy(all_params[all_param_count].fourcc, fourcc, 4);
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

param::operator float()
{
	return *pv;
}
float* param::operator& ()
{
	return pv;
}
float& param::operator= (float in)		// ram operation only
{
	return *pv = in;
}
void param::save()						// save to eeprom
{
	for(int i=0; i<sizeof(all_params[0]); i+=2)
		EE_WriteVariable(VirtAddVarTab[0]+ i/2 + EEPROM_CONFIG + pos* sizeof(all_params[0]), *(uint16_t*)(((uint8_t*)&all_params[pos])+i));

	if (pos == all_param_count-1)
	{
		memcpy(all_params[all_param_count].fourcc, PARAM_ENDL, 4);
		for(int i=0; i<sizeof(all_params[0]); i+=2)
			EE_WriteVariable(VirtAddVarTab[0]+ i/2 + EEPROM_CONFIG + all_param_count* sizeof(all_params[0]), *(uint16_t*)(((uint8_t*)&all_params[all_param_count])+i));
	}
}
void param::init_all()
{
	FLASH_Unlock();
#ifdef STM32F4
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
#endif
	EE_Init();

	all_param_count = 0;
	int pos = EEPROM_CONFIG;
	while (all_param_count<MAX_PARAM_COUNT)
	{
		for(int i=0; i<sizeof(all_params[0]); i+=2)
		{
			if (0 != EE_ReadVariable(VirtAddVarTab[0]+i/2+pos, (uint16_t*)(((uint8_t*)&all_params[all_param_count])+i)))
				return;
		}

		if (memcmp(all_params[all_param_count].fourcc, PARAM_ENDL, 4) == 0)
			return;

		pos += 8;
		all_param_count++;
	}
}