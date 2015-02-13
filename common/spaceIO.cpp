#include "space.h"
#include <string.h>


static int min(int a, int b)
{
	if (a>b)
		return b;
	return a;
}


#ifdef WIN32

static unsigned char p[buffer_size];
int space_raw_read(int address, void *data, int size)
{
	int count = min(size, buffer_size - address);
	memcpy(data, p+address, count);

	return count;
}

int space_raw_write(int address, const void *data, int size)
{
	int count = min(size, buffer_size - address);
	memcpy(p+address, data, count);

	return count;
}

int space_raw_erase(int address)
{
	memset(p+ address / page_size * page_size, 0xff, page_size);

	return 0;
}

int space_raw_init()
{
	return 0;
}
#else

#include "mcu.h"

int space_raw_read(int address, void *data, int size)
{
	int count = min(size, buffer_size - address);
	memcpy(data, (char*)START_ADDRESS+address, count);

	return count;
}

int space_raw_write(int address, const void *data, int size)
{
	int count = min(size, buffer_size - address);
	char *p = (char*)data;

	for(int i=0; i<count/4*4; i+=4)
		FLASH_ProgramWord(START_ADDRESS+address+i, *(uint32_t*)(p+i));

	if (count %4)
	{
		uint32_t pending = 0xffffffff;
		memcpy(&pending, p+count/4*4, count%4);
		FLASH_ProgramWord(START_ADDRESS+address+count/4*4, pending);
	}

	#ifdef STM32F1
		FLASH_WaitForLastOperation(100);
	#endif
	
	#ifdef STM32F4
		FLASH_WaitForLastOperation();
	#endif
	
	return count;
}

int space_raw_erase(int address)
{
	#ifdef STM32F1
	FLASH_ErasePage(START_ADDRESS + address);
	#endif
	
	#ifdef STM32F4
		FLASH_Status res = FLASH_EraseSector(address >= 0x080C0000 + page_size ? FLASH_Sector_11 : FLASH_Sector_10, VoltageRange_3);
	#endif

	return 0;
}

int space_raw_init()
{
	FLASH_Unlock();

	return 0;
}

#endif
