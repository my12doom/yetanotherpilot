#include "mcu.h"
#include <stdio.h>
#include "common/timer.h"
#include "common/printf.h"
#include "common/space.h"
#include "common/ads1258.h"



int main()
{
	init_timer();
	printf_init();
	space_init();
	delayms(400);
	ads1258_init();
	
	int i = 0;

	while(1)
	{
		
		printf("HelloWorld %d!       \r\n", i++);
		
		for(i=0; i<10; i++)
		{
		delayms(50);
			uint8_t data;

		ads1258_read_registers(i, 1, &data);


		ERROR("reg(%d)=0x%02x\n", i, data);
		}
		
		delayms(1000);
	}
}
