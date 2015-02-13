#include <stdio.h>
#include <stm32f10x.h>
#include <math.h>
#include "RFData.h"
#include "common/printf.h"
#include "common/NRF24L01.h"
#include "common/config.h"
#include <string.h>
extern "C" {
#include "common/MMC_SD.h"
#include "fat/ff.h"
#include "fat/diskio.h"
}

char buf1[64];
char buf2[64];

int escape(const char *string, int size, char*escaped)
{
	int i,j;
	for(i=0,j=0; i<size; i++,j++)
	{
		escaped[j] = string[i];
		if (string[i] == '\r')
			escaped[j++] = '\r';
	}
	return j;
}

int unescape(const char *escaped, char *string)
{
	return 0;
}

FRESULT scan_files (
    char* path        /* Start node to be scanned (also used as work area) */
)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;   /* This function is assuming non-Unicode cfg. */
#if _USE_LFN
    static char lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif


    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                sprintf(&path[i], "/%s", fn);
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
                printf("%s/%s\n", path, fn);
            }
        }
        //f_closedir(&dir)
    }

    return res;
}
FRESULT res;
int main()
{
	SysTick_Config(720);
	printf_init();
	NRF_Init();
	init_timer();
	delayms(200);
	int tf = MMC_SD_Init();
	printf("MMC=%d\r\n", tf);
	//NRF_RX_Mode();
	
	uint32_t capacity = MMC_SD_ReadCapacity();
	printf("capacity = %u", capacity);
	
	int us = getus();
	int packet = 0;
	
	// sd test
	FATFS fs;
	FIL file;
	UINT done;
	disk_initialize(0);
	res = f_mount(&fs, "", 0);
	char path[1024] = {0};
	res = f_mkfs(path, 0, 0);
	res = scan_files(path);
	
	// file read write test
	char filename[255];
	strcpy(filename, "test3.txt");
	res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	res = f_write(&file, path, 256, &done);
	res = f_close(&file);
	
	for(int i=0; i<1024;i++)
		path[i] = i;
	
	res = f_open(&file, filename, FA_OPEN_EXISTING | FA_WRITE | FA_READ);
	res = f_read(&file, path, 256, &done);
	res = f_close(&file);
	
	for(int i=0; i<1024;i++)
		printf("%d,", path[i]);
	
	while(1)
	{
		int result = NRF_Rx_Dat((uint8_t*)buf1);
		
		if (result & RX_OK)
		{
			int size = escape(buf1, 32, buf2);
			for(int i=0; i<size; i++)
			{
				USART_SendData(USART1, (unsigned char) buf2[i]);
				while (!(USART1->SR & USART_FLAG_TXE));
			}
			USART_SendData(USART1, (unsigned char) '\r');
			while (!(USART1->SR & USART_FLAG_TXE));
			USART_SendData(USART1, (unsigned char) '\n');
			while (!(USART1->SR & USART_FLAG_TXE));
			//printf("packet!, escaped size = %d\r\n", size);
			packet ++;
		}
		
		if (getus() - us > 1000000)
		{
			//printf("packet speed:%d\r\n", packet);
			packet=0;
			us = getus();
		}
		
		//printf("loop!\r\n");
		//delayms(1000);
	}
}
