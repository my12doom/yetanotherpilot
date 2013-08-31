#include <stdio.h>
#include <stm32f10x.h>
#include <misc.h>
#include <stm32f10x_fsmc.h>
#include "RFData.h"

extern "C"
{
#include "common/printf.h"
#include "common/NRF24L01.h"
#include "common/common.h"
#include "fat/ff.h"
#include "fat/sdcard.h"
#include "stm32f10x_sdio.h"
}

SD_Error SD_InitAndConfig(void)
{
	SD_CardInfo SDCardInfo;
	SD_Error Status ;
  Status = SD_Init();

  if (Status == SD_OK)
  {
    Status = SD_GetCardInfo(&SDCardInfo);
  }
  if (Status == SD_OK)
  {
    Status = SD_SelectDeselect((u32) (SDCardInfo.RCA << 16));
  }
  if (Status == SD_OK)
  {
    Status = SD_EnableWideBusOperation(SDIO_BusWide_4b);
  }
  if (Status == SD_OK)
  {
    Status = SD_SetDeviceMode(SD_DMA_MODE);
  }
	return Status;
}

#include "LCD/arc_lcd.h"
uint16_t buf[512];

int main(void)
{

	// Basic Initialization
	SysTick_Config(720);
	printf_init();
	SPI_NRF_Init();
	int nrf = NRF_Check();
	printf("NRF_Check() = %d\r\n", nrf);
	NRF_RX_Mode();
	init_timer();
	
	
	ARC_LCD_Init();
	ARC_LCD_Clear(0);
	uint8_t tmp[32];
	sprintf((char*)tmp, "Æô¶¯ok,ºÄÊ±%dÎ¢Ãë, NRF=%d", (int)getus(), nrf);
	printf("%s\r\n", tmp);
	ARC_LCD_Clear(0);
	ARC_LCD_ShowString(0, 0, tmp);
	delayms(1000);
		
	
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	SD_InitAndConfig();
	
	FATFS fs;
	FIL file;
	UINT done;
	disk_initialize(0);
	FRESULT res = f_mount(0, &fs);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*
	res = f_open(&file, "1055.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	res = f_lseek(&file,file.fsize);
	f_printf (&file,"%s, %d\r\n", "Hello Station!", (int)getus());
	f_write(&file, "\x12\x34\x56\x78", 4, &done);	
	f_close(&file); 
	*/
	
	
	int name = 0;
	rf_data recv;
	int t1 = 0;
	int t2 = 0;
	int t4=0;
	while(1)
	{
		int result = NRF_Rx_Dat((u8*)&recv);
		int ref = 0;
		if (result & RX_OK)
		{
			int64_t time = recv.time & (~TAG_MASK);
			int type = 3;
			if ((recv.time & TAG_MASK) == TAG_SENSOR_DATA)
			{
				type = 1;
				t1 ++;
			}
			else if ((recv.time & TAG_MASK) == TAG_IMU_DATA)
			{
				type =2;
				t2 ++;
			}
			else
			{
				// unknown data
				t4++;
			}
			
			ref = 1;

			sprintf((char*)tmp, "%dms, t123=%d,%d,%d,%d", int(time/1000), t1, t2, t4);
			if (t4 %100 == 0)
				printf("%dms, t123=%d,%d,%d\r\n", int(time/1000), t1, t2, t4);
		}
		else
		{
			//sprintf((char*)tmp, "time=%dms, type=%d", 0, 999);
		}
		
		
		if (ref)
		{
			ARC_LCD_Clear(0);
			ARC_LCD_ShowString(0, 0, tmp);
		}
		
		//printf("\rt1=%d, t2=%d", t1, t2);
		/*
		name = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
		res = f_open(&file, name==0?"1.bmp":"2.bmp", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		//ARC_LCD_Clear(0);
			ARC_LCD_SetCursor(0, 0);
		for(int y=0; y<320; y++)
		{
			f_read(&file, buf, 480, &done);
			ARC_LCD_WriteGRAM(buf, 240);
		}
		f_close(&file);
		*/
		
	}
	f_mount(0, NULL);
	
		
	while (1)
	{
	}
}
