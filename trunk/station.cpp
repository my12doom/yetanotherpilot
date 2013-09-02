#include <stdio.h>
#include <stm32f10x.h>
#include <misc.h>
#include <stm32f10x_fsmc.h>
#include <math.h>
#include "RFData.h"


extern "C"
{
#include "common/vector.h"
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
	
	uint8_t tmp[20];
	uint8_t tmp2[20];
	uint8_t tmp3[20];
	uint8_t tmp4[20];
	uint8_t yawstr[50];

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
	imu_data imu = {0};
	sensor_data sensor = {0};
	int type = 3;
	while(1)
	{
		int result = NRF_Rx_Dat((u8*)&recv);
		int ref = 0;
		if (result & RX_OK)
		{
			int64_t time = recv.time & (~TAG_MASK);
			if ((recv.time & TAG_MASK) == TAG_SENSOR_DATA)
			{
				type = 1;
				t1 ++;
				
				sensor = recv.data.sensor;
			}
			else if ((recv.time & TAG_MASK) == TAG_IMU_DATA)
			{
				type =2;
				t2 ++;
				imu = recv.data.imu;
			}
			else
			{
				// unknown data
			}
			
			t4++;
			ref = 1;

			sprintf((char*)tmp, "%dms, t123=%d,%d,%d,%d", int(time/1000), t1, t2, t4);
			if (t4 %100 == 0)
				printf("%dms, t123=%d,%d,%d\r\n", int(time/1000), t1, t2, t4);
			
			for(int i=0; i<8; i++)
			{
				sprintf((char*)tmp+2*i, "%02x", ((u8*)&recv)[i]);
				sprintf((char*)tmp2+2*i, "%02x", ((u8*)&recv)[i+8]);
				sprintf((char*)tmp3+2*i, "%02x", ((u8*)&recv)[i+16]);
				sprintf((char*)tmp4+2*i, "%02x", ((u8*)&recv)[i+24]);
			}
		}
		else
		{
			//sprintf((char*)tmp, "time=%dms, type=%d", 0, 999);
		}
		
		
		if (ref && (t4%20==0))
		{
			vector estAccGyro = {imu.estAccGyro[0], imu.estAccGyro[1], imu.estAccGyro[2]};
			vector estMagGyro = {imu.estMagGyro[0], imu.estMagGyro[1], imu.estMagGyro[2]};
			vector estAccGyro16 = estAccGyro;
			vector_divide(&estAccGyro16, 16);
			float xxzz = (estAccGyro16.V.x*estAccGyro16.V.x + estAccGyro16.V.z * estAccGyro16.V.z);
			float G = sqrt(xxzz+estAccGyro16.V.y*estAccGyro16.V.y);
			float yaw_est = atan2(estMagGyro.V.z * estAccGyro16.V.x - estMagGyro.V.x * estAccGyro16.V.z,
				(estMagGyro.V.y * xxzz - (estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);
			
			sprintf((char*)yawstr, "%d,%d,%d,  %f     ", sensor.accel[0], sensor.accel[1], sensor.accel[2], yaw_est * 180 / 3.1415926);
			//sprintf((char*)yawstr, "%f", yaw_est * 180 / 3.1415926);
			ARC_LCD_Clear(0);
			ARC_LCD_ShowString(0, 0, tmp);
			ARC_LCD_ShowString(0, 16, tmp2);
			ARC_LCD_ShowString(0, 32, tmp3);
			ARC_LCD_ShowString(0, 48, tmp4);
			ARC_LCD_ShowString(0, 64, yawstr);
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
