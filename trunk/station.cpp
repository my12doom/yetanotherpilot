#include <stdio.h>
#include <stm32f10x.h>
#include <misc.h>
#include <stm32f10x_fsmc.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_rtc.h>
#include <stm32f10x_bkp.h>
#include <stm32f10x_pwr.h>
#include <math.h>
#include <time.h>
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


#define PI 3.14159265
FRESULT set_timestamp(char *filename);
void RTC_Init();


float radian_add(float a, float b)
{
	a += b;
	if (a>=PI)
		a -= 2*PI;
	if (a<-PI)
		a += 2*PI;
	
	return a;
}

time_t build_time;
struct tm current_time()
{
	time_t t = build_time + RTC_GetCounter();
	return *localtime (&t);
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
uint8_t yawstr[50];
imu_data imu = {0};
sensor_data sensor = {0};
pilot_data pilot = {0};

const char *mode_tbl[] = 
{
	"initializing",
	"manual",
	"acrobatic",
	"fly_by_wire",
	"rc_fail",
};

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
	
	RTC_Init();
	

	// SD CARD
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	SD_Error sd = SD_InitAndConfig();
	
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
	
	while(sd == SD_OK)
	{
		
		sprintf((char*)yawstr, "%d.dat", done ++);
		res = f_open(&file, (char*)yawstr, FA_CREATE_NEW | FA_WRITE | FA_READ);
		if (res == FR_OK)
			break;
	}
	
	set_timestamp((char*)yawstr);
		
	// LCD
	ARC_LCD_Init();
	ARC_LCD_Clear(0);
	sprintf((char*)tmp, "启动ok,耗时%d微秒, NRF=%d", (int)getus(), nrf);
	printf("%s\r\n", tmp);
	ARC_LCD_Clear(0);
	ARC_LCD_ShowString(0, 0, tmp);
	delayms(1000);
		
	
	rf_data recv;
	int packet = 0;
	int error_packet = 0;
	int type = 0x80000000;
	int packet_types = 0;
	int64_t last_update = 0;
	int64_t pilot_time = 0;
	int64_t last_packet_time = 0;
	while(1)
	{
		int result = NRF_Rx_Dat((u8*)&recv);
		if (result & RX_OK)
		{
			pilot_time = recv.time & (~TAG_MASK);
			if ((recv.time & TAG_MASK) == TAG_SENSOR_DATA)
			{
				type = 1;
				sensor = recv.data.sensor;
			}
			else if ((recv.time & TAG_MASK) == TAG_IMU_DATA)
			{
				type = 2;
				imu = recv.data.imu;
			}

			else if ((recv.time & TAG_MASK) == TAG_PILOT_DATA)
			{
				type = 4;
				pilot = recv.data.pilot;
			}
			else
			{
				// unknown data
				error_packet ++;
			}
			
			
			last_packet_time = getus();
			packet_types |= type;
			packet++;
			
			if (sd == SD_OK)
			{
				res = f_write(&file, &recv, 32, &done);
				if (res != FR_OK)
					sd = SD_ERROR;
				if ((packet % (512/32) == 0))
					f_sync(&file);
			}
		}
		
		
		if (getus()-last_update > 100000)
		{
			ARC_LCD_Clear(0);
			
			if (((packet_types&0xff) == 7))
			{
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
				float yaw_est = atan2(estMagGyro.V.z * estAccGyro16.V.x - estMagGyro.V.x * estAccGyro16.V.z,
					(estMagGyro.V.y * xxzz - (estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);
				float yaw_gyro = atan2(estGyro.V.z * estAccGyro16.V.x - estGyro.V.x * estAccGyro16.V.z,
					(estGyro.V.y * xxzz - (estGyro.V.x * estAccGyro16.V.x + estGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y )/G);

				
				sprintf((char*)yawstr, "RolPitYaw:%.2f %.2f %.2f", roll * 180 / 3.1415926, pitch * 180 / 3.1415926, yaw_est * 180 / 3.1415926);
				ARC_LCD_ShowString(0, 0, yawstr);
				sprintf((char*)yawstr, "mode:%s time:%.2f", mode_tbl[pilot.fly_mode], pilot_time / 1000000.0f);
				ARC_LCD_ShowString(0, 16, yawstr);
				sprintf((char*)yawstr, "error:%.2f %.2f %.2f", pilot.error[0]/100.f, pilot.error[1]/100.f, pilot.error[2]/100.f);
				ARC_LCD_ShowString(0, 32, yawstr);
				sprintf((char*)yawstr, "target:%.2f %.2f %.2f", pilot.target[0]/100.f, pilot.target[1]/100.f, pilot.target[2]/100.f);
				ARC_LCD_ShowString(0, 48, yawstr);
				sprintf((char*)yawstr, "altitude:%.2f", pilot.altitude/100.f);
				ARC_LCD_ShowString(0, 64, yawstr);
				sprintf((char*)yawstr, "RC:%d %d %d", pilot.rc[0], pilot.rc[1], pilot.rc[2]);
				ARC_LCD_ShowString(0, 80, yawstr);
				
			}
			
			if ((getus() - last_packet_time > 2000000))
			{
				ARC_LCD_ShowString(0, 110, "WARNING");
				sprintf((char*)yawstr, "NRF NOT RESPONDING %x", packet_types);
				ARC_LCD_ShowString(0, 126, yawstr);
			}
			if (sd != SD_OK)
			{
				ARC_LCD_ShowString(0, 142, "SDCARD ERROR");
			}
			
			int t = RTC_GetCounter();
			struct tm time = current_time();
			sprintf((char*)yawstr, "%d月%d日 %02d:%02d:%02d", time.tm_mon+1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
			ARC_LCD_ShowString(0, 300, yawstr);
			
			last_update = getus();
		}
	}	
}


FRESULT set_timestamp2 (char *obj, int year, int month, int mday, int hour, int min, int sec)
{
    FILINFO fno;

    fno.fdate = (WORD)(((year - 1980) * 512U) | month * 32U | mday);
    fno.ftime = (WORD)(hour * 2048U | min * 32U | sec / 2U);

    return f_utime(obj, &fno);
}
FRESULT set_timestamp(char *filename)
{
	struct tm time = current_time();
	return set_timestamp2(filename, time.tm_year-1900, time.tm_mon+1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
}

void RTC_Init()
{	
	const unsigned char MonthStr[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov","Dec"};
	unsigned char temp_str[4] = {0, 0, 0, 0}, i = 0;

	int year, month = 0, day, hour, minute, second;
	sscanf(__DATE__, "%s %2d %4d", temp_str, &day, &year);
	sscanf(__TIME__, "%2d:%2d:%2d", &hour, &minute, &second);
	for (i = 0; i < 12; i++)
	{
		if (temp_str[0] == MonthStr[i][0] && temp_str[1] == MonthStr[i][1] && temp_str[2] == MonthStr[i][2])
		{
			month = i+1;
			break;
		}
	}	
	printf("build time: %d-%d-%d %d-%02d-%02d\r\n", year, month, day, hour, minute, second);
	struct tm tm = {second, minute, hour, day, month-1, year-1900};
	build_time = mktime(&tm);
	
	if (BKP_ReadBackupRegister(BKP_DR1) != (build_time&0xffff))
	{
		// Backup data register value is not correct or not yet programmed (when
		// the first time the program is executed) 
		printf("\r\nThis is a RTC demo!\r\n");
		printf("\r\n\n RTC not yet configured....");
		
		// Enable PWR and BKP clocks 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
		
		// Allow access to BKP Domain 
		PWR_BackupAccessCmd(ENABLE);
		
		// Reset Backup Domain 
		BKP_DeInit();
		
		// Enable LSE 
		RCC_LSEConfig(RCC_LSE_ON);
		// Wait till LSE is ready 
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
		{}
		
		// Select LSE as RTC Clock Source 
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		
		// Enable RTC Clock 
		RCC_RTCCLKCmd(ENABLE);
		
		// Wait for RTC registers synchronization 
		RTC_WaitForSynchro();
		
		// Wait until last write operation on RTC registers has finished 
		RTC_WaitForLastTask();
		
		// Enable the RTC Second 
		RTC_ITConfig(RTC_IT_SEC, ENABLE);
		
		// Wait until last write operation on RTC registers has finished 
		RTC_WaitForLastTask();
		
		// Set RTC prescaler: set RTC period to 1sec 
		RTC_SetPrescaler(32767); // RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) 
		
		// Wait until last write operation on RTC registers has finished 
		RTC_WaitForLastTask();
		
				
		// Adjust time by values entred by the user on the hyperterminal 
		RTC_SetCounter(0);
		
		BKP_WriteBackupRegister(BKP_DR1, (build_time&0xffff));
	}
}
