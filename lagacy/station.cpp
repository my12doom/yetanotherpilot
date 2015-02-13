#include <stdio.h>
#include "../mcu.h"

#include <math.h>
#include <time.h>
#include "RFData.h"
#include "common/vector.h"
#include "common/printf.h"
#include "common/NRF24L01.h"
#include "common/common.h"
#include <string.h>
extern "C"
{
#include "fat/ff.h"
#include "fat/diskio.h"
#include "usb_mass_storage/hw_config.h"
#include "usb_mass_storage/usb_init.h"
}

FRESULT set_timestamp(char *filename);
void RTC_Init();


time_t build_time;
struct tm current_time()
{
	time_t t = build_time + RTC_GetCounter();
	return *localtime (&t);
}

float NDEG2DEG(float ndeg)
{
	int degree = ndeg / 100;
	int minute = int(floor(ndeg)) % 100;	
	
	return degree + minute/60.0f + modf(ndeg, &ndeg)/60.0f;
}

extern "C" SD_Error SD_InitAndConfig(void);

int8_t keys[4];

#include "LCD/arc_lcd.h"
uint16_t buf[512];	
uint8_t tmp[20];
uint8_t yawstr[50];
char filename[20] = {0};
imu_data imu = {0};
sensor_data sensor = {0};
pilot_data pilot = {0};
pilot_data2 pilot2 = {0};
ppm_data ppm = {0};
gps_data gps = {0};
const char *mode_tbl[] = 
{
	"initializing",
	"manual",
	"acrobatic",
	"fly_by_wire",
	"quadcopter",
	"shutdown",
	"rc_fail",
	"acrobaticV",
};

SD_CardInfo SDCardInfo;
SD_Error Status ;
SD_Error SD_InitAndConfig(void)
{
  Status = SD_Init();

  if (Status == SD_OK)
  {
    Status = SD_GetCardInfo(&SDCardInfo);
  }
  if (Status == SD_OK)
  {
    Status = SD_SelectDeselect((uint32_t) (SDCardInfo.RCA << 16));
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

FRESULT res = FR_OK;
FRESULT scan_files (
    char* path        /* Start node to be scanned (also used as work area) */
)
{
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

int heartbeat = 1;
int heartbeat_count = 0;
int nrfresult = 1;
int nrfcb(int result, int user_data)
{
	if (!(result & TX_OK))
		return 0;

	heartbeat_count = (getus() - user_data)/1000;

	//heartbeat_count ++;
	heartbeat = 1;
	return 0;
}


#define LINE_HEIGHT 12
char path[256];
extern int tx_queue_count;
extern int tx_ok;
int main(void)
{	
	// Basic Initialization
	SysTick_Config(720);
	printf_init();	
	NRF_Init();
	int nrf = NRF_Check();
	printf("NRF_Check() = %d\r\n", nrf);
	NRF_RX_Mode();
	init_timer();
	
	RTC_Init();
	
	
	// NEMA LED init
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE, GPIO_Pin_0 | GPIO_Pin_2);
	
	// controll key
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

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
	FIL *file = NULL;
	UINT done;
	disk_initialize(0);
	res = f_mount(&fs, "", 0);
	res = scan_files(path);
		
	// sdcard speed test
	int minimum = 9999999;
	int maximum = 0;
	int average = 0;
	{
		char buf[32] = {1};
		FIL f2;
		res = f_open(&f2, "test.bin", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
		
		for(int i=0; i<1000; i++)
		{
			long us = getus();
			res = f_write(&f2, buf, 32, &done);
			us = getus() - us;
			minimum = (minimum < us) ? minimum : us;
			average += us;
			maximum = (maximum > us) ? maximum : us;
		}
		
		average /= 1000;
		f_close(&f2);		
	}

	// USB
	Set_System();
	SD_InitAndConfig();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
		
	// LCD
	ARC_LCD_Init();
	ARC_LCD_Clear(0);
	sprintf((char*)tmp, "Startup ok, %dus, NRF=%d", (int)getus(), nrf);
	printf("%s\r\n", tmp);
	ARC_LCD_Clear(0);
	ARC_LCD_ShowString(0, 0, tmp);
	sprintf((char*)tmp, "sd = %d-%d us, %d avg", minimum, maximum, average);
	printf("%s\r\n", tmp);
	ARC_LCD_ShowString(0, LINE_HEIGHT, tmp);
	delayms(1000);
		
	
	rf_data recv;
	int packet = 0;
	int error_packet = 0;
	int type = 0x80000000;
	int packet_types = 0;
	int64_t last_update = 0;
	int64_t pilot_time = 0;
	int64_t last_packet_time = 0;
	vector mag_zero;
	float mag_radius = -1;
	int packet_speed = 0;
	int packet_speed_counter = 0;
	int64_t packet_speed_time = getus();
	int64_t last_nema_parse = getus() - 2000000;
	
	
	NRF_Init();
	nrf = NRF_Check();
	printf("NRF_Check() = %d\r\n", nrf);
	
	while(1)
	{		
		NRF_RX_Mode();
		// read keys
		for(int i=0; i<4; i++)
			keys[i] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12<<i);

		int result = NRF_Rx_Dat((uint8_t*)&recv);
		if (getus() - packet_speed_time > 1000000)
		{
			packet_speed = packet_speed_counter;
			packet_speed_counter = 0;
			packet_speed_time = getus();
		}
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

			else if ((recv.time & TAG_MASK) == TAG_PILOT_DATA2)
			{
				type = 4;
				pilot2 = recv.data.pilot2;
			}
			else if ((recv.time & TAG_MASK) == TAG_CTRL_DATA)
			{
				type = 4;
				controll_data &controll = recv.data.controll;
				if (controll.cmd == CTRL_CMD_FEEDBACK && controll.reg == CTRL_REG_MAGNET )
				{
					mag_radius = controll.value/1000.0f;
					for(int i=0; i<3; i++)
						mag_zero.array[i] = controll.data[i]/1000.0f;
				}
			}


			else if ((recv.time & TAG_MASK) == TAG_PPM_DATA)
			{
				type = 5;
				ppm = recv.data.ppm;
			}

			else if ((recv.time & TAG_MASK) == TAG_GPS_DATA)
			{
				last_nema_parse = getus();
				type = 6;
				gps = recv.data.gps;
			}
			else
			{
				// unknown data
				error_packet ++;
			}
			
			
			last_packet_time = getus();
			packet_types |= type;
			packet++;
			packet_speed_counter++;
			
			if (sd == SD_OK)
			{
				if (file == NULL)
				{
					file = new FIL;
					while(sd == SD_OK)
					{
						
						sprintf(filename, "%d.dat", done ++);
						res = f_open(file, filename, FA_CREATE_NEW | FA_WRITE | FA_READ);
						if (res == FR_OK)
						{
							f_close(file);
							set_timestamp(filename);
							res = f_open(file, filename, FA_OPEN_EXISTING | FA_WRITE | FA_READ);
							break;
						}
					}
				}
				
				res = f_write(file, &recv, 32, &done);
				if (res != FR_OK)
					sd = SD_ERROR;
				if ((packet % (512/32) == 0))
				{
					f_sync(file);
					set_timestamp(filename);
				}
			}
		}
		
		
		
		
		if (getus()-last_update > 100000)
		{
			// heartbeat packet
			if (heartbeat == 1)
			{
				heartbeat = 0;
				uint8_t packet[32];
				NRF_Tx_DatEx(packet, 1, nrfcb, getus());
			}
			
			// 10 ms TX time
			/*NRF_TX_Mode();
			uint64_t us = getus();
			while(getus()-us < 10000)
				NRF_Handle_Queue();
			NRF_RX_Mode();
			*/
			
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

				
				sprintf((char*)yawstr, "RolPitHead:%.2f %.2f %.2f", roll * 180 / 3.1415926, pitch * 180 / 3.1415926, yaw_est * 180 / 3.1415926);
				ARC_LCD_ShowString(0, LINE_HEIGHT*0, yawstr);
				sprintf((char*)yawstr, "mode:%s time:%.2f", mode_tbl[pilot.fly_mode], pilot_time / 1000000.0f);
				ARC_LCD_ShowString(0, LINE_HEIGHT*1, yawstr);
				sprintf((char*)yawstr, "error:%.2f %.2f %.2f", pilot.error[0]/100.f, pilot.error[1]/100.f, pilot.error[2]/100.f);
				ARC_LCD_ShowString(0, LINE_HEIGHT*2, yawstr);
				sprintf((char*)yawstr, "target:%.2f %.2f %.2f", pilot.target[0]/100.f, pilot.target[1]/100.f, pilot.target[2]/100.f);
				ARC_LCD_ShowString(0, LINE_HEIGHT*3, yawstr);
				sprintf((char*)yawstr, "altitude:%.2f", pilot.altitude/100.f);
				ARC_LCD_ShowString(0, LINE_HEIGHT*4, yawstr);
				sprintf((char*)yawstr, "RC:");
				ARC_LCD_ShowString(0, LINE_HEIGHT*5, yawstr);
				sprintf((char*)yawstr, "%04d %04d %04d %04d %04d %04d", ppm.in[0], ppm.in[1], ppm.in[2], ppm.in[3], ppm.in[4], ppm.in[5]);
				ARC_LCD_ShowString(0, LINE_HEIGHT*6, yawstr);
				sprintf((char*)yawstr, "%04d %04d %04d %04d %04d %04d", ppm.out[0], ppm.out[1], ppm.out[2], ppm.out[3], ppm.out[4], ppm.out[5]);
				ARC_LCD_ShowString(0, LINE_HEIGHT*7, yawstr);
				
				
				sprintf((char*)yawstr, "accel:%d %d %d, %.2f", sensor.accel[0], sensor.accel[1], sensor.accel[2], 
					sqrt((float)sensor.accel[0]*sensor.accel[0] + sensor.accel[1]*sensor.accel[1] + sensor.accel[2] * sensor.accel[2]));
				ARC_LCD_ShowString(0, LINE_HEIGHT*8, yawstr);
				
				sprintf((char*)yawstr, "%.2f,%.2f,%.2f", pilot.error[0]/100.f, pilot2.I[0]/100.f, pilot2.D[0]/100.f);
				ARC_LCD_ShowString(0, LINE_HEIGHT*9, yawstr);

				sprintf((char*)yawstr, "%.2f,%.2f,%.2f", pilot.error[1]/100.f, pilot2.I[1]/100.f, pilot2.D[1]/100.f);
				ARC_LCD_ShowString(0, LINE_HEIGHT*10, yawstr);
			}
			
			// NEMA test
			sprintf((char*)yawstr, "fix:%d,%d, DOP=%.1f,%.1f,%.1f", gps.sig, gps.fix, (float)gps.DOP[0]/100, (float)gps.DOP[1]/100, (float)gps.DOP[2]/100);
			ARC_LCD_ShowString(0, LINE_HEIGHT*11, yawstr);
			sprintf((char*)yawstr, "%f%s %f%s, %.2fm", NDEG2DEG(gps.latitude), gps.latitude > 0 ? "N" : "S", NDEG2DEG(gps.longitude), gps.longitude > 0 ? "E" : "W", (float)gps.altitude);
			ARC_LCD_ShowString(0,LINE_HEIGHT*12, yawstr);
			sprintf((char*)yawstr, "v:%.1fm/s, %d/%d sat", (float)gps.speed/100.0f, gps.satelite_in_use, gps.satelite_in_view);
			ARC_LCD_ShowString(0,LINE_HEIGHT*13, yawstr);

			/*
			if (gps.fix > 1)
				GPIO_ResetBits(GPIOE, GPIO_Pin_2);
			else
				GPIO_SetBits(GPIOE, GPIO_Pin_2);
			
			if (last_nema_parse > getus() - 2000000)
				GPIO_ResetBits(GPIOE, GPIO_Pin_0);
			else
				GPIO_SetBits(GPIOE, GPIO_Pin_0 | GPIO_Pin_2);
			*/
			
			if ((getus() - last_packet_time > 2000000))
			{
				sprintf((char*)yawstr, "NRF NOT RESPONDING %x", packet_types);
				ARC_LCD_ShowString(0, 166, yawstr);
			}
			if (sd != SD_OK)
			{
				ARC_LCD_ShowString(0, 182, "SDCARD ERROR");
			}
			
			sprintf((char*)yawstr, "key:%d%d%d%d", keys[0], keys[1], keys[2], keys[3]);
			ARC_LCD_ShowString(104, 182, yawstr);
			
			
			// airspeed in m/s
			// airspeed = sqrt( 5 * k * R * T * ( (pd/ph+1)^(1/3.5) -1 ) )
			// k : 1.403
			// R : 287.05287
			// T : us ms5611 sensor, convert to kalvin
			// pd : use airspeed sensor, convert to kPa
			// ph : use ms5611 sensor, convert to kPa
			
			float airspeed = sqrt ( 5 * 1.403 * 287.05287 * (imu.temperature/100.0+273.15) *  (pow((-pilot.airspeed/1000.0)/(imu.pressure/1000.0)+1.0, 1/3.5 ) - 1.0) );
			airspeed = sqrt ( 5 * 1.403 * 287.05287 * (20+273.15) *  (pow((pilot.airspeed/1000.0)/(100.0)+1.0, 1/3.5 ) - 1.0) );

			
			int t = RTC_GetCounter();
			struct tm time = current_time();
			sprintf((char*)yawstr, "airspeed=%d, %.2f m/s, HB=%d,%d,0x%02x,%d",  pilot.airspeed, airspeed, heartbeat_count, tx_queue_count, nrfresult, tx_ok);
			ARC_LCD_ShowString(0, 244, yawstr);
			sprintf((char*)yawstr, "Mag:%d,%d,%d, R=%d", (int)mag_zero.array[0], (int)mag_zero.array[1], (int)mag_zero.array[2], (int)mag_radius);
			ARC_LCD_ShowString(0, 256, yawstr);
			sprintf((char*)yawstr, "Battery:%.2fV, %.1fA", sensor.voltage/1000.0f, sensor.current/1000.0f);
			ARC_LCD_ShowString(0, 268, yawstr);
			sprintf((char*)yawstr, "packet speed:%d", packet_speed);
			ARC_LCD_ShowString(0, 284, yawstr);
			sprintf((char*)yawstr, "%d-%d %02d:%02d:%02d %s", time.tm_mon+1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, filename);
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
	return set_timestamp2(filename, time.tm_year+1900, time.tm_mon+1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
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
