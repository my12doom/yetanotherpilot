#include <stdio.h>
#include <stm32f10x.h>
#include <misc.h>
#include <stm32f10x_fsmc.h>

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

void LCD_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* ʹ��FSMCʱ��*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
    
    /* ʹ��FSMC��Ӧ��Ӧ�ܽ�ʱ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE , ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    /* ����LCD������ƹܽ�*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /* ����LCD��λ���ƹܽ�*/
   /* LCD��λ��ϵͳ��λ��һ�����Բ��������������ڸ�λ����û��������IO����λ�������PE1û�����ҽ���LED�ϣ�*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 	 
    GPIO_Init(GPIOE, &GPIO_InitStructure);  		   
    
    /* ����FSMC���Ӧ��������,FSMC-D0~D15: PD 14 15 0 1,PE 7 8 9 10 11 12 13 14 15,PD 8 9 10*/	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | 
                                  GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                  GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure); 
    
  /* ����FSMC���Ӧ�Ŀ�����
   * PD4-FSMC_NOE  :LCD-RD  ��
   * PD5-FSMC_NWE  :LCD-WR  д
   * PD7-FSMC_NE1  :LCD-CS  Ƭѡ
   * PD11-FSMC_A16 :LCD-RS/DC  ����/����
	 **********************************************/
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_11; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void LCD_FSMC_Config(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p; 
   
    p.FSMC_AddressSetupTime = 0x02;	 //��ַ����ʱ��
    p.FSMC_AddressHoldTime = 0x00;	 //��ַ����ʱ��
    p.FSMC_DataSetupTime = 0x05;		 //���ݽ���ʱ��
    p.FSMC_BusTurnAroundDuration = 0x00;
    p.FSMC_CLKDivision = 0x00;
    p.FSMC_DataLatency = 0x00;
    p.FSMC_AccessMode = FSMC_AccessMode_B;	
    
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;	
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p; 
   
    
    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
    
    /* ʹ�� FSMC Bank1_SRAM Bank */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}

#include "LCD/arc_lcd.h"

int main(void)
{

	// Basic Initialization
	SysTick_Config(720);
	printf_init();
	SPI_NRF_Init();
	int nrf = NRF_Check();
	printf("NRF_Check() = %d\r\n", nrf);
	init_timer();
	
	LCD_GPIO_Config();	
	LCD_FSMC_Config();
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	delayms(50);
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
	delayms(10);
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	
	ARC_LCD_Init();
	ARC_LCD_Clear(0);
	uint8_t tmp[32];
	for(int i=0; i<0xffff; i= (i+0x0100)&0xffff)
	{
		sprintf((char*)tmp, "HelloWorld���:%d", (int)getus());
		printf("%s\r\n", tmp);
		ARC_LCD_Clear(0);
		ARC_LCD_ShowString(0, 0, tmp);
		
		delayms(10);
	}
	
	
	// LCD test
	
	while(1)
	{
		
	}
	/*

	
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
	res = f_open(&file, "1055.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	res = f_lseek(&file,file.fsize);
	f_printf (&file,"%s, %d\r\n", "Hello Station!", (int)getus());
	f_write(&file, "\x12\x34\x56\x78", 4, &done);	
	f_close(&file); 
	f_mount(0, NULL);
	
  while (1)
  {
  }
	
	*/
}
