#include "ads1258.h"
#include "../mcu.h"
#include "build.h"
#include <string.h>
#include "printf.h"

#define ADS1258_ADDR1 0x90
#define ADS1258_ADDR2

#define CONVERSION 0
#define CONFIG 1
#define LOW_THRESH 2
#define HIGH_THRESH 3


void ads1258_begin()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
}
void ads1258_end()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_2);
}

uint8_t ads1258_tx_rx(uint8_t Data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, Data);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Data = SPI_I2S_ReceiveData(SPI1);

	return Data;
} 


uint8_t ads1258_read_registers(uint8_t start, uint8_t n, void *out)
{
	int i;
	uint8_t *p = (uint8_t*)out;
	ads1258_begin();
	i = (start&0xf) | (CMD_RegisterRead<<5) | (n>1?0x10:0x00);
	ads1258_tx_rx(i);
	for(i=0; i<n; i++)
		p[i]=ads1258_tx_rx(0xff);

	ads1258_end();

	return n;
}

uint8_t ads1258_write_registers(uint8_t start, uint8_t n, void *data)
{
	int i;
	uint8_t *p = (uint8_t*)data;
	ads1258_begin();
	ads1258_tx_rx((start&0xf) | 0x50);
	ads1258_tx_rx(n-1);
	delayms(2);
	for(i=0; i<n; i++)
		ads1258_tx_rx(p[i]);

	ads1258_end();

	return n;
}

uint8_t ads1258_read_register(uint8_t reg)
{
	ads1258_read_registers(reg, 1, &reg);
	return reg;
}
void ads1258_write_register(uint8_t reg, uint8_t data)
{
	ads1258_write_registers(reg, 1, &data);
}



int ads1258_init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t data = 55;
	int i = 0;

#ifdef STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F4
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// ÅäÖÃ SCK,MISO,MOSI Òý½Å£¬GPIOA^5,GPIOA^6,GPIOA^7
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#endif
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);

	//ÅäÖÃ CS Òý½Å£¬GPIOA^3(PCB1.0) / GPIOB^2(PCB2.0) / GPIOA^4(PCB3.0)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#endif
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	ads1258_end();

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //Ë«ÏßÈ«Ë«¹¤
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//Ö÷Ä£Ê½
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//Êý¾Ý´óÐ¡ 8 Î»
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//Ê±ÖÓ¼«ÐÔ£¬¿ÕÏÐÊ±ÎªµÍ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//µÚ 1 ¸ö±ßÑØÓÐÐ§£¬ÉÏÉýÑØÎª²ÉÑùÊ±¿Ì
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS ÐÅºÅÓÉÈí¼þ²úÉú
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//8 ·ÖÆµ£¬9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//¸ßÎ»ÔÚÇ°
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);

	ads1258_begin();
	ads1258_tx_rx(0xff);
	ads1258_end();

	delayms(50);

	for(i=0; i<10; i++)
	{
		delayms(50);

		ads1258_read_registers(i, 1, &data);


		ERROR("reg(%d)=0x%02x\n", i, data);
	}

	return 0;
}

int ads1258_startconvert(void)
{
	
	return 0;
}

int ads1258_getresult(short *result)		// return -1 if still converting, 0 if conversion completed, further calls return the last conversion result.
{
	
	return 0;
}

short ads1258_convert(void)				// a simplfied version which start a new conversion ,wait it to complete and returns the result directly
{
	short v;
	ads1258_startconvert();
	while (ads1258_getresult(&v)!=0)
		;
	return v;
}
