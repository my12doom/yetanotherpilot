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

int channel_data[29];
int last_update_channel = -1;

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
	i = (start&0xf) | (CMD_RegisterWrite<<5) | (n>1?0x10:0x00);
	ads1258_tx_rx(i);
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

int irq = 0;

void EXTI4_IRQHandler()
{
	irq++;
	EXTI_ClearITPendingBit(EXTI_Line4);
	ads1258_go();
}

int ads1258_init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);


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
		ads1258_read_registers(i, 1, &data);
		LOGE("reg(%d)=0x%02x\n", i, data);
	}

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_ClearITPendingBit(EXTI_Line4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, GPIO_PinSource4);
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


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

static int t = 0;
int ads1258_go(void)
{
	int i;
	int channel;
	char p[4];
	ads1258_begin();
	i = (1<<5) | 0x10;
	ads1258_tx_rx(i);
	for(i=0; i<4; i++)
		p[i]=ads1258_tx_rx(0xff);

	ads1258_end();

	if (p[0] & 0x80)		// new data?
	{
		int state = p[0];
		channel = p[0] & 0x1f;
		p[0] = (p[1] & 0x80) ? 0xff : 0x00;
		swap(p,4);

		channel_data[channel] = *(int*)p;
		last_update_channel = channel;
		
		return channel;
	}
	
	return -1;
}
