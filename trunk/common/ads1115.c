#include "ads1115.h"
#include "../mcu.h"
#include <string.h>

#define ADS1115_ADDR1 0x90
#define ADS1115_ADDR2

#define CONVERSION 0
#define CONFIG 1
#define LOW_THRESH 2
#define HIGH_THRESH 3


typedef struct
{
	unsigned MODE:1;
	unsigned PGA:3;
	unsigned MUX:3;
	unsigned OS:1;
	
	
	unsigned COMP_QUE:2;
	unsigned COMP_LAT:1;
	unsigned COMP_POL:1;
	unsigned COMP_MODE:1;
	unsigned DR:3;
	
} ads1115_config_value;

ads1115_config_value config;

static void swap(void *buf, int size)
{
	char *p = (char*)buf;
	int i;
	for(i=0; i<size/2; i++)
	{
		char t = p[i];
		p[i] = p[size-1-i];
		p[size-1-i] = t;
	}
}


static int mSCL_PIN;
static int mSDA_PIN;
static GPIO_TypeDef *mSDA_PORT;
static GPIO_TypeDef *mSCL_PORT;
extern int SCL_PIN;
extern int SDA_PIN;
extern GPIO_TypeDef *SDA_PORT;
extern GPIO_TypeDef *SCL_PORT;

static void switch_I2C()
{
	/*
	mSCL_PIN = SCL_PIN;
	mSDA_PIN = SDA_PIN;
	mSCL_PORT = SCL_PORT;
	mSDA_PORT = SDA_PORT;

	SCL_PIN = GPIO_Pin_0;
	SDA_PIN = GPIO_Pin_1;
	SCL_PORT = GPIOA;
	SDA_PORT = GPIOA;
	*/
}

static void restore_I2C()
{
	/*
	SCL_PIN = mSCL_PIN;
	SDA_PIN = mSDA_PIN;
	SCL_PORT = mSCL_PORT;
	SDA_PORT = mSDA_PORT;
	*/
}

int ads1115_init(void)
{
// 	GPIO_InitTypeDef  GPIO_InitStructure;
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	memset(&config, 0, 4);
	switch_I2C();
	I2C_init(0x30);
	I2C_WriteReg(0, 0x06, 0);			// 0,0x06 = reset	
	I2C_ReadReg(ADS1115_ADDR1, CONFIG, (uint8_t*)&config, 2);
	restore_I2C();
	
	if (*(unsigned short*)&config != 0x8385)	// the default config register
		return -1;
	
	return 0;
}

int ads1115_config(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, ads1115_mode mode)
{
	switch_I2C();
	I2C_ReadReg(ADS1115_ADDR1, CONFIG, (uint8_t*)&config, 2);
	config.DR = speed;
	config.MUX = channel;
	config.PGA = gain;
	config.MODE = mode;
	I2C_WriteRegs(ADS1115_ADDR1, CONFIG, (uint8_t*)&config, 2);
	restore_I2C();
	
	return 0;
}

int ads1115_startconvert(void)
{
	switch_I2C();
	I2C_ReadReg(ADS1115_ADDR1, CONFIG, (uint8_t*)&config, 2);
	config.OS = 1;
	I2C_WriteRegs(ADS1115_ADDR1, CONFIG, (uint8_t*)&config, 2);
	restore_I2C();
	
	return 0;
}

int ads1115_getresult(short *result)		// return -1 if still converting, 0 if conversion completed, further calls return the last conversion result.
{
	switch_I2C();
	I2C_ReadReg(ADS1115_ADDR1, CONFIG, (uint8_t*)&config, 2);
	
	if (config.MODE == 1 && config.OS == 0)
	{
		restore_I2C();
		return -1;
	}

	I2C_ReadReg(ADS1115_ADDR1, CONVERSION, (uint8_t*)result, 2);
	restore_I2C();
	swap(result, 2);
	
	return 0;
}

short ads1115_convert(void)				// a simplfied version which start a new conversion ,wait it to complete and returns the result directly
{
	short v;
	ads1115_startconvert();
	while (ads1115_getresult(&v)!=0)
		;
	return v;
}
