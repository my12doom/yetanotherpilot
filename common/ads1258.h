#pragma once

#include "mcu.h"

extern int channel_data[29];		// data read in auto-scan mode
extern int last_update_channel;

typedef enum
{
	ads1258_speed_30000sps = 0xf0,
	ads1258_speed_15000sps = 0xe0,
	ads1258_speed_7500sps = 0xd0,
	ads1258_speed_3750sps = 0xc0,
	ads1258_speed_2000sps = 0xb0,
	ads1258_speed_1000sps = 0xa1,
	ads1258_speed_500sps = 0x92,
	ads1258_speed_100sps = 0x82,
	ads1258_speed_60sps = 0x72,
	ads1258_speed_50sps = 0x63,
	ads1258_speed_30sps = 0x53,
	ads1258_speed_25sps = 0x43,
	ads1258_speed_15sps = 0x33,
	ads1258_speed_10sps = 0x23,
	ads1258_speed_5sps = 0x13,
	ads1258_speed_2_5sps = 0x03,
} ads1258_speed;

typedef enum
{
	ads1258_channnel_AIN0 = 0,
	ads1258_channnel_AIN1 = 1,
	ads1258_channnel_AIN2 = 2,
	ads1258_channnel_AIN3 = 3,
	ads1258_channnel_AIN4 = 4,
	ads1258_channnel_AIN5 = 5,
	ads1258_channnel_AIN6 = 6,
	ads1258_channnel_AIN7 = 7,
	ads1258_channnel_AINCOM = 8,
} ads1258_channel;

typedef enum
{
	ads1258_gain_1 = 0,		// x1
	ads1258_gain_2 = 1,		// x2
	ads1258_gain_4 = 2,		// x4
	ads1258_gain_8 = 3,		// x8
	ads1258_gain_16 = 4,	// x16
	ads1258_gain_32 = 5,	// x32
	ads1258_gain_64 = 6,	// x64
	ads1258_gain_64_2 = 7,	// x64
} ads1258_PGA;


typedef struct
{
	unsigned zero0:1;
	unsigned STAT:1;
	unsigned CHOP:1;
	unsigned CLKENB:1;
	unsigned BYPAS:1;
	unsigned MUXMOD:1;
	unsigned SPIRST:1;
	unsigned zero1:1;
} ads1258_config0;						// CONFIG0 register(0)

typedef struct
{
	unsigned DRATE:2;
	unsigned SBCS:2;
	unsigned DLY:3;
	unsigned IDLMOD:1;
} ads1258_config1;						// CONFIG1 register(0)

typedef struct
{
	unsigned negative:4;
	unsigned positive:4;
} ads1258_mux_sch;						// MUXSCH register(1)

typedef int8_t ads1258_mux_diff;
typedef int8_t ads1258_mux_sg0;
typedef int8_t ads1258_mux_sg1;
typedef int8_t ads1258_mux_sg1;
typedef int8_t ads1258_gpioc;
typedef int8_t ads1258_gpiod;
typedef int8_t ads1258_id;

typedef struct
{
	unsigned _OFFSET:1;
	unsigned zero:1;
	unsigned VCC:1;
	unsigned TEMP:1;
	unsigned GAIN:1;
	unsigned REF:1;
	unsigned zero1:2;
} ads1258_SYSRED;						// SYSRED register(0)


typedef enum
{
	REG_CONFIG0 = 0,
	REG_CONFIG1 = 1,
	REG_MUXSCH= 2,
	REG_MUXDIF = 3,
	REG_MUXSG0 = 4,
	REG_MUXSG1 = 5,
	REG_SYSRED = 6,
	REG_GPIOC = 7,
	REG_GPIOD = 8,
	REG_ID = 9,
} ads1258_register;

typedef enum
{
	CMD_ReadDirect = 0x00,
	CMD_ReadCommand = 0x01,
	CMD_RegisterRead = 0x02,
	CMD_RegisterWrite = 0x03,
	CMD_PulseConvert = 0x04,
	CMD_Reserved = 0x05,
	CMD_Reset = 0x06,
	CMD_ReadDirect2 = 0x07,
} ads1258_command;

#ifdef __cplusplus
extern "C" {
#endif

void ads1258_end(void);
void ads1258_begin(void);
uint8_t ads1258_tx_rx(uint8_t Data);
uint8_t ads1258_read_registers(uint8_t start, uint8_t n, void *out);
uint8_t ads1258_write_registers(uint8_t start, uint8_t n, void *out);
uint8_t ads1258_read_register(uint8_t reg);
void ads1258_write_register(uint8_t reg, uint8_t data);
int ads1258_init(void);
int ads1258_startconvert(void);
int ads1258_getresult(short *result);		// return -1 if still converting, 0 if conversion completed or continuous mode, further calls return the last conversion result.
short ads1258_convert(void);				// a simplfied version which start a new conversion ,wait it to complete and returns the result directly
int ads1258_go(void);

#ifdef __cplusplus
}
#endif
