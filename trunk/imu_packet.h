#pragma once

#include "mcu.h"

typedef struct 
{
	//int64_t timestamp;
	uint32_t crc;
	float data[16];
	//uint16_t time_delta[16];
	uint32_t endcode;
} imu_packet;