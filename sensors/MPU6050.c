#include "MPU6050.h"
#include <stdio.h>
#include "../common/I2C.h"
#include "../common/common.h"
#include "../common/printf.h"
#include "../common/timer.h"

int MPU6050SlaveAddress = 0xD2;
#define MPU6050_REG_WHO_AM_I 0x75

// Gyro and accelerator registers
#define	SMPLRT_DIV		0x19	//??????,???:0x07(125Hz)
#define	MPU6050_CONFIG 0x1A	//??????,???:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//??????????,???:0x18(???,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//?????????????????,???:0x01(???,2G,5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define EXT_SENS_DATA 0x49
#define	PWR_MGMT_1		0x6B	//????,???:0x00(????)
#define	WHO_AM_I		0x75	//IIC?????(????0x68,??)

static int res;
// call initI2C before this
int init_MPU6050(void)
{
	uint8_t who_am_i = 0;
	
	res = I2C_ReadReg(MPU6050SlaveAddress, WHO_AM_I, &who_am_i, 1);
	if (who_am_i != 0x68)
		MPU6050SlaveAddress = 0xD0;

	TRACE("start MPU6050\r\n");
	delayms(10);
	I2C_WriteReg(MPU6050SlaveAddress, PWR_MGMT_1, 0x80);
	delayms(10);
	I2C_WriteReg(MPU6050SlaveAddress, PWR_MGMT_1, 0x00);
	#if QUADCOPTER == 1
	I2C_WriteReg(MPU6050SlaveAddress, SMPLRT_DIV, 0x02);
	#else
	I2C_WriteReg(MPU6050SlaveAddress, SMPLRT_DIV, 0x07);
	#endif
	I2C_WriteReg(MPU6050SlaveAddress, MPU6050_CONFIG, 0x04);
	I2C_WriteReg(MPU6050SlaveAddress, GYRO_CONFIG, 0);			// full scale : +/-8192; +/- 2000 degree/s
	I2C_WriteReg(MPU6050SlaveAddress, ACCEL_CONFIG, 0x18);
	
	res = I2C_ReadReg(MPU6050SlaveAddress, WHO_AM_I, &who_am_i, 1);
	ERROR("MPU6050 initialized, WHO_AM_I=%x, address = %x\r\n", who_am_i, MPU6050SlaveAddress);
	
	// enable I2C bypass for AUX I2C and initialize HMC5883 into continues mode
#ifdef EXTERNAL_HMC5883_2
 	I2C_WriteReg(MPU6050SlaveAddress, 0x37, 0);
#else
	I2C_WriteReg(MPU6050SlaveAddress, 0x37, 0x02);
#endif
	delayms(10);

	if (who_am_i != 0x68)
		return -1;
	
	return 0;
}

// data[0 ~ 7] :
// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
short gyro_o[3];
static short gyro_raw[3];
int64_t lastus = -1;
int read_MPU6050(short*data)
{	
	int i;
	//int64_t us;
	int result = I2C_ReadReg(MPU6050SlaveAddress, ACCEL_XOUT_H, (uint8_t*)data, 14);
	for(i=0; i<7; i++)
		swap((uint8_t*)&data[i], 2);

	// apply 5hz high pass filter for gyro data
	/*
	us = getus();

	if (lastus == -1)
	{
		for(i=0; i<3; i++)
			gyro_raw[i] = gyro_o[i] = data[i+4];
	}
	else
	{
		float dt = (us-lastus)/1000000.0f;
		const float RC = 0.03183f;//1.0f/(2*3.1415926 * 5);	// 5hz High pass filter		
		float alpha = RC / (dt + RC);

		for(i=0; i<3; i++)
		{
			gyro_o[i] = alpha * (gyro_o[i] + data[i+4] - gyro_raw[i]);
		}

		for(i=0; i<3; i++)
		{
			gyro_raw[i] = data[i+4];
			data[i+4] = gyro_o[i];
		}
	}
	lastus = us;
	*/
	
	return result;
}
