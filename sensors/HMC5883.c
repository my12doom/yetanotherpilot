#include "HMC5883.h"
#include "../common/I2C.h"
#include "../common/common.h"
#include <math.h>

static float gain[3] = {0};

#define	HMC5883SlaveAddress 0x3C
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2


int init_HMC5883(void)
{
	int i;
	int j;
	short data[3];
	float mag_ref[3] = {1.16, 1.16, 1.08};

	I2C_WriteReg(HMC5883SlaveAddress, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS);	// Reg A DOR=0x010 + MS1,MS0 set to pos bias
	I2C_WriteReg(HMC5883SlaveAddress, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
	I2C_WriteReg(HMC5883SlaveAddress,HMC58X3_R_MODE, 1);
	delayms(10);			// Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
											// The new gain setting is effective from the second measurement and on.
	I2C_ReadReg(HMC5883SlaveAddress, 0x03, (u8*)data, 6);
	
	for(j=0; j<10; j++)
	{
		I2C_WriteReg(HMC5883SlaveAddress,HMC58X3_R_MODE, 1);
		delayms(10);
		I2C_ReadReg(HMC5883SlaveAddress, 0x03, (u8*)data, 6);
		for(i=0; i<3; i++)
		{
			swap(&data[i], 2);
			gain[i] += data[i];
		}
				
		// TODO : detect saturation
	}
	
	I2C_WriteReg(HMC5883SlaveAddress,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
	I2C_WriteReg(HMC5883SlaveAddress,HMC58X3_R_MODE, 1);
	delayms(10);
	I2C_ReadReg(HMC5883SlaveAddress, 0x03, (u8*)data, 6);
	
	for(j=0; j<10; j++)
	{
		I2C_WriteReg(HMC5883SlaveAddress,HMC58X3_R_MODE, 1);
		delayms(10);
		I2C_ReadReg(HMC5883SlaveAddress, 0x03, (u8*)data, 6);
		for(i=0; i<3; i++)
		{
			swap(&data[i], 2);
			gain[i] -= data[i];
		}
				
		// TODO : detect saturation
	}
	
	for(i=0; i<3; i++)
		gain[i]=fabs(820.0*mag_ref[i]*2.0*10.0/gain[i]);
	
	I2C_WriteReg(HMC5883SlaveAddress ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	I2C_WriteReg(HMC5883SlaveAddress ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	I2C_WriteReg(HMC5883SlaveAddress, HMC58X3_R_MODE, 0x00);
	
	return 0;
}
int read_HMC5883(short*data)
{
	int i;
	int result = I2C_ReadReg(HMC5883SlaveAddress, 0x03, (u8*)data, 6);
	for(i=0; i<3; i++)
	{
		swap((u8*)&data[i], 2);
		data[i] *= gain[i];
	}
	
	return result;
}
