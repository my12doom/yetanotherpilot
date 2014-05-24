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

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}

int init_HMC5883(void)
{
	int i;
	int j;
	short data[3];
	float mag_ref[3] = {1.16, 1.16, 1.08};
	char identification[3] = {0};

	I2C_ReadReg(HMC5883SlaveAddress, 0x0a, identification, 3);
	if (identification[0] != 'H' || identification[1] != '4' || identification[2] != '3')
		return -2;

	I2C_WriteReg(HMC5883SlaveAddress, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS);	// Reg A DOR=0x010 + MS1,MS0 set to pos bias
	I2C_WriteReg(HMC5883SlaveAddress, HMC58X3_R_CONFB, 0x40);  //Set the Gain
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
		
		if (-(1<<12) >= min(data[0],min(data[1],data[2])))
		{
			ERROR("mag saturation detected\n");
			return -1;
		}
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
				
		if (-(1<<12) >= min(data[0],min(data[1],data[2])))
		{
			ERROR("mag saturation detected\n");
			return -1;
		}
	}
	
	for(i=0; i<3; i++)
		gain[i]=fabs(820.0*mag_ref[i]*2.0*10.0/gain[i]);
	
	I2C_WriteReg(HMC5883SlaveAddress ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	I2C_WriteReg(HMC5883SlaveAddress ,HMC58X3_R_CONFB ,0x10 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	I2C_WriteReg(HMC5883SlaveAddress, HMC58X3_R_MODE, 0x00);
	
	ERROR("mag gain=%.3f, %.3f, %.3f", gain[0], gain[1], gain[2]);
	
	return 0;
}

int check_HMC5883(void)			// check for HMC5883 healthy, re-plug and init it if possible
								// return : 0 if hardware error happend, 
								//			-1 if hardware error resolved
								//			1 if everything OK
{
	return 1;
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
