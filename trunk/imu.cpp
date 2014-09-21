#include "mcu.h"
#include <stdio.h>
#include <string.h>
#include "common/timer.h"
#include "common/printf.h"
#include "common/space.h"
#include "common/ads1258.h"
#include "common/i2c.h"
#include "sensors/hmc5883.h"

void UART4_init(uint32_t baud_rate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	// config USART1 clock and its GPIO Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	// UART4 GPIO (PC10(TX) & PC11(RX))config 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// Configure PC11(RX) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);

	  
	// USART1 mode config
	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure); 
	USART_Cmd(UART4, ENABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
}


int main()
{
	init_timer();
	printf_init();
	space_init();
	delayms(400);
	ads1258_init();
	I2C_init(0);
	UART4_init(115200);
	
	int hmc5883 = init_HMC5883();

	ads1258_config1 config1;
	ads1258_read_registers(REG_CONFIG1, 1, &config1);
	config1.DRATE = 0;
	config1.DLY = 0;
	ads1258_write_registers(REG_CONFIG1, 1, &config1);
	ads1258_read_registers(REG_CONFIG1, 1, &config1);


	ads1258_config0 config0;
	ads1258_read_registers(REG_CONFIG0, 1, &config0);
	config0.BYPAS = 1;
	config0.CHOP = 1;
	ads1258_write_registers(REG_CONFIG0, 1, &config0);
	ads1258_read_registers(REG_CONFIG0, 1, &config0);
	
	ads1258_SYSRED sys;
	ads1258_read_registers(REG_SYSRED, 1, &sys);
	/*
	sys._OFFSET = 1;
	sys.REF = 1;
	sys.GAIN = 1;
	sys.VCC = 1;
	sys.zero = 1;
	sys.TEMP = 1;
	*/

	memset(&sys, 0, 1);
	ads1258_write_registers(REG_SYSRED, 1, &sys);
	ads1258_read_registers(REG_SYSRED, 1, &sys);

	GPIO_SetBits(GPIOA, GPIO_Pin_3);

	
	int i = 0;
	int t = getus();

	int counter = 0;
	float pa = 0;

	float avg_gyro=0;
	int avg_gyro_count = 0;

	while(1)
	{
		ads1258_go();


		if (last_update_channel == 22)
		{
			float v = 5.3f * channel_data[8]/ 8388607.0f;
			float v50 = 5.3f * 1 * channel_data[11]/ 8388607.0f;
			float vaccel = 5.3f * 1 * channel_data[22]/ 8388607.0f;
			float vgyro = 5.3f * channel_data[9]/ 8388607.0f;
			float pa_6115;
			float a = (vaccel - 2.50f) * 9.805f;
			float gyro_degree = (vgyro-2.50f)/0.006f;
			//v50 = 5.0f;
			
			pa_6115 = 15000 + (v-v50*0.05f)/(0.9f*v50)*100000.0f;
			//ERROR("%d=%d(%08x)(%f), state=%x   \r", channel, channel_data[channel], channel_data[channel], pa_6115, state);


			t = getus() - t;

			counter++;
			pa += pa_6115;

			avg_gyro_count++;
			avg_gyro += gyro_degree;


			if (counter == 8)
			{
				char tmp[200];
				sprintf(tmp, "%f,%.3f\n", getus()/1000000.0f, pa/8);
				//ERROR(tmp);
				int len = strlen(tmp);
				for(int i=0; i<len; i++)
				{
					USART_SendData(UART4, (unsigned char) tmp[i]);
					while (!(UART4->SR & USART_FLAG_TXE));
				}
				pa = counter = 0;
			}

			ERROR("%f,%.3f, %f\n", getus()/1000000.0f, a, avg_gyro/avg_gyro_count);


			t = getus();
			
			last_update_channel = -1;
		}
	}
}
