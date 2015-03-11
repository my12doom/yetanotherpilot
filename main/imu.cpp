#include "../common/mcu.h"
#include <stdio.h>
#include <string.h>
#include "../common/timer.h"
#include "../common/printf.h"
#include "../common/space.h"
#include "../common/ads1258.h"
#include "../common/i2c.h"
#include "../sensors/hmc5883.h"
#include "../common/fifo.h"
#include "../common/uart4.h"
#include "stm32f4xx_dma.h"
#include "crc32.h"
#include "imu_packet.h"


/*
CircularQueue<unsigned char, 512> tx_queue;
imu_packet packet = {0};
imu_packet tx;

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
	
	// Configure PC11(RX) as input floating
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);

	  
	// UART4 mode config
	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure); 
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART4, ENABLE);
	
	USART_ClockInitTypeDef USART_ClockInitStruct;
	USART_ClockStructInit(&USART_ClockInitStruct);
	USART_ClockInit(UART4, &USART_ClockInitStruct);


}

// DMA test
char tx_buffer[512] = "HelloWorld!!12345678";

extern "C"
void DMA1_Stream4_IRQHandler()
{
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4) ;
	//ERROR("HI %d       \n", int(getus()));
}

int dma_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);


	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
	NVIC_Init(&NVIC_InitStructure);


	DMA_InitTypeDef DMA_InitStructure = {0};
	DMA_DeInit(DMA1_Stream4);  

	DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(UART4->DR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&tx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = sizeof(tx);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);

	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); 
	
 	return 0;
}

int dma_go()
{
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,DISABLE);
	DMA_Cmd(DMA1_Stream4, DISABLE);
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
	DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream4, ENABLE);


	return 0;
}
*/

int main()
{
	init_timer();
	printf_init();
	space_init();
	ads1258_init();
	I2C_init(0);
	UART4_Init(2100000, 1);
	
	int hmc5883 = init_HMC5883();

	ads1258_config1 config1;
	ads1258_read_registers(REG_CONFIG1, 1, &config1);
	config1.DRATE = 2;
	config1.DLY = 0;
	ads1258_write_registers(REG_CONFIG1, 1, &config1);
	ads1258_read_registers(REG_CONFIG1, 1, &config1);


	ads1258_config0 config0;
	ads1258_read_registers(REG_CONFIG0, 1, &config0);
	config0.BYPAS = 1;
	config0.CHOP = 0;
	ads1258_write_registers(REG_CONFIG0, 1, &config0);
	ads1258_read_registers(REG_CONFIG0, 1, &config0);
	
	ads1258_SYSRED sys;
	ads1258_read_registers(REG_SYSRED, 1, &sys);
	memset(&sys, 0, 1);
	sys.VCC = 1;
	sys.REF = 1;
	/*
	sys._OFFSET = 1;
	sys.GAIN = 1;
	sys.zero = 1;
	sys.TEMP = 1;
	*/

	ads1258_write_registers(REG_SYSRED, 1, &sys);
	ads1258_read_registers(REG_SYSRED, 1, &sys);

	GPIO_SetBits(GPIOA, GPIO_Pin_3);

	
	int i = 0;
	int t = getus();

	int counter = 0;
	float pa = 0;

	float avg_gyro=0;
	int avg_gyro_count = 0;

	struct
	{
		float data[16];
		unsigned long crc;
		char N;
	} packet;
	packet.N = '\n';
	while(1)
	{
		int channel = last_update_channel;
		if (channel >= 0 && channel >= 8 && channel <24)
		{
			last_update_channel = -1;
			packet.data[channel-8] = channel_data[channel] * 5.3f / 8388607.0f;
			
			if (channel == 23)
			{
				packet.crc = crc32(0, (uint8_t*)&packet, sizeof(packet.data));
				UART4_SendPacket(&packet, sizeof(packet));
				float a1 = (packet.data[13] - 2.50f);
				float a4 = (packet.data[11] - 2.50f) * 10;
				//printf("%f\n", packet.data[11]);
				//printf("%f,%f\n", a1, a4);
			}
		}
		
		//printf("\r%d", int(getus()));
		
		if (last_update_channel == 22)
		{
			float vcc = channel_data[25] / 786432.0f;
			float vref = channel_data[28] / 786432.0f;
			float vtemp = channel_data[10] * 5.3 / 8388607.0f;
			float vcc2 = channel_data[11] * 5.3 / 8388607.0f;
			
			LOGE("\r%f,%f,%f,%f", vcc, vref, vtemp, vcc2);
		}

		
		/*
		if (last_update_channel == 22)
		{
			float v = 5.3f * channel_data[8]/ 8388607.0f;
			float v50 = 5.3f * 1 * channel_data[11]/ 8388607.0f;
			float pa_6115 = 15000 + (v-v50*0.05f)/(0.9f*v50)*100000.0f;
			
			//ERROR("pa=%.2f", pa_6115);
		}

		if (last_update_channel == 15)
		{
			// AIN0 = channel 8
			
			float v = 5.3f * channel_data[8]/ 8388607.0f;
			float v50 = 5.3f * 1 * channel_data[11]/ 8388607.0f;
			float vaccel = 5.3f * 1 * channel_data[22]/ 8388607.0f;
			float vgyro = 5.3f * channel_data[18]/ 8388607.0f;
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


			if (counter == 8 && true)
			{
				char tmp[200];
				sprintf(tmp, "%f,%.3f\n", getus()/1000000.0f, pa/8);
				ERROR(tmp);
				int len = strlen(tmp);
				for(int i=0; i<len; i++)
				{
					USART_SendData(UART4, (unsigned char) tmp[i]);
					while (!(UART4->SR & USART_FLAG_TXE));
				}
				pa = counter = 0;
			}

			//ERROR("%f,%.3f, %f\n", getus()/1000000.0f, gyro_degree, avg_gyro/avg_gyro_count);
			//for(int i=0; i<16; i++)
			//	ERROR("(%d)%01.5f,", i, 5.3f * channel_data[8+i]/ 8388607.0f);
			//ERROR(",%.3f\r", getus()/1000000.0f);


			t = getus();
			last_update_channel = -1;
			
		}
		
		extern int irq;
		//printf("irq=%.2f", irq/(getus()/1000000.0f));
		*/
	}
}
