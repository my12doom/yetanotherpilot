#include "uart4.h"
#include "build.h"
#include <string.h>
#include <stdio.h>
#include "../mcu.h"
#include "common.h"

#define TX_BUFFER_SIZE 1024
#define RX_BUFFER_SIZE 512

static char buffer[RX_BUFFER_SIZE];		// circular buffer
static int start = 0;						// valid data start in circular
static int end = 0;							// valid data endl in circular buffer
static int end_sentence = 0;				// valid sentence endl in circular buffer
static int lastR = 0;

static char tx_buffer[TX_BUFFER_SIZE];
static int tx_start = 0;					// valid data start in circular
static int tx_end = 0;						// valid data endl in circular buffer
static int ongoing_tx_size = 0;
static int dma_running = 0;

void UART4_IRQHandler(void)
{
	int c = -1;
	//if(USART_GetFlagStatus(UART4,USART_IT_RXNE)==SET)
	{
		c = USART_ReceiveData(UART4);
	}
	
	//USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	UART4->SR = (uint16_t)~0x20;
	if (c>0)
	{
		/*
		if (lastR)
		{
			lastR = 0;
			if (c == '\n')
				end_sentence = end;
			if (c == '\r')
			{
				buffer[end] = c;
				end++;
				end %= sizeof(buffer);
			}
			// other escapes are ignored here
		}
		else
		{
			if (c == '\r')
				lastR = 1;
			else
			{
				buffer[end] = c;
				end++;
				end %= sizeof(buffer);
			}
		}
		buffer[end] = 0;
		*/
		
		buffer[end] = c;
		end++;
		end %= sizeof(buffer);
		if (c == '\n')
			end_sentence = end;

	}
}

int dma_handle_queue()
{
	if (dma_running)
		return 0;

	DMA_Cmd(DMA1_Stream4, DISABLE);

	ongoing_tx_size = tx_end - tx_start;
	if (ongoing_tx_size == 0)
		return 0;
	if (ongoing_tx_size < 0)
		ongoing_tx_size = sizeof(tx_buffer) - tx_start;
	
	DMA1_Stream4->NDTR = ongoing_tx_size;
	DMA1_Stream4->M0AR = (uint32_t)tx_buffer + tx_start;

	DMA_Cmd(DMA1_Stream4, ENABLE);

	dma_running = 1;

	//ERROR("!!%d!!", ongoing_tx_size);

	return 0;
}

int UART4_SendPacket(const void *buf, int size)
{
	int i;
	const char *p = (const char*)buf;
	int new_pos = (tx_end + size) % sizeof(tx_buffer);
	int old_size = tx_end-tx_start < 0 ? tx_end-tx_start + sizeof(tx_buffer) : tx_end-tx_start;
	if (size + old_size > sizeof(tx_buffer))
	{
		dma_handle_queue();
		return 0;		// reject all data if buffer overrun
	}

	for(i=0; i<size; i++)
		tx_buffer[(tx_end+i)%sizeof(tx_buffer)] = p[i];

	tx_end = (tx_end+size)%sizeof(tx_buffer);
	
	size = tx_end-tx_start;
	if (size<0)
		size += sizeof(tx_buffer);

	//if (old_size > size)
	//	ERROR("holy");
	//ERROR("(%d)", size);
	
	dma_handle_queue();
	
	return size;
}


static int dma_init()
{
	DMA_InitTypeDef DMA_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	DMA_DeInit(DMA1_Stream4);  

	DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(UART4->DR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&tx_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = sizeof(tx_buffer);
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

void UART4_Init(uint32_t baud_rate, int PC10_11)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// config UART4 clock and its GPIO Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	
	if (PC10_11)
	{
		// UART4 GPIO (PC10(TX) & PC11(RX))config 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	}
	else
	{
		// UART4 GPIO (PA0(TX) & PA1(RX))config 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
	}

	// NVIC config
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// UART4 mode config
	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure); 
	USART_Cmd(UART4, ENABLE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	// 
	dma_init();
}

void DMA1_Stream4_IRQHandler()
{
	tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
	dma_running = 0;
	dma_handle_queue();
}



int UART4_ReadPacket(void *out, int maxsize)
{
	char *p = (char*)out;
	int _end_sentence = end_sentence;
	int j=0;
	int i;
	int size;
	int lastR = 0;
	
	if (_end_sentence == start)
		return -1;

	size = _end_sentence - start;
	if (size<0)
		size += sizeof(buffer);

	if (size >= maxsize)
		return -2;

	
	for(i=start; i!= _end_sentence; i=(i+1)%sizeof(buffer))
	{
		if (buffer[i] == '\r')
		{
			if (lastR)
				p[j++] = buffer[i];
			lastR = !lastR;
		}
		
		p[j++] = buffer[i];
		if (buffer[i] == '\n')
		{
			i=(i+1)%sizeof(buffer);
			break;
		}
	}
	
	p[j] = 0;
	
	start = i;

	return j;
}
