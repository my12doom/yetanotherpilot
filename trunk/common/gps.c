#include "gps.h"
#include "config.h"
#include <misc.h>

nmeaINFO info;
nmeaPARSER parser;
char buffer[GPS_BUFFER_BLOCK];		// circular buffer
int start = 0;						// valid data start in circular
int end = 0;						// valid data length in circular buffer
int end_sentence = 0;				// valid sentence length in circular buffer

void USART1_IRQHandler(void)
{
	int c = -1;
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==SET)
	{
		c = USART_ReceiveData(USART1);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	
	if (c>0)
	{
		buffer[end] = c;
		end++;
		end %= GPS_BUFFER_BLOCK;
		if (c == '\n')
			end_sentence = end;
		buffer[end] = 0;
	}
}

void GPS_Init(uint32_t baud_rate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	// config USART1 clock and its GPIO Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	// USART1 GPIO (A9(TX) & A10(RX))config 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	// NVIC config
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	  
	// USART1 mode config
	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	// NMEA init
	nmea_zero_INFO(&info);
	nmea_parser_init(&parser);
}

char to_parse[GPS_BUFFER_BLOCK];
int GPS_ParseBuffer()
{
	int _end_sentence = end_sentence;
	int j=0;
	int i;
	int len;
	
	if (_end_sentence == start)
		return 0;
	for(i=start; i!= _end_sentence; i=(i+1)%GPS_BUFFER_BLOCK, j++)
		to_parse[j] = buffer[i];
	
	to_parse[j] = 0;
	TRACE("GPS:%s", to_parse);
	
	start = _end_sentence;

	return nmea_parse(&parser, to_parse, j, &info);
}

nmeaINFO* GPS_GetInfo()
{
	return &info;
}
