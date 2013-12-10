#include "printf.h"
#include "config.h"
#include <stdarg.h>
#include <misc.h>
#include <stdio.h>
#include "../nmea/nmea.h"
nmeaINFO info;
nmeaPARSER parser;

void printf_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// NEMA init
	nmea_zero_INFO(&info);
	nmea_parser_init(&parser);
	
	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	// NVIC config
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   /*3.4??????USART1_IRQChannel,?stm32f10x.h?*/
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	  
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 4800;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

static char str[1024];
static int n = 0;

static int strlen(const char*p)
{
	int o = 0;
	while (*p++)
		o++;
	return o;
}

void USART1_IRQHandler(void)
{
	unsigned int c = 0;
	int p = 0;
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==SET)
	{
		c = USART_ReceiveData(USART1);
		p = 1;
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) ;
	}

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	
	if (p)
	{
		//fputc(c, stdout);
		//USART_SendData(USART1, c);
		str[n++] = c;
		if (c == '\n')
		{
			str[n] = NULL;
			printf(str);
			nmea_parse(&parser, str, (int)strlen(str), &info);
			n = 0;
		}
	}
}

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin; 

int fputc(int ch, FILE *f)
{
#ifdef USART1_DBG
	USART_SendData(USART1, (unsigned char) ch);
	while (!(USART1->SR & USART_FLAG_TXE));
#endif
#ifdef ITM_DBG
  if (DEMCR & TRCENA) 
	{
    while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
#endif
	return (ch);
}
