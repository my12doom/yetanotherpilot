#include "timer.h"
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <misc.h>

#define overflow 60000
#define overhead 144
#define overhead2 25
#define unstablezone 2

static volatile int64_t overflow_count = 0;
static volatile int us_cycle_count;
static volatile int overflow_time_us;

void TIM5_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);
	overflow_count += overflow;	
}

int init_timer(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	us_cycle_count = SystemCoreClock/1000000;
	overflow_time_us = overflow / us_cycle_count / 2;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;      
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_DeInit(TIM5);
	TIM_InternalClockConfig(TIM5);
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=overflow-1;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM5,DISABLE);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM5,ENABLE);
	
	return 0;
}
int64_t gettick(void)
{
	volatile int i = TIM5->CNT;
	if(i <us_cycle_count*unstablezone || i > overflow-us_cycle_count*unstablezone)
		delayus(unstablezone);
	
	return overflow_count + TIM5->CNT;
}
int64_t getus(void)
{
	return gettick() / us_cycle_count;
}

int delayms(int count)
{
	return delayus(count*1000);
}

void delaytick(int count)
{
	volatile uint16_t start; 
	volatile uint16_t target;
	
	if (count < overhead)
		return;
	
	start = TIM5->CNT;
	target = (start + count - overhead) % overflow;
	
	if (start <= target)
	{
		while(TIM5->CNT < target && TIM5->CNT >= start)
			;
	}
	else
	{
		while (TIM5->CNT > start || TIM5->CNT < target)
			;
	}
}
int delayus(int count)
{
	if (count < overflow_time_us)
	{
		delaytick(count*us_cycle_count - overhead2);
	}
	else
	{
		volatile int64_t target = gettick() + count*us_cycle_count;
		while(gettick()<target)	
			;
	}
	
	return 0;
}
