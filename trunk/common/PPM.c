#include "PPM.h"
#include "timer.h"
#include <stm32f10x_exti.h>
#include <stm32f10x_tim.h>
#include <misc.h>

u32 g_ppm_input_start[4];
float g_ppm_input[4];
int ppm_raw_input[4][3];						//	[channel][v1, v2, v3], median filter with window-size=3
int ppm_raw_input_counter[4] = {0};
u16 g_ppm_output[8] = {0};
int64_t g_ppm_input_update[4] = {0};
int g_enable_input;
int tbl[3];


// PPM input handler
void EXTI9_5_IRQHandler(void)
{
	int channel = -1;
	static int pin_tbl[4] = {GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9};
	static int line_tbl[4] = {EXTI_Line6, EXTI_Line7, EXTI_Line8, EXTI_Line9};

	while(1)
	{
		channel = -1;
		if (EXTI_GetITStatus(EXTI_Line6) != RESET)
			channel = 0;
		if (EXTI_GetITStatus(EXTI_Line7) != RESET)
			channel = 1;
		if (EXTI_GetITStatus(EXTI_Line8) != RESET)
			channel = 2;
		if (EXTI_GetITStatus(EXTI_Line9) != RESET)
			channel = 3;
		if (channel == -1)
			break;
	
		if(GPIOB->IDR & pin_tbl[channel])
			g_ppm_input_start[channel] = TIM_GetCounter(TIM4);
		else
		{
			const float RC = 0.00795f;//1.0f/(2*3.1415926 * 20);	// 20hz low pass filter
			u32 now = TIM_GetCounter(TIM4);
			float t_delta = (getus() - g_ppm_input_update[channel]) / 1000000.0f;
			float alpha = t_delta / (t_delta + RC);
			int *p = &ppm_raw_input_counter[channel];
			(*p)++;
			*p &= 3;		// %=4
			if (now > g_ppm_input_start[channel])
				ppm_raw_input[channel][*p] = now - g_ppm_input_start[channel];
			else
				ppm_raw_input[channel][*p] = now + 10000 - g_ppm_input_start[channel];
			
			// median filter then 20hz LPF filter
			{
				int t;				
				#define swap(a,b) {t = a; a=b; b=t;}
				tbl[0] = ppm_raw_input[channel][0];
				tbl[1] = ppm_raw_input[channel][1];
				tbl[2] = ppm_raw_input[channel][2];
				if (tbl[0] > tbl[1])
					swap(tbl[0], tbl[1]);
				if (tbl[1] > tbl[2])
					swap(tbl[1], tbl[2]);
				if (tbl[0] > tbl[1])
					swap(tbl[0], tbl[1]);
				
				g_ppm_input[channel] = g_ppm_input[channel] * (1-alpha) + alpha * tbl[1];

				g_ppm_input_update[channel] = getus();
			}
		}
		
		EXTI_ClearITPendingBit(line_tbl[channel]);
	}
}

static void GPIO_Config(int enable_input) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// enable clocks: TIM3 TIM4 GPIOA GPIOB AFIO
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// GPIOA Configuration

	// always open B0 B1 B4 B5 as output
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// open B6 B7 B8 B9 as input or ouput according to option
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = enable_input ? GPIO_Mode_IPU : GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// remap TIM3
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

	// disable JTAG, it conflicts with TIM3, leave SW-DP enabled, 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

static void Timer_Config(int enable_input)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 9999;
	TIM_TimeBaseStructure.TIM_Prescaler = 71;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[0];
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[1];
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[2];
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[3];
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[4];
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[5];
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[6];
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[7];
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);


	// configure interrupt for input if needed
	if (enable_input)
	{
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

		EXTI_ClearITPendingBit(EXTI_Line6);
		EXTI_ClearITPendingBit(EXTI_Line7);
		EXTI_ClearITPendingBit(EXTI_Line8);
		EXTI_ClearITPendingBit(EXTI_Line9);
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);


		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;

		EXTI_InitStructure.EXTI_Line = EXTI_Line6;
		EXTI_Init(&EXTI_InitStructure);

		EXTI_InitStructure.EXTI_Line = EXTI_Line7;
		EXTI_Init(&EXTI_InitStructure);

		EXTI_InitStructure.EXTI_Line = EXTI_Line8;	
		EXTI_Init(&EXTI_InitStructure);

		EXTI_InitStructure.EXTI_Line = EXTI_Line9;
		EXTI_Init(&EXTI_InitStructure);


		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	else
	{
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}

void PPM_update_output_channel(int channel_to_update)
{
	

	if (channel_to_update & PPM_OUTPUT_CHANNEL0)
	{
		TIM_SetCompare1(TIM3, g_ppm_output[0]);
	}

	if (channel_to_update & PPM_OUTPUT_CHANNEL1)
	{
		TIM_SetCompare2(TIM3, g_ppm_output[1]);
	}

	if (channel_to_update & PPM_OUTPUT_CHANNEL2)
	{
		TIM_SetCompare3(TIM3, g_ppm_output[2]);
	}

	if (channel_to_update & PPM_OUTPUT_CHANNEL3)
	{
		TIM_SetCompare4(TIM3, g_ppm_output[3]);
	}

	if (!g_enable_input)
	{
		if (channel_to_update & PPM_OUTPUT_CHANNEL4)
		{
			TIM_SetCompare1(TIM4, g_ppm_output[4]);
		}

		if (channel_to_update & PPM_OUTPUT_CHANNEL5)
		{
			TIM_SetCompare1(TIM4, g_ppm_output[5]);
		}

		if (channel_to_update & PPM_OUTPUT_CHANNEL6)
		{
			TIM_SetCompare1(TIM4, g_ppm_output[6]);
		}

		if (channel_to_update & PPM_OUTPUT_CHANNEL7)
		{
			TIM_SetCompare1(TIM4, g_ppm_output[7]);
		}
	}
}


void PPM_init(int enable_input)
{
	int i;
	for(i=0; i<4; i++)
		ppm_raw_input[i][0] = ppm_raw_input[i][1] = ppm_raw_input[i][2] = 1520;
	
	g_enable_input = enable_input;
	GPIO_Config(enable_input);
	Timer_Config(enable_input);
	PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);	
}