#include "PPM.h"
#include "timer.h"
#include "config.h"
#include <stm32f10x_exti.h>
#include <stm32f10x_tim.h>
#include <misc.h>

u32 g_ppm_input_start[6];
float g_ppm_input[6];
int ppm_raw_input[6][3];						//	[channel][v1, v2, v3], median filter with window-size=3
int ppm_raw_input_counter[6] = {0};
int64_t g_ppm_input_update[6] = {0};
u16 g_ppm_output[8] = {0};
int tbl[3];


// PPM input handler
static void PPM_EXTI_Handler(void)
{
	while(1)
	{
		int channel = -1;
#if PCB_VERSION == 1
	static int pin_tbl[6] = {GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15};
	static int line_tbl[6] = {EXTI_Line10, EXTI_Line11, EXTI_Line12, EXTI_Line13, EXTI_Line14, EXTI_Line15};
		if (EXTI_GetITStatus(EXTI_Line10) != RESET)
			channel = 0;
		if (EXTI_GetITStatus(EXTI_Line11) != RESET)
			channel = 1;
		if (EXTI_GetITStatus(EXTI_Line12) != RESET)
			channel = 2;
		if (EXTI_GetITStatus(EXTI_Line13) != RESET)
			channel = 3;
		if (EXTI_GetITStatus(EXTI_Line14) != RESET)
			channel = 4;
		if (EXTI_GetITStatus(EXTI_Line15) != RESET)
			channel = 5;
#elif PCB_VERSION == 2
	static int pin_tbl[6] = {GPIO_Pin_15, GPIO_Pin_14, GPIO_Pin_13, GPIO_Pin_12, GPIO_Pin_11, GPIO_Pin_10};
	static int line_tbl[6] = {EXTI_Line15, EXTI_Line14, EXTI_Line13, EXTI_Line12, EXTI_Line11, EXTI_Line10};
		if (EXTI_GetITStatus(EXTI_Line10) != RESET)
			channel = 5;
		if (EXTI_GetITStatus(EXTI_Line11) != RESET)
			channel = 4;
		if (EXTI_GetITStatus(EXTI_Line12) != RESET)
			channel = 3;
		if (EXTI_GetITStatus(EXTI_Line13) != RESET)
			channel = 2;
		if (EXTI_GetITStatus(EXTI_Line14) != RESET)
			channel = 1;
		if (EXTI_GetITStatus(EXTI_Line15) != RESET)
			channel = 0;
#endif
		if (channel == -1)
			break;
	
		if(GPIOB->IDR & pin_tbl[channel])
			g_ppm_input_start[channel] = TIM_GetCounter(TIM4);
		else
		{
			const float RC = 1.0f/(2*3.1415926 * 200);	// 200hz low pass filter
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
			
			// median filter then LPF filter
			if(0)
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

			}
			else
			{
				g_ppm_input[channel] = ppm_raw_input[channel][*p];
			}
			g_ppm_input_update[channel] = getus();
		}
		
		EXTI_ClearITPendingBit(line_tbl[channel]);
	}
}

void EXTI15_10_IRQHandler(void)
{
	PPM_EXTI_Handler();
}

static void GPIO_Config(int enable_input) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// enable clocks: TIM3 TIM4 GPIOA GPIOB AFIO
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// open B0 B1 B4 B5 B6 B7 B8 B9 as output
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// open B10-B15 as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
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
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[1];
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[2];
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[3];
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[4];
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[5];
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[6];
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[7];
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);


	// configure interrupts

	EXTI_ClearITPendingBit(EXTI_Line10);
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);


	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line11;	
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line14;
	EXTI_Init(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_Init(&EXTI_InitStructure);


	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void PPM_update_output_channel(int channel_to_update)
{
#if PCB_VERSION == 1
	if (channel_to_update & PPM_OUTPUT_CHANNEL0)
		TIM_SetCompare3(TIM3, g_ppm_output[0]);		// PB0

	if (channel_to_update & PPM_OUTPUT_CHANNEL1)
		TIM_SetCompare4(TIM3, g_ppm_output[1]);		// PB1

	if (channel_to_update & PPM_OUTPUT_CHANNEL2)
		TIM_SetCompare4(TIM4, g_ppm_output[2]);		// PB9

	if (channel_to_update & PPM_OUTPUT_CHANNEL3)
		TIM_SetCompare3(TIM4, g_ppm_output[3]);		// PB8

	if (channel_to_update & PPM_OUTPUT_CHANNEL4)
		TIM_SetCompare2(TIM4, g_ppm_output[4]);		// PB7

	if (channel_to_update & PPM_OUTPUT_CHANNEL5)
		TIM_SetCompare1(TIM4, g_ppm_output[5]);		// PB6

	if (channel_to_update & PPM_OUTPUT_CHANNEL6)
		TIM_SetCompare2(TIM3, g_ppm_output[6]);		// PB5

	if (channel_to_update & PPM_OUTPUT_CHANNEL7)
		TIM_SetCompare1(TIM3, g_ppm_output[7]);		// PB4

#elif PCB_VERSION == 2

	if (channel_to_update & PPM_OUTPUT_CHANNEL0)
		TIM_SetCompare1(TIM3, g_ppm_output[0]);		// PB4

	if (channel_to_update & PPM_OUTPUT_CHANNEL1)
		TIM_SetCompare2(TIM3, g_ppm_output[1]);		// PB5

	if (channel_to_update & PPM_OUTPUT_CHANNEL2)
		TIM_SetCompare1(TIM4, g_ppm_output[2]);		// PB6

	if (channel_to_update & PPM_OUTPUT_CHANNEL3)
		TIM_SetCompare2(TIM4, g_ppm_output[3]);		// PB7

	if (channel_to_update & PPM_OUTPUT_CHANNEL4)
		TIM_SetCompare3(TIM4, g_ppm_output[4]);		// PB8

	if (channel_to_update & PPM_OUTPUT_CHANNEL5)
		TIM_SetCompare4(TIM4, g_ppm_output[5]);		// PB9

	if (channel_to_update & PPM_OUTPUT_CHANNEL6)
		TIM_SetCompare4(TIM3, g_ppm_output[6]);		// PB1

	if (channel_to_update & PPM_OUTPUT_CHANNEL7)
		TIM_SetCompare3(TIM3, g_ppm_output[7]);		// PB0
#endif
}


void PPM_init(int enable_input)
{
	int i;
	for(i=0; i<4; i++)
		ppm_raw_input[i][0] = ppm_raw_input[i][1] = ppm_raw_input[i][2] = 1520;
	
	GPIO_Config(enable_input);
	Timer_Config(enable_input);
	PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);	
}