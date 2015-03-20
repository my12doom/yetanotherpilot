#include "PPM.h"
#include "common.h"
#include "timer.h"
#include "build.h"
#include "mcu.h"
#include <math.h>
#include "param.h"

#define OC 12

uint32_t g_pwm_input_start[6];
float g_pwm_input[6];
float pwm_static[8][2];
int64_t g_pwm_input_update[6] = {0};

int last_high_tim = -1;
int ppm_channel_id = 0;
int ppm_channel_count = 0;

// initialize this before calling ppm_init()!
uint16_t g_ppm_output[8] = {0};

static float f_min(float a, float b)
{
	return a > b ? b : a;
}
static float f_max(float a, float b)
{
	return a > b ? a : b;
}


int handle_ppm(int now)
{
	float delta = 0;
	if (last_high_tim < 0)
	{
		last_high_tim = now;
		return 0;
	}
	
	if (now > last_high_tim)
		delta = now - last_high_tim;
	else
		delta = now + 60000 - last_high_tim;

	last_high_tim = now;

	if (delta > 2100)
	{
		ppm_channel_count = ppm_channel_id;
		ppm_channel_id = 0;
		TRACE("        %.0f\r", delta);
	}
	else if (ppm_channel_id < sizeof(g_pwm_input)/sizeof(g_pwm_input[0]))
	{
		g_pwm_input[ppm_channel_id] = delta;
		TRACE("%.0f,", g_pwm_input[ppm_channel_id-1]);

		g_pwm_input_update[ppm_channel_id] = getus_nodelay();
		pwm_static[ppm_channel_id][0] = f_min(pwm_static[ppm_channel_id][0], g_pwm_input[ppm_channel_id]);
		pwm_static[ppm_channel_id][1] = f_max(pwm_static[ppm_channel_id][1], g_pwm_input[ppm_channel_id]);

		ppm_channel_id++;
	}
	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		handle_ppm(TIM_GetCounter(TIM5));
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

static void GPIO_Config() 
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	// enable clocks: TIM3 TIM4 GPIOA GPIOB AFIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

	// open A0 A1 A2 A3 A6 A7 as output
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// open PC5 as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
}

static void Timer_Config()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 3000*OC-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84/OC-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseStructure.TIM_Period = 60000;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM5, ENABLE);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[0];
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[1];
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[2];
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[3];
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[4];
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = g_ppm_output[5];
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	// configure interrupts
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_ClearITPendingBit(EXTI_Line5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void PPM_update_output_channel(int channel_to_update)
{
	if (channel_to_update & PPM_OUTPUT_CHANNEL0)
		TIM_SetCompare1(TIM2, g_ppm_output[0]*OC);		// PA0

	if (channel_to_update & PPM_OUTPUT_CHANNEL1)
		TIM_SetCompare2(TIM2, g_ppm_output[1]*OC);		// PA1

	if (channel_to_update & PPM_OUTPUT_CHANNEL2)
		TIM_SetCompare3(TIM2, g_ppm_output[2]*OC);		// PA2

	if (channel_to_update & PPM_OUTPUT_CHANNEL3)
		TIM_SetCompare4(TIM2, g_ppm_output[3]*OC);		// PA3

	if (channel_to_update & PPM_OUTPUT_CHANNEL4)
		TIM_SetCompare1(TIM3, g_ppm_output[4]*OC);		// PA6

	if (channel_to_update & PPM_OUTPUT_CHANNEL5)
		TIM_SetCompare2(TIM3, g_ppm_output[5]*OC);		// PA7
}

void PPM_reset_static()
{
	int i;
	for(i=0; i<8; i++)
	{
		pwm_static[i][0] = 99999;		// min
		pwm_static[i][1] = 0;			// max
	}
}

void PPM_init()
{
	PPM_reset_static();
	GPIO_Config();
	Timer_Config();
	PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);
}
