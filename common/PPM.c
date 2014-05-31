#include "PPM.h"
#include "timer.h"
#include "config.h"
#include "../mcu.h"

#define PPM_6 1

u32 g_ppm_input_start[6];
float g_ppm_input[6];
int64_t g_ppm_input_update[6] = {0};
#if QUADCOPTER == 0
u16 g_ppm_output[8] = {1520, 1520, THROTTLE_STOP, 1520, 1520, 1520, 1520, 1520};
#else
u16 g_ppm_output[8] = {THROTTLE_STOP, THROTTLE_STOP, THROTTLE_STOP, THROTTLE_STOP, THROTTLE_STOP, THROTTLE_STOP, THROTTLE_STOP, THROTTLE_STOP};
#endif


// PPM input handler
static void PPM_EXTI_Handler(void)
{
	while(1)
	{
		int channel = -1;
#if PCB_VERSION == 1 ||  PCB_VERSION == 3
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
			u32 now = TIM_GetCounter(TIM4);
			if (now > g_ppm_input_start[channel])
				g_ppm_input[channel]= now - g_ppm_input_start[channel];
			else
#if QUADCOPTER == 1
				g_ppm_input[channel]= now + 3000 - g_ppm_input_start[channel];
#else
				g_ppm_input[channel]= now + 10000 - g_ppm_input_start[channel];
#endif			
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
	
	#if PPM_6 == 1
	GPIO_InitStructure.GPIO_Pin &= ~(GPIO_Pin_0 | GPIO_Pin_1);
	#endif
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
	#if QUADCOPTER == 1
	TIM_TimeBaseStructure.TIM_Period = 2999;
	#else
	TIM_TimeBaseStructure.TIM_Period = 9999;
	#endif
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
	
	#if PPM_6 == 1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	#endif



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

#elif PCB_VERSION == 2 ||  PCB_VERSION == 3

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

	#if PPM_6 == 0
	if (channel_to_update & PPM_OUTPUT_CHANNEL6)
		TIM_SetCompare4(TIM3, g_ppm_output[6]);		// PB1

	if (channel_to_update & PPM_OUTPUT_CHANNEL7)
		TIM_SetCompare3(TIM3, g_ppm_output[7]);		// PB0
	#endif
#endif
}


void PPM_init(int enable_input)
{	
	GPIO_Config(enable_input);
	Timer_Config(enable_input);
	PPM_update_output_channel(PPM_OUTPUT_CHANNEL_ALL);
}
