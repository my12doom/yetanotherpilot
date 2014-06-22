#include "adc.h"
#include "../mcu.h"

static void ADC1_GPIO_Config(void)
{	
	
}

static void ADC1_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
#ifdef STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
#endif
#ifdef STM32F4
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
#endif


	// ADC1 configuration
#ifdef STM32F1
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_Init(ADC1, &ADC_InitStructure);
#endif

#ifdef STM32F4
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	ADC_Init(ADC1, &ADC_InitStructure);
#endif
	
#ifdef STM32F1
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
#endif
#ifdef STM32F4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_480Cycles);
#endif		
	ADC_Cmd(ADC1, ENABLE);
	
#ifdef STM32F1
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
#endif
	
}

int ADC1_SelectChannel(uint16_t ADC_Channel)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	if (ADC_Channel > 9)
		return 0;

	// Configure GPIO as analog input
	GPIO_InitStructure.GPIO_Pin = (1 << (ADC_Channel%8));
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ADC_Channel>8?GPIOB:GPIOA, &GPIO_InitStructure);

#ifdef STM32F1
	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_239Cycles5);
#endif
#ifdef STM32F4
	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_480Cycles);
#endif		

	return 1;
}

void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

int ADC1_Read(void)
{
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
#ifdef STM32F1
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
#endif
#ifdef STM32F4
	ADC_SoftwareStartConv(ADC1);
#endif

	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	return ADC_GetConversionValue(ADC1);
}
