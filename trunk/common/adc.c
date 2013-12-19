#include "adc.h"

static void ADC1_GPIO_Config(void)
{	
	
}

static void ADC1_Mode_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

	// Configure PA.04  as analog input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ADC1 configuration
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
		
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	
}

void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

int ADC1_Read(void)
{
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	return ADC_GetConversionValue(ADC1);
}
