#pragma once

#ifdef STM32F1
#include <misc.h>
#include <stm32f10x.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_sdio.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_flash.h>
#endif

#ifdef STM32F4
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_usart.h>
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_sd.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_spi.h>
// #include <stm32f4xx_hal_exti.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_tim.h>
#include <stm32f4xx_hal_flash.h>
#endif
