#pragma once

#ifdef STM32F1
#ifndef __cplusplus
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif

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
#include <stdint.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f4xx_syscfg.h>
#include <misc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_sdio.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_flash.h>
#endif
