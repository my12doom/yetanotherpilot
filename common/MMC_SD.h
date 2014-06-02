/*******************************************************************/
/*          SD diriver for  MP3 Player                             */
/*                                                                 */
/* Platform   : AVRStudio4.13 b528 + WinAVR20070525                */
/*              optimize -0s                                       */
/* Author     : bozai(Zhang Qibo)                                  */
/* E-mail     : sudazqb@163.com                                    */
/* MSN        : zhangqibo_1985@hotmail.com                         */
/* Date       : 2006-06-16                                         */
/*******************************************************************/
/*  20080101: modify codes for STM32F10x Cortex-M3 controller */
/*  20071208: modify codes for ARM platform (AT91SAM7S256)  */
/*  2007-10-21: Rewrite some function, now only suply 4 functions  */
/*  2007-10-18: Adjust some time & retry count for compatibility   */
/*              consideration                                      */
/*  2007-06-16: After reading the spec. in detail, I found some    */
/*              of the code don't meet the spec., that is after    */
/*              the last SPI transaction, it need an extra 8 CLK   */
/*              to finish it's work                                */
/*  2007-05-04: add read capacity function                         */
/*  2007-04-21:                                                    */
/*  Enable some code incase that when SD reset                     */
/*  faild program can't jump the loop                              */
/*******************************************************************/

#ifndef __MMC_SD_h__
#define __MMC_SD_h__


#define uint8  unsigned char
#define  int8    signed char
#define uint16 unsigned short
#define  int16   signed short
#define uint32 unsigned int
#define  int32   signed int

/*********************************************************/
#define MMC_SD_CS			GPIO_Pin_2
#define MMC_SD_CS_PORT		GPIOB
/******************************/

#define MMC_SD_SCK			GPIO_Pin_5
#define MMC_SD_SCK_PORT	GPIOA

#define MMC_SD_MISO		GPIO_Pin_6
#define MMC_SD_MISO_PORT	GPIOA

#define MMC_SD_MOSI		GPIO_Pin_7
#define MMC_SD_MOSI_PORT	GPIOA

//#define MMC_SD_SPI_SAME	1	/* it indicate that all SPI signal are in the same port*/
#define MMC_SD_SPI_PORT	GPIOA
/*****************************/

#ifdef STM32F1
#define MMC_SD_GPIO_PORTS	RCC_APB2Periph_GPIOA
#endif
#ifdef STM32F4
#define MMC_SD_GPIO_PORTS	RCC_AHB1Periph_GPIOA
#endif
/****************************************************************/


uint8 MMC_SD_Init(void);
uint8 MMC_SD_ReadSingleBlock(uint32 sector, uint8* buffer);
uint8 MMC_SD_WriteSingleBlock(uint32 sector, const uint8* buffer);
uint32 MMC_SD_ReadCapacity(void);

#endif
