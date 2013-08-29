/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ARC_LCD_H
#define __ARC_LCD_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup ARC_LCD
  * @{
  */

/** @defgroup ARC_LCD_Exported_Types
  * @{
  */

#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */
#define	LCD_XSIZE	240
#define	LCD_YSIZE	320

typedef enum 
{
    LCD_I80 = 0, 
    LCD_FSMC = 1, 
    LCD_BUSTYPE_OTHER = 2
}ARC_LCD_BusType;

/** 
  * @brief  LCD_Direction_TypeDef enumeration definition  
  */
typedef enum
{
    LCD_DIR_HORIZONTAL = 0,
    LCD_DIR_VERTICAL = 1
}LCD_Direction_TypeDef;

typedef enum
{
    LCD_ILI9320     = 0x9320,
    LCD_ILI9325     = 0x9325,
    LCD_LGDP4531    = 0x4531,
    LCD_LGDP4535    = 0x4535,
    LCD_SPFD5408    = 0x5408,
    LCD_HX8312      = 0x8312,
    LCD_HX8347A     = 0X0047,  
	LCD_SSD1289     = 0X8989,  
    LCD_OTHER       = 0xffff
}ARC_LCD_TYPE;

typedef struct
{
    ARC_LCD_TYPE LCD_Type;
    ARC_LCD_BusType LCD_BusType;
    LCD_Direction_TypeDef LCD_Direction;
}ARC_LCD_Params;

/**
  * @}
  */

/** @defgroup ARC_LCD_Exported_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup ARC_LCD_Exported_Constants
  * @{
  */

#define LCD_REG_00H                 0x00
#define LCD_REG_01H                 0x01
#define LCD_REG_02H                 0x02
#define LCD_REG_03H                 0x03
#define LCD_REG_04H                 0x04
#define LCD_REG_05H                 0x05
#define LCD_REG_06H                 0x06
#define LCD_REG_07H                 0x07
#define LCD_REG_08H                 0x08
#define LCD_REG_09H                 0x09
#define LCD_REG_0AH                 0x0A
#define LCD_REG_0CH                 0x0C
#define LCD_REG_0DH                 0x0D
#define LCD_REG_0EH                 0x0E
#define LCD_REG_0FH                 0x0F
#define LCD_REG_10H                 0x10
#define LCD_REG_11H                 0x11
#define LCD_REG_12H                 0x12
#define LCD_REG_13H                 0x13
#define LCD_REG_14H                 0x14
#define LCD_REG_15H                 0x15
#define LCD_REG_16H                 0x16
#define LCD_REG_17H                 0x17
#define LCD_REG_18H                 0x18
#define LCD_REG_19H                 0x19
#define LCD_REG_1AH                 0x1A
#define LCD_REG_1BH                 0x1B
#define LCD_REG_1CH                 0x1C
#define LCD_REG_1DH                 0x1D
#define LCD_REG_1EH                 0x1E
#define LCD_REG_1FH                 0x1F
#define LCD_REG_20H                 0x20
#define LCD_REG_21H                 0x21
#define LCD_REG_22H                 0x22
#define LCD_REG_23H                 0x23
#define LCD_REG_24H                 0x24
#define LCD_REG_25H                 0x25
#define LCD_REG_26H                 0x26
#define LCD_REG_27H                 0x27
#define LCD_REG_28H                 0x28
#define LCD_REG_29H                 0x29
#define LCD_REG_2AH                 0x2A
#define LCD_REG_2BH                 0x2B
#define LCD_REG_2CH                 0x2C
#define LCD_REG_2DH                 0x2D
#define LCD_REG_30H                 0x30
#define LCD_REG_31H                 0x31
#define LCD_REG_32H                 0x32
#define LCD_REG_33H                 0x33
#define LCD_REG_34H                 0x34
#define LCD_REG_35H                 0x35
#define LCD_REG_36H                 0x36
#define LCD_REG_37H                 0x37
#define LCD_REG_38H                 0x38
#define LCD_REG_39H                 0x39
#define LCD_REG_3AH                 0x3A
#define LCD_REG_3BH                 0x3B
#define LCD_REG_3CH                 0x3C
#define LCD_REG_3DH                 0x3D
#define LCD_REG_3EH                 0x3E
#define LCD_REG_3FH                 0x3F
#define LCD_REG_40H                 0x40
#define LCD_REG_41H                 0x41
#define LCD_REG_42H                 0x42
#define LCD_REG_43H                 0x43
#define LCD_REG_44H                 0x44
#define LCD_REG_45H                 0x45
#define LCD_REG_46H                 0x46
#define LCD_REG_47H                 0x47
#define LCD_REG_48H                 0x48
#define LCD_REG_49H                 0x49
#define LCD_REG_4AH                 0x4A
#define LCD_REG_4BH                 0x4B
#define LCD_REG_4CH                 0x4C
#define LCD_REG_4DH                 0x4D
#define LCD_REG_4EH                 0x4E
#define LCD_REG_4FH                 0x4F
#define LCD_REG_50H                 0x50
#define LCD_REG_51H                 0x51
#define LCD_REG_52H                 0x52
#define LCD_REG_53H                 0x53
#define LCD_REG_55H                 0x55
#define LCD_REG_57H                 0x57
#define LCD_REG_60H                 0x60
#define LCD_REG_61H                 0x61
#define LCD_REG_67H                 0x67
#define LCD_REG_6AH                 0x6A
#define LCD_REG_70H                 0x70
#define LCD_REG_76H                 0x76
#define LCD_REG_80H                 0x80
#define LCD_REG_81H                 0x81
#define LCD_REG_82H                 0x82
#define LCD_REG_83H                 0x83
#define LCD_REG_84H                 0x84
#define LCD_REG_85H                 0x85
#define LCD_REG_86H                 0x86
#define LCD_REG_87H                 0x87
#define LCD_REG_88H                 0x88
#define LCD_REG_89H                 0x89
#define LCD_REG_8BH                 0x8B
#define LCD_REG_8CH                 0x8C
#define LCD_REG_8DH                 0x8D
#define LCD_REG_8FH                 0x8F
#define LCD_REG_90H                 0x90
#define LCD_REG_91H                 0x91
#define LCD_REG_92H                 0x92
#define LCD_REG_93H                 0x93
#define LCD_REG_94H                 0x94
#define LCD_REG_95H                 0x95
#define LCD_REG_96H                 0x96
#define LCD_REG_97H                 0x97
#define LCD_REG_98H                 0x98
#define LCD_REG_99H                 0x99
#define LCD_REG_9AH                 0x9A
#define LCD_REG_9DH                 0x9D
#define LCD_REG_C0H                 0xC0
#define LCD_REG_C1H                 0xC1
#define LCD_REG_E3H                 0xE3
#define LCD_REG_E5H                 0xE5
#define LCD_REG_E7H                 0xE7
#define LCD_REG_EFH                 0xEF

#define LCD_COLOR_WHITE             0xFFFF
#define LCD_COLOR_BLACK             0x0000
#define LCD_COLOR_GREY              0xF7DE
#define LCD_COLOR_BLUE              0x001F
#define LCD_COLOR_BLUE2             0x051F
#define LCD_COLOR_RED               0xF800
#define LCD_COLOR_MAGENTA           0xF81F
#define LCD_COLOR_GREEN             0x07E0
#define LCD_COLOR_CYAN              0x7FFF
#define LCD_COLOR_YELLOW            0xFFE0



/**
  * @}
  */

/** @defgroup ARC_LCD_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup ARC_LCD_Exported_Functions
  * @{
  */
ARC_LCD_Params *ARC_LCD_get_param(void);
void ARC_LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue);
uint16_t ARC_LCD_ReadReg(uint16_t LCD_Index);
int32_t ARC_LCD_Init(void);
void ARC_LCD_On(void);
void ARC_LCD_ShowString(int32_t x, int32_t y, const uint8_t *lcd_font);
void ARC_LCD_Clear(uint16_t Color);
void ARC_LCD_DrawBigPoint(uint16_t x,uint16_t y, uint16_t pointColor);
void ARC_LCD_DrawCross(uint16_t x, uint16_t y, uint16_t pointColor);
void LCD_L0_DrawBitmap(int32_t x0, int32_t y0, int32_t xsize, int32_t ysize, int32_t BitsPerPixel, int32_t BytesPerLine,
                        const uint8_t * pData, int32_t Diff, const uint16_t* pTrans);
void ARC_LCD_SetPixelIndex(int32_t x,int32_t y,int32_t ColorIndex);
uint32_t ARC_LCD_GetPixelIndex(int32_t x, int32_t y);
void ARC_LCD_SetCursor(uint16_t x, uint16_t y);
void ARC_LCD_WriteGRAM_Same(uint16_t LCD_RegValue, uint32_t NumValue); 
void ARC_LCD_FillRect  (int32_t x0, int32_t y0, int32_t x1, int32_t y1);
void ARC_LCD_SetDisplayWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);
void ARC_LCD_SetMultiPixelsIndex(uint16_t x, uint16_t y, uint16_t const *LCD_RegValue, uint32_t NumValue);
void ARC_LCD_WriteGRAM(const uint16_t *LCD_RegValue, uint32_t NumValue);
void ARC_LCD_DrawLine(int32_t x1, int32_t y1, int32_t x2, int32_t y2,int32_t nColor);
void GUI_Pieslice(uint32_t x, uint32_t y, uint32_t r, uint32_t stangle, uint32_t endangle, uint32_t color);
/**
  * @}
  */


#ifdef __cplusplus
}
#endif


#endif /* __ARC_LCD_H */

/**
  * @}
  */ 

/**
  * @}
  */ 
 

/******************* (C) www.armrunc.com *****END OF FILE****/
