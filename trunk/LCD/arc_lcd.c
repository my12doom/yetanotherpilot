/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_fsmc.h"
#include "ARC_LCD.h"
#include "ARC_Font.h"
#include "HzLib.h"


/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup ARC_LCD
  * @{
  */ 

/** @defgroup ARC_LCD_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup ARC_LCD_Private_Defines
  * @{
  */


#define ARC_LCD_BL_RESET()      //GPIOC->BRR = GPIO_Pin_5
#define ARC_LCD_BL_SET()        //GPIOC->BSRR = GPIO_Pin_5



/**
  * @}
  */ 

/** @defgroup ARC_LCD_Private_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup ARC_LCD_Private_Variables
  * @{
  */
static ARC_LCD_Params ARC_LCD_Param;
int LCD_COLOR;

/**
  * @}
  */

/** @defgroup ARC_LCD_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup ARC_LCD_Private_Functions
  * @{
  */

/**
  * @brief  延时函数
  * @param  ms 延时参数，单位为毫秒
  * @retval None
  */
void delay_ms(uint16_t ms)    
{ 
	uint16_t i,j; 
	for( i = 0; i < ms; i++ )
	{ 
		for( j = 0; j < 1141; j++ );
	}
} 
/**
  * @brief  get the pointer to the LCD parameters.
  * @param  None
  * @retval the pointer to the LCD parameters
  */
ARC_LCD_Params *ARC_LCD_get_param(void)
{
    return &ARC_LCD_Param;
}
/**
  * @brief  write register index.
  * @param  irData, the index register value to be written.
  * @retval None
  */
#define ARC_LCD_WriteRegIndex(irData)\
{\
	LCD_REG = irData;\
}

/**
  * @brief  write register data.
  * @param  Data, the data to be written.
  * @retval None
  */
#define ARC_LCD_WriteRegData(RegData)\
{\
	LCD_RAM = RegData;\
}

/**
  * @brief  write register data, becareful, no CS operattion to speed up.
  * @param  Data, the data to be written.
  * @retval None
  */
#define ARC_LCD_WriteRegData_NoCS(RegData)\
{\
	LCD_RAM = RegData;\
}


/**
  * @brief  read register data, becareful.
  * @param  Data, the data read from IR.
  * @retval None
  */
#define ARC_LCD_ReadRegData(RegData)\
{\
	RegData = LCD_REG;\
}

/**
  * @brief  read GRAM data, becareful, no CS or RD operation.
  * @param  Data, the data read from IR.
  * @retval None
  */

#define ARC_LCD_ReadGRAMData(GRAMData)\
{\
    GRAMData = LCD_RAM;\
}

/**
  * @brief  Sets the cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position. 
  * @retval None
  */
__inline void ARC_LCD_SetCursor(uint16_t x, uint16_t y)
{
	if(ARC_LCD_Param.LCD_Type == LCD_HX8347A)
    {
        ARC_LCD_WriteReg(LCD_REG_03H, x & 0xff);
        ARC_LCD_WriteReg(LCD_REG_02H, (x >> 8) & 0xff);
        ARC_LCD_WriteReg(LCD_REG_07H, y & 0xff);
        ARC_LCD_WriteReg(LCD_REG_06H, (y >> 8) & 0xff);
    }
	else if(ARC_LCD_Param.LCD_Type == LCD_SSD1289)
	{
#if (LCD_XSIZE == 320) && (LCD_YSIZE == 240)
		ARC_LCD_WriteReg(LCD_REG_4EH, y);
        ARC_LCD_WriteReg(LCD_REG_4FH, x);
#else
		ARC_LCD_WriteReg(LCD_REG_4EH, x);
        ARC_LCD_WriteReg(LCD_REG_4FH, y);
#endif
	}
    else
    {
		ARC_LCD_WriteReg(LCD_REG_20H, x);
        ARC_LCD_WriteReg(LCD_REG_21H, y);
    }
}

/**
  * @brief  Write LCD register.
  * @param  LCD_Index, the register index to be written.
  * @param  LCD_Data, value to be written.
  * @retval None.
  */
__inline void ARC_LCD_WriteReg(uint16_t LCD_Index, uint16_t LCD_Data)
{
    ARC_LCD_WriteRegIndex(LCD_Index);
    ARC_LCD_WriteRegData(LCD_Data);
}

/**
  * @brief  Write to the LCD RAM a number of value.
  * @param  *LCD_RegValue, the pointers to the values to be written.
  * @param  NumValue, the number of values to be written.
  * @retval None.
  */
void ARC_LCD_WriteGRAM(const uint16_t *LCD_RegValue, uint32_t NumValue)
{
	const uint16_t *dst = LCD_RegValue + NumValue;

	ARC_LCD_WriteRegIndex(LCD_REG_22H);
	
	for(; LCD_RegValue<dst; LCD_RegValue++)
	{
			ARC_LCD_WriteRegData_NoCS(*LCD_RegValue);
	}
}

/**
  * @brief  Write to the LCD RAM a number of value.
  * @param  LCD_RegValue, the values to be written.
  * @param  NumValue, the number of values to be written.
  * @retval None.
  */
__inline void ARC_LCD_WriteGRAM_Same(uint16_t LCD_RegValue, uint32_t NumValue)
{
    ARC_LCD_WriteRegIndex(LCD_REG_22H);

    for(; NumValue; NumValue--)
    {
        ARC_LCD_WriteRegData_NoCS(LCD_RegValue);
    }
}

/**
  * @brief  Write to the LCD RAM a number of value.
  * @param  *LCD_RegValue, the pointers to the values to be written.
  * @param  NumValue, the number of values to be written.
  * @retval None.
  */
__inline void ARC_LCD_SetMultiPixelsIndex(uint16_t x, uint16_t y, uint16_t const *LCD_RegValue, uint32_t NumValue)
{
    ARC_LCD_SetCursor(x, y);
    ARC_LCD_WriteGRAM(LCD_RegValue, NumValue);
}

/**
  * @brief  Read LCD register.
  * @param  LCD_Reg, the register index to be read.
  * @retval value read from LCD.
  */
__inline uint16_t ARC_LCD_ReadReg(uint16_t LCD_Index)
{
    LCD_REG = LCD_Index;
	return LCD_RAM;
}

/**
  * @brief  Sets a display window
  * @param  x1: specifies the X top left position.
  * @param  y1: specifies the Y top left position.
  * @param  x2: specifies the X buttom right position.
  * @param  y2: specifies the Y buttom right position.
  * @retval None
  */
__inline void ARC_LCD_SetDisplayWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{
	if(ARC_LCD_Param.LCD_Type == LCD_SSD1289)
	{
#if (LCD_XSIZE == 240) && (LCD_YSIZE == 320)
        ARC_LCD_WriteReg(LCD_REG_4EH, x0);
        ARC_LCD_WriteReg(LCD_REG_4FH, y0);
        ARC_LCD_WriteReg(LCD_REG_44H,(x0+(x1<<8)));      //x坐标起始（低八位）和结束（高八位）
        ARC_LCD_WriteReg(LCD_REG_45H,y0);                 //y坐标起始           
        ARC_LCD_WriteReg(LCD_REG_46H,y1);                 //y坐标结束
#else
        ARC_LCD_WriteReg(LCD_REG_4EH, y0);
        ARC_LCD_WriteReg(LCD_REG_4FH, x0);
        ARC_LCD_WriteReg(LCD_REG_44H,(y0+(y1<<8)));      //x坐标起始（低八位）和结束（高八位）
        ARC_LCD_WriteReg(LCD_REG_45H,x0);                 //y坐标起始           
        ARC_LCD_WriteReg(LCD_REG_46H,x1);                 //y坐标结束

#endif
	}
    else
    {
        /* Horizontal GRAM Start Address */
        ARC_LCD_WriteReg(LCD_REG_50H, x0);
        /* Horizontal GRAM End Address */
        ARC_LCD_WriteReg(LCD_REG_51H, x1);
        /* Vertical GRAM Start Address */
        ARC_LCD_WriteReg(LCD_REG_52H, y0);
        /* Vertical GRAM End Address */
        ARC_LCD_WriteReg(LCD_REG_53H, y1);
        ARC_LCD_SetCursor(x0, y0);
    }
}

/**
  * @brief  initialize panel.
  * @param  None.
  * @retval None
  */
void ARC_HX8347A_Init(LCD_Direction_TypeDef lcd_dir)
{   
    //=========Initial HX8347  for INL2.8"QVGA Panel======// 
	delay_ms(150);

    ARC_LCD_WriteReg(0x0042,0x0008);   
    //Gamma setting  
    ARC_LCD_WriteReg(0x0046,0x00B4); 
    ARC_LCD_WriteReg(0x0047,0x0043); 
    ARC_LCD_WriteReg(0x0048,0x0013); 
    ARC_LCD_WriteReg(0x0049,0x0047); 
    ARC_LCD_WriteReg(0x004A,0x0014); 
    ARC_LCD_WriteReg(0x004B,0x0036); 
    ARC_LCD_WriteReg(0x004C,0x0003); 
    ARC_LCD_WriteReg(0x004D,0x0046); 
    ARC_LCD_WriteReg(0x004E,0x0005);  
    ARC_LCD_WriteReg(0x004F,0x0010);  
    ARC_LCD_WriteReg(0x0050,0x0008);  
    ARC_LCD_WriteReg(0x0051,0x000a);  

    //Window Setting 
    ARC_LCD_WriteReg(0x0002,0x0000); 
    ARC_LCD_WriteReg(0x0003,0x0000); 
    ARC_LCD_WriteReg(0x0004,0x0000); 
    ARC_LCD_WriteReg(0x0005,0x00EF); 
    ARC_LCD_WriteReg(0x0006,0x0000); 
    ARC_LCD_WriteReg(0x0007,0x0000); 
    ARC_LCD_WriteReg(0x0008,0x0001); 
    ARC_LCD_WriteReg(0x0009,0x003F); 

	delay_ms(10);

    ARC_LCD_WriteReg(0x0001,0x0006); 
    ARC_LCD_WriteReg(0x0016,0x00C8);   
    ARC_LCD_WriteReg(0x0023,0x0095); 
    ARC_LCD_WriteReg(0x0024,0x0095); 
    ARC_LCD_WriteReg(0x0025,0x00FF); 
    ARC_LCD_WriteReg(0x0027,0x0002); 
    ARC_LCD_WriteReg(0x0028,0x0002); 
    ARC_LCD_WriteReg(0x0029,0x0002); 
    ARC_LCD_WriteReg(0x002A,0x0002); 
    ARC_LCD_WriteReg(0x002C,0x0002); 
    ARC_LCD_WriteReg(0x002D,0x0002); 

    ARC_LCD_WriteReg(0x003A,0x0001);  
    ARC_LCD_WriteReg(0x003B,0x0001);  
    ARC_LCD_WriteReg(0x003C,0x00F0);    
    ARC_LCD_WriteReg(0x003D,0x0000); 

	delay_ms(20);

    ARC_LCD_WriteReg(0x0035,0x0038); 
    ARC_LCD_WriteReg(0x0036,0x0078); 
    ARC_LCD_WriteReg(0x003E,0x0038); 
    ARC_LCD_WriteReg(0x0040,0x000F); 
    ARC_LCD_WriteReg(0x0041,0x00F0);  

    ARC_LCD_WriteReg(0x0038,0x0000); 

    // Power Setting 
    ARC_LCD_WriteReg(0x0019,0x0049);  
    ARC_LCD_WriteReg(0x0093,0x000A); 

    delay_ms(10);               

    ARC_LCD_WriteReg(0x0020,0x0020);   
    ARC_LCD_WriteReg(0x001D,0x0003);   
    ARC_LCD_WriteReg(0x001E,0x0000);  

    ARC_LCD_WriteReg(0x001F,0x0009);   

    ARC_LCD_WriteReg(0x0044,0x0053);  
    ARC_LCD_WriteReg(0x0045,0x0010);   


    delay_ms(10);   

    ARC_LCD_WriteReg(0x001C,0x0004);  

	delay_ms(20);  

    ARC_LCD_WriteReg(0x0043,0x0080);    

	delay_ms(5);

    ARC_LCD_WriteReg(0x001B,0x000a);    

	delay_ms(40);  

    ARC_LCD_WriteReg(0x001B,0x0012);    

	delay_ms(40);   
    //Display On Setting 

    ARC_LCD_WriteReg(0x0090,0x007F); 
    ARC_LCD_WriteReg(0x0026,0x0004); 

    delay_ms(40); 

    ARC_LCD_WriteReg(0x0026,0x0024); 
    ARC_LCD_WriteReg(0x0026,0x002C); 

	delay_ms(40);    
    ARC_LCD_WriteReg(0x0070,0x0008); 
    ARC_LCD_WriteReg(0x0026,0x003C);  
    ARC_LCD_WriteReg(0x0057,0x0002); 
    ARC_LCD_WriteReg(0x0055,0x0000); 
    ARC_LCD_WriteReg(0x0057,0x0000);
 } 

/**
  * @brief  initialize panel.
  * @param  None.
  * @retval None
  */
void ARC_ILI9325_Init(LCD_Direction_TypeDef lcd_dir)
{

    /* Start Initial Sequence ------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_00H, 0x0001); /* Start internal OSC. */
    ARC_LCD_WriteReg(LCD_REG_01H, 0x0100); /* Set SS and SM bit */
    ARC_LCD_WriteReg(LCD_REG_02H, 0x0700); /* Set 1 line inversion */
    ARC_LCD_WriteReg(LCD_REG_03H, (1<<12)|(3<<4)|(0<<3)); /* Set GRAM write direction and BGR=1. */
    ARC_LCD_WriteReg(LCD_REG_04H, 0x0000); /* Resize register */
    ARC_LCD_WriteReg(LCD_REG_08H, 0x0202); /* Set the back porch and front porch */
    ARC_LCD_WriteReg(LCD_REG_09H, 0x0000); /* Set non-display area refresh cycle ISC[3:0] */
    ARC_LCD_WriteReg(LCD_REG_0AH, 0x0000); /* FMARK function */
    ARC_LCD_WriteReg(LCD_REG_0CH, 0x0000); /* RGB interface setting */
    ARC_LCD_WriteReg(LCD_REG_0DH, 0x0000); /* Frame marker Position */
    ARC_LCD_WriteReg(LCD_REG_0FH, 0x0000); /* RGB interface polarity */

    /* Power On sequence -----------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_10H, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    ARC_LCD_WriteReg(LCD_REG_11H, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    ARC_LCD_WriteReg(LCD_REG_12H, 0x0000); /* VREG1OUT voltage */
    ARC_LCD_WriteReg(LCD_REG_13H, 0x0000); /* VDV[4:0] for VCOM amplitude */
	delay_ms(200);
    ARC_LCD_WriteReg(LCD_REG_10H, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    ARC_LCD_WriteReg(LCD_REG_11H, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
	delay_ms(50);
    ARC_LCD_WriteReg(LCD_REG_12H, 0x0139); /* VREG1OUT voltage */
	delay_ms(50);
    ARC_LCD_WriteReg(LCD_REG_13H, 0x1d00); /* VDV[4:0] for VCOM amplitude */
    ARC_LCD_WriteReg(LCD_REG_29H, 0x0013); /* VCM[4:0] for VCOMH */
	delay_ms(50);
    ARC_LCD_WriteReg(LCD_REG_20H, 0x0000); /* GRAM horizontal Address */
    ARC_LCD_WriteReg(LCD_REG_21H, 0x0000); /* GRAM Vertical Address */

    /* Adjust the Gamma Curve (ILI9325)---------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_30H, 0x0007);
    ARC_LCD_WriteReg(LCD_REG_31H, 0x0302);
    ARC_LCD_WriteReg(LCD_REG_32H, 0x0105);
    ARC_LCD_WriteReg(LCD_REG_35H, 0x0206);
    ARC_LCD_WriteReg(LCD_REG_36H, 0x0808);
    ARC_LCD_WriteReg(LCD_REG_37H, 0x0206);
    ARC_LCD_WriteReg(LCD_REG_38H, 0x0504);
    ARC_LCD_WriteReg(LCD_REG_39H, 0x0007);
    ARC_LCD_WriteReg(LCD_REG_3CH, 0x0105);
    ARC_LCD_WriteReg(LCD_REG_3DH, 0x0808);

    /* Set GRAM area ---------------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_50H, 0x0000); /* Horizontal GRAM Start Address */
    ARC_LCD_WriteReg(LCD_REG_51H, 0x00EF); /* Horizontal GRAM End Address */
    ARC_LCD_WriteReg(LCD_REG_52H, 0x0000); /* Vertical GRAM Start Address */
    ARC_LCD_WriteReg(LCD_REG_53H, 0x013F); /* Vertical GRAM End Address */

    ARC_LCD_WriteReg(LCD_REG_60H,  0xA700); /* Gate Scan Line(GS=1, scan direction is G320~G1) */
    ARC_LCD_WriteReg(LCD_REG_61H,  0x0001); /* NDL,VLE, REV */
    ARC_LCD_WriteReg(LCD_REG_6AH, 0x0000); /* set scrolling line */

    /* Partial Display Control -----------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_80H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_81H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_82H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_83H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_84H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_85H, 0x0000);

    /* Panel Control ---------------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_90H, 0x0010);
    ARC_LCD_WriteReg(LCD_REG_92H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_93H, 0x0003);
    ARC_LCD_WriteReg(LCD_REG_95H, 0x0110);
    ARC_LCD_WriteReg(LCD_REG_97H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_98H, 0x0000);

    /* set GRAM write direction and BGR = 1 */
    /* I/D=00 (Horizontal : increment, Vertical : decrement) */
    /* AM=1 (address is updated in vertical writing direction) */
    ARC_LCD_WriteReg(LCD_REG_03H, (1<<12)|(3<<4)|(0<<3));

    ARC_LCD_WriteReg(LCD_REG_07H, 0x0133); /* 262K color and display ON */
}

/**
  * @brief  initialize panel.
  * @param  None.
  * @retval None
  */
void ARC_ILI9320_Init(LCD_Direction_TypeDef lcd_dir)
{

    	delay_ms(50);
    /* Start Initial Sequence ------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_E5H, 0x8000); /* Set the internal vcore voltage */
    ARC_LCD_WriteReg(LCD_REG_00H,  0x0001); /* Start internal OSC. */
    ARC_LCD_WriteReg(LCD_REG_01H,  0x0100); /* set SS and SM bit */
    ARC_LCD_WriteReg(LCD_REG_02H,  0x0700); /* set 1 line inversion */
    ARC_LCD_WriteReg(LCD_REG_03H,  (1<<12)|(3<<4)|(0<<3)); /* set GRAM write direction and BGR=1. */
    ARC_LCD_WriteReg(LCD_REG_04H,  0x0000); /* Resize register */
    ARC_LCD_WriteReg(LCD_REG_08H,  0x0202); /* set the back porch and front porch */
    ARC_LCD_WriteReg(LCD_REG_09H,  0x0000); /* set non-display area refresh cycle ISC[3:0] */
    ARC_LCD_WriteReg(LCD_REG_0AH, 0x0000); /* FMARK function */
    ARC_LCD_WriteReg(LCD_REG_0CH, 0x0000); /* RGB interface setting */
    ARC_LCD_WriteReg(LCD_REG_0DH, 0x0000); /* Frame marker Position */
    ARC_LCD_WriteReg(LCD_REG_0FH, 0x0000); /* RGB interface polarity */
    /* Power On sequence -----------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_10H, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    ARC_LCD_WriteReg(LCD_REG_11H, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    ARC_LCD_WriteReg(LCD_REG_12H, 0x0000); /* VREG1OUT voltage */
    ARC_LCD_WriteReg(LCD_REG_13H, 0x0000); /* VDV[4:0] for VCOM amplitude */
    delay_ms(200);                       /* Dis-charge capacitor power voltage (200ms) */
    ARC_LCD_WriteReg(LCD_REG_10H, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    ARC_LCD_WriteReg(LCD_REG_11H, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
    delay_ms(50);                       /* Delay 50 ms */
    ARC_LCD_WriteReg(LCD_REG_12H, 0x0139); /* VREG1OUT voltage */
    delay_ms(50);                       /* Delay 50 ms */
    ARC_LCD_WriteReg(LCD_REG_13H, 0x1d00); /* VDV[4:0] for VCOM amplitude */
    ARC_LCD_WriteReg(LCD_REG_29H, 0x0013); /* VCM[4:0] for VCOMH */
    delay_ms(50);                        /* Delay 50 ms */
    ARC_LCD_WriteReg(LCD_REG_20H, 0x0000); /* GRAM horizontal Address */
    ARC_LCD_WriteReg(LCD_REG_21H, 0x0000); /* GRAM Vertical Address */
    /* Adjust the Gamma Curve ------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_30H, 0x0006);
    ARC_LCD_WriteReg(LCD_REG_31H, 0x0101);
    ARC_LCD_WriteReg(LCD_REG_32H, 0x0003);
    ARC_LCD_WriteReg(LCD_REG_35H, 0x0106);
    ARC_LCD_WriteReg(LCD_REG_36H, 0x0b02);
    ARC_LCD_WriteReg(LCD_REG_37H, 0x0302);
    ARC_LCD_WriteReg(LCD_REG_38H, 0x0707);
    ARC_LCD_WriteReg(LCD_REG_39H, 0x0007);
    ARC_LCD_WriteReg(LCD_REG_3CH, 0x0600);
    ARC_LCD_WriteReg(LCD_REG_3DH, 0x020b);
  
    /* Set GRAM area ---------------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_50H, 0x0000); /* Horizontal GRAM Start Address */
    ARC_LCD_WriteReg(LCD_REG_51H, 0x00EF); /* Horizontal GRAM End Address */
    ARC_LCD_WriteReg(LCD_REG_52H, 0x0000); /* Vertical GRAM Start Address */
    ARC_LCD_WriteReg(LCD_REG_53H, 0x013F); /* Vertical GRAM End Address */
    ARC_LCD_WriteReg(LCD_REG_60H, 0x2700); /* Gate Scan Line */
    ARC_LCD_WriteReg(LCD_REG_61H, 0x0001); /* NDL,VLE, REV */
    ARC_LCD_WriteReg(LCD_REG_6AH, 0x0000); /* set scrolling line */
    /* Partial Display Control -----------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_80H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_81H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_82H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_83H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_84H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_85H, 0x0000);
    /* Panel Control ---------------------------------------------------------*/
    ARC_LCD_WriteReg(LCD_REG_90H, 0x0010);
    ARC_LCD_WriteReg(LCD_REG_92H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_93H, 0x0003);
    ARC_LCD_WriteReg(LCD_REG_95H, 0x0110);
    ARC_LCD_WriteReg(LCD_REG_97H, 0x0000);
    ARC_LCD_WriteReg(LCD_REG_98H, 0x0000);
    /* Set GRAM write direction and BGR = 1 */
    /* I/D=01 (Horizontal : increment, Vertical : decrement) */
    /* AM=1 (address is updated in vertical writing direction) */
    ARC_LCD_WriteReg(LCD_REG_03H, (1<<12)|(3<<4)|(0<<3));
    ARC_LCD_WriteReg(LCD_REG_07H, 0x0173); /* 262K color and display ON */
}

/**
  * @brief  initialize panel.
  * @param  None.
  * @retval None
  */
void ARC_SSD1289_Init(LCD_Direction_TypeDef lcd_dir)
{

	ARC_LCD_WriteReg(0x0000,0x0001);       /* 打开晶振 */
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0003,0xA8A4);    
	delay_ms(10);  
	ARC_LCD_WriteReg(0x000C,0x0000);    
	delay_ms(10);   
	ARC_LCD_WriteReg(0x000D,0x080C);    
	delay_ms(10);    
	ARC_LCD_WriteReg(0x000E,0x2B00);    
	delay_ms(10);    
	ARC_LCD_WriteReg(0x001E,0x00B0);    
	delay_ms(10);
#if (LCD_XSIZE == 320) && (LCD_YSIZE == 240)    
	ARC_LCD_WriteReg(0x0001,0x293F);       /* 驱动输出控制320*240 0x213F */
#else
	ARC_LCD_WriteReg(0x0001,0x2B3F);       /* 驱动输出控制320*240 0x233F */
#endif
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0002,0x0600);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0010,0x0000);    
	delay_ms(10); 
#if (LCD_XSIZE == 320) && (LCD_YSIZE == 240)
	ARC_LCD_WriteReg(0x0011,0x6078);       /* 定义数据格式 16位色 横屏 0x6078 */
#else
	ARC_LCD_WriteReg(0x0011,0x6070);       /* 定义数据格式 16位色 竖屏 0x6070 */
#endif
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0005,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0006,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0016,0xEF1C);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0017,0x0003);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0007,0x0133);    
	delay_ms(10);          
	ARC_LCD_WriteReg(0x000B,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x000F,0x0000);       /* 扫描开始地址 */
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0041,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0042,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0048,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0049,0x013F);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x004A,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x004B,0x0000);    
	delay_ms(10); 
 	ARC_LCD_WriteReg(0x0044,0xEF00);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0045,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0046,0x013F);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0030,0x0707);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0031,0x0204);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0032,0x0204);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0033,0x0502);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0034,0x0507);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0035,0x0204);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0036,0x0204);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0037,0x0502);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x003A,0x0302);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x003B,0x0302);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0023,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0024,0x0000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x0025,0x8000);    
	delay_ms(10); 
	ARC_LCD_WriteReg(0x004f,0);        /* 行首址0 */
	ARC_LCD_WriteReg(0x004e,0);        /* 列首址0 */
}
/**
  * @brief  Get the ID of the LCD controller.
  * @param  None
  * @retval The ID of the LCD controller
  */
uint16_t ARC_LCD_ID_Check()
{

    uint16_t DeviceCode = 0xFFFF;
    delay_ms(100);  /* delay 100 ms */
    DeviceCode = ARC_LCD_ReadReg(LCD_REG_00H); 
    if((DeviceCode != 0x9320) && (DeviceCode != 0x9325) && (DeviceCode != 0x9328) && (DeviceCode != 0x8989))
    {
        delay_ms(100);  /* delay 100 ms */
        DeviceCode = ARC_LCD_ReadReg(LCD_REG_67H); 
    }
    return DeviceCode;
}
/**
  * @brief  使能FSMC和LCD用到的管脚时钟.
  * @param  None
  * @retval None
  */
void ARC_LCD_RCC_Init(void)
{
	/* Enable FSMC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE\
	| RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);
}

/**
  * @brief  配置FSMC和LCD用到的管脚.
  * @param  None
  * @retval None
  */
void ARC_LCD_GPIO_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能FSMC时钟*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
    
    /* 使能FSMC对应相应管脚时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE , ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    /* 配置LCD背光控制管脚*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /* 配置LCD复位控制管脚*/
   /* LCD复位与系统复位在一起，所以不用其他引脚用于复位（我没有用其他IO来复位它下面的PE1没有用我接在LED上）*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 	 
    GPIO_Init(GPIOE, &GPIO_InitStructure);  		   
    
    /* 配置FSMC相对应的数据线,FSMC-D0~D15: PD 14 15 0 1,PE 7 8 9 10 11 12 13 14 15,PD 8 9 10*/	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | 
                                  GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                  GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure); 
    
  /* 配置FSMC相对应的控制线
   * PD4-FSMC_NOE  :LCD-RD  读
   * PD5-FSMC_NWE  :LCD-WR  写
   * PD7-FSMC_NE1  :LCD-CS  片选
   * PD11-FSMC_A16 :LCD-RS/DC  命令/数据
	 **********************************************/
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_11; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	// send Reset command
	/*
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	delayms(50);
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
	delayms(10);
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	delayms(900);
	*/
}
/**
  * @brief  Initialize LCD FSMC.
  * @param  None
  * @retval None
  */
static void ARC_LCD_FSMCConfig()
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p; 
   
    p.FSMC_AddressSetupTime = 0x02;	 //地址建立时间
    p.FSMC_AddressHoldTime = 0x00;	 //地址保持时间
    p.FSMC_DataSetupTime = 0x05;		 //数据建立时间
    p.FSMC_BusTurnAroundDuration = 0x00;
    p.FSMC_CLKDivision = 0x00;
    p.FSMC_DataLatency = 0x00;
    p.FSMC_AccessMode = FSMC_AccessMode_B;	
    
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;	
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p; 
   
    
    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
    
    /* 使能 FSMC Bank1_SRAM Bank */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);   
}
/**
  * @brief  Initialize LCD.
  * @param  None
  * @retval None
  */
int32_t ARC_LCD_Init(void)
{
    uint16_t DeviceCode;
    ARC_LCD_Param.LCD_Type = LCD_OTHER;
    ARC_LCD_Param.LCD_BusType = LCD_FSMC;
    ARC_LCD_Param.LCD_Direction = LCD_DIR_VERTICAL;
    ARC_LCD_RCC_Init();
    ARC_LCD_GPIO_Init();
	ARC_LCD_FSMCConfig();

	// reset the LCD
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	delay_ms(50);
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
	delay_ms(10);
	GPIO_SetBits(GPIOE, GPIO_Pin_1);
	delay_ms(10);
    
    DeviceCode = ARC_LCD_ID_Check();

    if(DeviceCode == 0x9320)
    {
        ARC_LCD_Param.LCD_Type = LCD_ILI9320;
        ARC_ILI9320_Init(ARC_LCD_Param.LCD_Direction);
    }
    else if(DeviceCode == 0x9325 || DeviceCode == 0x9328)
    {
        ARC_LCD_Param.LCD_Type = LCD_ILI9325;
        ARC_ILI9325_Init(ARC_LCD_Param.LCD_Direction);
    }
    else if(DeviceCode == 0x47)
    {
        ARC_LCD_Param.LCD_Type = LCD_HX8347A;
        ARC_HX8347A_Init(ARC_LCD_Param.LCD_Direction);
    }
	else if(DeviceCode == 0x8989)
	{
        ARC_LCD_Param.LCD_Type = LCD_SSD1289;
        ARC_SSD1289_Init(ARC_LCD_Param.LCD_Direction);
	}
    else
    {
			printf("ARC_LCD_ID_Check() failed(DeviceCode=%x)\r\n", DeviceCode);
        return 1;
    }
	//ARC_LCD_FSMCConfig(5,2,5);
	ARC_LCD_FillRect(0,0,LCD_XSIZE-1,LCD_YSIZE-1);
    return 0;
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ARC_LCD_On(void)
{

    if((ARC_LCD_Param.LCD_Type == LCD_ILI9320) || (ARC_LCD_Param.LCD_Type == LCD_SPFD5408))
    {
        /* Display On */
        ARC_LCD_WriteReg(LCD_REG_07H, 0x0173); /* 262K color and display ON */
    }
    else if(ARC_LCD_Param.LCD_Type == LCD_HX8312)
    {  
        ARC_LCD_WriteReg(LCD_REG_01H, 0x50);
        ARC_LCD_WriteReg(LCD_REG_05H, 0x04);           
        /* Display On */
        ARC_LCD_WriteReg(LCD_REG_00H, 0x80);
        ARC_LCD_WriteReg(LCD_REG_3BH, 0x01);
		delay_ms(40);
        ARC_LCD_WriteReg(LCD_REG_00H, 0x20);
    }
    ARC_LCD_BL_SET();
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ARC_LCD_Off(void)
{
    ARC_LCD_BL_RESET();
    if((ARC_LCD_Param.LCD_Type == LCD_ILI9320) || (ARC_LCD_Param.LCD_Type == LCD_SPFD5408))
    {
        /* Display Off */
        ARC_LCD_WriteReg(LCD_REG_07H, 0x0);
    }
    else if(ARC_LCD_Param.LCD_Type == LCD_HX8312)
    { 
        /* Display Off */
        ARC_LCD_WriteReg(LCD_REG_00H, 0xA0);
        delay_ms(40);
        ARC_LCD_WriteReg(LCD_REG_3BH, 0x00);
    } 
}

/**
  * @brief  set a pixel at position (x, y) with a color ColorIndex.
  * @param  x, the x position of the LCD
  * @param  y, the y position of the LCD
  * @param  ColorIndex, the color to be set to the pixel.
  * @retval None
  */
void ARC_LCD_SetPixelIndex(int32_t x,int32_t y,int32_t ColorIndex)
{
    ARC_LCD_SetCursor(x, y);
    ARC_LCD_WriteReg(LCD_REG_22H, ColorIndex);
}

/**
  * @brief  get a pixel value at position (x, y).
  * @param  x, the x position of the LCD
  * @param  y, the y position of the LCD
  * @retval the pixel value
  */
__inline uint32_t ARC_LCD_GetPixelIndex(int32_t x, int32_t y) 
{
    uint16_t temp=0;
    ARC_LCD_SetCursor(x, y);
    ARC_LCD_WriteRegIndex(LCD_REG_22H);
    ARC_LCD_ReadGRAMData(temp);//SSD1289要求读两次，第一次数据无效
	ARC_LCD_ReadGRAMData(temp);
#if LCD_SWAP_RB == 0
	return  temp;
#else
    return  (((temp)  & 0x1f)<<11) + (((temp>>5)  & 0x3f)<<5) + (((temp>>11) & 0x1f));
#endif
}
//以下函数是针对SSD1289液晶屏优化过，刷屏速度更快了
#if (1)
/**
  * @brief  draw a horizontal line.
  */
void ARC_LCD_DrawHLine  (int32_t x0, int32_t y,  int32_t x1)
{
    ARC_LCD_SetCursor(x0, y);
    ARC_LCD_WriteGRAM_Same(LCD_COLOR, (x1 - x0 + 1));
}

/**
  * @brief  draw a vertical line.
  */
void ARC_LCD_DrawVLine  (int32_t x, int32_t y0,  int32_t y1) 
{
    uint16_t i;
    for(i = y0; i <= y1; i ++)
        ARC_LCD_SetPixelIndex(x, i, LCD_COLOR);
}

void ARC_LCD_DrawLine(int32_t x1, int32_t y1, int32_t x2, int32_t y2,int32_t nColor)
{
    int32_t i, deltax, deltay, numpixels, d, dinc1, dinc2,

      x, xinc1, xinc2, y, yinc1, yinc2;
    // Calculate delta-x and delta-y for initialization
    if(x2-x1 < 0)
                 deltax = x1-x2 ;
    else        deltax = x2-x1 ;
    if(y2-y1 < 0)
                 deltay = y1-y2 ;
    else        deltay = y2-y1 ;

        // Initialize all vars based on which is the independent variable
        //
        if(deltax >= deltay)
        {
            //x is independent variable
            //
            numpixels = deltax + 1;
            d = (2 * deltay) - deltax;
            dinc1 = deltay << 1;
            dinc2 = (deltay - deltax) << 1;
            xinc1 = 1;
            xinc2 = 1;
            yinc1 = 0;
            yinc2 = 1;
        }
       else
        {

            // y is independent variable
            //
            numpixels = deltay + 1;
            d = (2 * deltax) - deltay;
            dinc1 = deltax << 1;
            dinc2 = (deltax - deltay) << 1;
            xinc1 = 0;
            xinc2 = 1;
            yinc1 = 1;
            yinc2 = 1;
        }
        // Make sure x and y move in the right directions
        //
        if(x1 > x2)
        {
           xinc1 = - xinc1;
           xinc2 = - xinc2;
        }
        if(y1 > y2)
        {
           yinc1 = - yinc1;
           yinc2 = - yinc2;
        }

        // Start drawing at <x1, y1>
        //
        x = x1;
        y = y1;
        // Draw the pixels
       //
       for(i= 1; i< numpixels;i++)
        {
                 // 画点
                ARC_LCD_SetPixelIndex(x, y, nColor);
                if (d < 0)
                {
                    d = d + dinc1;
                    x = x + xinc1;
                    y = y + yinc1;
                }
                else
                {
                    d = d + dinc2;
                    x = x + xinc2;
                    y = y + yinc2;
                }
        }
}

/**
  * @brief  fill a rectangle.
  */
void ARC_LCD_FillRect  (int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    ARC_LCD_SetDisplayWindow(x0, y0, x1, y1);
    ARC_LCD_WriteGRAM_Same(LCD_COLOR, ((x1 - x0 + 1) * (y1 - y0 + 1)));
    ARC_LCD_SetDisplayWindow(0, 0, LCD_XSIZE - 1, LCD_YSIZE - 1);
}

///**
//  * @brief  xor a pixel.
//  */
//void ARC_LCD_XorPixel(int32_t x, int32_t y) 
//{
//    uint16_t Index = ARC_LCD_GetPixelIndex(x,y);
//    ARC_LCD_SetPixelIndex(x, y, LCD_NUM_COLORS - 1 - Index);
//}

/**
  * @brief  dummy function.
  */
void ARC_LCD_SetOrg(uint8_t Pos, uint32_t color)
{
}

/**
  * @brief  dummy function.
  */
void ARC_LCD_SetLUTEntry(void)
{
}
void ARC_LCD_Clear  (uint16_t lcdColor)
{
    ARC_LCD_SetCursor(0, 0);
    ARC_LCD_WriteGRAM_Same(lcdColor, 240 * 320);
}
#else

/**
  * @brief  draw a big point (3 x 3) on the specific position on the LCD screen.
  * @param  x: the center x position.
  * @param  y: the center y position.
  * @param  pointColor: the big point color.
  * @retval None
  */
void ARC_LCD_DrawBigPoint(uint16_t x,uint16_t y, uint16_t pointColor)
{
    uint8_t i, j;
    for (i = 0; i < 2; i++)
        for (j = 0; j < 2; j++)
            ARC_LCD_SetPixelIndex(x - 1 + i, y - 1 + j, pointColor);
}

/**
  * @brief  draw a cross on the specific position on the LCD screen.
  * @param  x: the center x position.
  * @param  y: the center y position.
  * @param  pointColor: the point color of the cross.
  * @retval None
  */
void ARC_LCD_DrawCross(uint16_t x, uint16_t y, uint16_t pointColor)
{
    int8_t i;
    for(i = -5; i <= 5; i++)
        ARC_LCD_SetPixelIndex(x + i, y, pointColor);
    
    for(i = -5; i <= 5; i++)
        ARC_LCD_SetPixelIndex(x, y + i, pointColor);
}



void ARC_LCD_Clear  (uint16_t lcdColor)
{
    ARC_LCD_SetCursor(0, 0);
    ARC_LCD_WriteGRAM_Same(lcdColor, 240 * 320);
}

#endif

/**
  * @brief  show a character on the LCD screen.
  * @param  *LCDParam, the LCD parameters.
  * @retval None
  */
void ARC_LCD_ShowChar(int32_t x, int32_t y, const uint8_t *lcd_font)
{   
    uint8_t point;
    uint8_t v, h;
    uint8_t font;

    font = *(lcd_font) - ' ';
    
    for(v = 0; v < 16; v++)
    {
        point = asc2_1608[font][v]; /* using 1206 font */
        for(h = 0; h < 8; h++)
        {                 
            if(point & 0x01)
                ARC_LCD_SetPixelIndex(x + h, y + v, LCD_COLOR_WHITE);

            point >>= 1; 
        }
    }
}

/**
  * @brief  show a character on the LCD screen.
  * @param  *LCDParam, the LCD parameters.
  * @retval None
  */
void ARC_LCD_ShowCharGBK(int32_t x, int32_t y, const uint8_t *lcd_font)
{
	uint8_t h,v;
	uint8_t buffer[32];
	uint16_t tmp_char=0;
	
	GetGBKCode(buffer, (unsigned char*)lcd_font);  /* 取字模数据 */

	for ( v = 0; v < 16; v++ )
	{
		tmp_char = buffer[v*2];
		tmp_char = ( tmp_char << 8 );
		tmp_char |= buffer[2*v+1];
		for (h = 0; h < 16; h++ )			// x
		{
		    if ( (tmp_char >> 15-h ) & 0x01 == 0x01 )
				ARC_LCD_SetPixelIndex(x + h, y + v, LCD_COLOR_WHITE);
	    }
	}

	/*
    uint8_t point;
    uint8_t v, h;
    uint8_t font;

    font = *(lcd_font) - ' ';
    
    for(v = 0; v < 16; v++)
    {
        point = asc2_1608[font][v];
        for(h = 0; h < 16; h++)
        {                 
            if(point & 0x01)
                ARC_LCD_SetPixelIndex(x + h, y + v, LCD_COLOR_WHITE);

            point >>= 1; 
        }
    }
		*/
}

/**
  * @brief  show strings on the LCD screen.
  * @param  *LCDParam, the LCD parameters.
  * @retval None
  */
void ARC_LCD_ShowString(int32_t x, int32_t y, const uint8_t *lcd_font)
{
    while(*lcd_font != '\0')
    {
		if (*lcd_font &0x80)
		{
			ARC_LCD_ShowCharGBK(x, y, lcd_font);
			x += 16;
			lcd_font+=2;
		}
		else
		{
			ARC_LCD_ShowChar(x, y, lcd_font);
			x += 8;
			lcd_font++;
		}
    }
}
/**
  * @}
  */

/**
  * @}
  */
  
/**
  * @}
  */  
    
/******************* (C) www.armrunc.com *****END OF FILE****/
