//#include "stm32f10x_lib.h" 
#include "stm32f10x_spi.h" 
#define MAX7456_SPI          SPI1

#define MAX7456_SPICLK       RCC_APB2Periph_SPI1
#define MAX7456_PCLK         RCC_APB2Periph_GPIOA
#define MAX7456_SPI_PORT     GPIOA
#define MAX7456_SCK          GPIO_Pin_5
#define MAX7456_MISO         GPIO_Pin_6
#define MAX7456_MOSI         GPIO_Pin_7
#define MAX7546_CS           GPIO_Pin_2

#define RADD1         0X80

#define VM0           0X00
#define VM1           0X01
#define HOS           0X02
#define VOS           0X03
#define DMM           0X04
#define DMAH          0X05
#define DMAL          0X06
#define DMDI          0X07
#define CMM           0X08
#define CMAH          0X09
#define CMAL          0X0A
#define CMDI          0X0B
#define OSDM          0X0C
#define RB            0X10
#define OSDBL_W_ADD   0X6C
#define OSDBL_R_ADD   0XEC
#define STAT          0XA0
#define DMDO          0XB0
#define CMDO          0XC0


#define CMM_WRITE		0xA0
#define CMM_READ		0x50

#define VIDEO_STANDARD	0x40
#define ENABLE_OSD	  	0x08
#define ENABLE_VSYNC  	0x04
#define SOFT_RESET    	0x02
#define BUFFER_ENABLE 	0x01

extern char screen[];


#ifdef __cplusplus
extern "C" {
#endif

void MAX7456_SYS_Init(void); 
u8 MAX7456_Read_Reg(u8 add); 
void MAX7456_Write_Char(u16 add,u8 chr); 
void MAX7456_WriteAtt_Char(u16 add,u8 attribute); 
void MAX7456_Write_Char_XY(u8 X,u8 Y,u8 chr); 
void MAX7456_WriteAtt_XY(u8 X,u8 Y,u8 chr); 
void MAX7456_Clear(void); 
void MAX7456_Write_ASCII_Chr(u8 X,u8 Y,u8 chr); 
void MAX7456_Write_ASCII_Char(u16 address, u8 c); 
void MAX7456_Write_ASCII_String(u8 X,u8 Y,u8 *Str); 
void Max7456_Learn_Char(u8 number, const u8 *data); 
void Max7456_Write_CustomChar(void); 
//void Max7456_Write_FileChar(void); 
void Max7456_Display_AllChar(void); 
void Max7456_Check_Custom(void);
u8	 Max7456_Get_System(void);
void Max7456_Set_System(u8 system);
void Max7456_Download_Char(u8 number, u8 *data);
void Max7456_Update_Char(u8 number, const u8 *data);
void MAX7456_Write_String(u16 address, u8 *Str);
void MAX7456_ClearScreen(void);
void MAX7456_WriteScreen(void);

#ifdef __cplusplus
}
#endif
