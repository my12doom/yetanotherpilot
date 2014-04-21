#include <string.h>
#include "MAX7456.h"


#define MAX7456_CS_Low()		GPIO_ResetBits(GPIOB, MAX7546_CS) 
#define MAX7456_CS_Hight()		GPIO_SetBits(GPIOB, MAX7546_CS) 
#define MAX7456_StartSpi()		MAX7456_CS_Low()
#define MAX7456_EndSpi()        MAX7456_CS_Hight()
#define MAX7456_Wait_NVM_Ready()	while((MAX7456_Read_Reg(STAT) & 0x20) != 0x00)
#define MAX7456_Wait_VSYNC()	while((MAX7456_Read_Reg(STAT) & 0x10) != 0x00)
#define MAX7456_Wait_SRAM_Ready()	while((MAX7456_Read_Reg(DMM) & 0x04) != 0x00)

GPIO_InitTypeDef GPIO_InitStructure_MAX7456;
SPI_InitTypeDef SPI_InitStructure_MAX7456;
char screen[30*16];

void MAX7456_PORT_Init(void) 
{ 
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(MAX7456_PCLK,ENABLE); 
	RCC_APB1PeriphClockCmd(MAX7456_SPICLK,ENABLE); 
	// 
	GPIO_InitStructure_MAX7456.GPIO_Pin=MAX7456_SCK|MAX7456_MOSI|MAX7456_MISO; 
	GPIO_InitStructure_MAX7456.GPIO_Mode=GPIO_Mode_AF_PP; 
	GPIO_InitStructure_MAX7456.GPIO_Speed=GPIO_Speed_50MHz; 
	GPIO_Init(MAX7456_SPI_PORT,&GPIO_InitStructure_MAX7456); 
	// 
	GPIO_InitStructure_MAX7456.GPIO_Mode=GPIO_Mode_Out_PP; 
	GPIO_InitStructure_MAX7456.GPIO_Pin=MAX7546_CS; 
	GPIO_Init(GPIOB,&GPIO_InitStructure_MAX7456); 
} 

void MAX7456_SPI_Init(void) 
{ 
	
	MAX7456_CS_Hight(); 
	// 
	SPI_InitStructure_MAX7456.SPI_Direction=SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure_MAX7456.SPI_Mode=SPI_Mode_Master; 
	SPI_InitStructure_MAX7456.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4;//max7456????10M SPI????P/2=36M 
	SPI_InitStructure_MAX7456.SPI_CPHA=SPI_CPHA_1Edge; 
	SPI_InitStructure_MAX7456.SPI_CPOL=SPI_CPOL_Low; 
	SPI_InitStructure_MAX7456.SPI_NSS=SPI_NSS_Soft; 
	SPI_InitStructure_MAX7456.SPI_DataSize=SPI_DataSize_8b; 
	SPI_InitStructure_MAX7456.SPI_FirstBit=SPI_FirstBit_MSB; 
	SPI_InitStructure_MAX7456.SPI_CRCPolynomial=7; 
	SPI_Init(MAX7456_SPI, &SPI_InitStructure_MAX7456); 
	// 
	SPI_Cmd(MAX7456_SPI,ENABLE); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

} 

u8 MAX7456_SPI_WriteByte(u8 Data) 
{ 
	/* Wait for SPI2 Tx buffer empty */ 
	while (SPI_I2S_GetFlagStatus(MAX7456_SPI, SPI_I2S_FLAG_TXE) == RESET); 
	    /* Send SPI2 data */ 
	    SPI_I2S_SendData(MAX7456_SPI, Data); 
	
	while (SPI_I2S_GetFlagStatus(MAX7456_SPI, SPI_I2S_FLAG_RXNE) == RESET); 
    /* Read SPI2 received data */ 
    Data = SPI_I2S_ReceiveData(MAX7456_SPI); 

    return Data; 
} 

u8 MAX7456_SPI_ReadByte(void) 
{ 
	u8 Data=0; 
	/* Wait for SPI2 Tx buffer empty */ 
	while (SPI_I2S_GetFlagStatus(MAX7456_SPI, SPI_I2S_FLAG_TXE) == RESET); 
    /* Send SPI2 data */ 
    SPI_I2S_SendData(MAX7456_SPI, Data); 

	while (SPI_I2S_GetFlagStatus(MAX7456_SPI, SPI_I2S_FLAG_RXNE) == RESET); 
    /* Read SPI2 received data */ 
    Data = SPI_I2S_ReceiveData(MAX7456_SPI); 

    return Data; 
} 

void MAX7456_Delay_uS(void) 
{ 
	u8 i=100;
	while(i)
	{  
		i--;  
	}  
} 

void MAX7456_Delay_mS(u32 mS) 
{ 
	u32 i=0; 
	for(i=0;i<mS*10;i++) 
	{ 
		MAX7456_Delay_uS(); 
	} 
} 

void MAX7456_Write_Reg(u8 add,u8 data) 
{ 
	MAX7456_StartSpi(); 
	
	MAX7456_SPI_WriteByte(add & (~RADD1) );//???? 
	MAX7456_SPI_WriteByte(data);//???? 
	
	MAX7456_EndSpi(); 
} 

u8 MAX7456_Read_Reg(u8 add)
{ 
	u8 Data=0; 
	MAX7456_StartSpi(); 
	
	MAX7456_SPI_WriteByte(add | RADD1);//???? 
	Data=MAX7456_SPI_ReadByte();//???? 
	
	MAX7456_EndSpi(); 
	
	return Data; 
} 

void MAX7456_SYS_Init(void) 
{
	int i,j;
	// 
	MAX7456_PORT_Init();
	// 
	MAX7456_SPI_Init();
	
	// 
	MAX7456_Write_Reg(VM0,0x02);
	
	delayms(200);
	printf("MAX7456 = %d\r\n", MAX7456_Read_Reg(HOS+RADD1));

	MAX7456_Write_Reg(VM0,0x00);

	MAX7456_Delay_mS(1000);


	MAX7456_Write_Reg(VM0,0X00 | ((MAX7456_Read_Reg(STAT) & 1) << 6)); // AUTO
	//MAX7456_Write_Reg(VM0,0X40); // PAL
		
	MAX7456_Write_Reg(DMM,0X04);
	// 
	MAX7456_Delay_mS(1000);
	
	MAX7456_Write_Reg(DMM,0X40);
	
	MAX7456_Delay_mS(1000); 
	
	MAX7456_Write_Reg(VM0,0X08 | ((MAX7456_Read_Reg(STAT) & 1) << 6)); // AUTO
	MAX7456_Write_Reg(VM0,0X08 | 0x4); //PAL
	MAX7456_Delay_mS(1000); 
	
	MAX7456_Write_Reg(VM1,0X00); 
	MAX7456_Delay_mS(1000); 
	
	MAX7456_Write_Reg(HOS,0X38); 
	MAX7456_Delay_mS(1000); 
	
	//MAX7456_Write_Reg(OSDBL_W_ADD,(MAX7456_Read_Reg(OSDBL_R_ADD)|0X10)); 
	
	MAX7456_Write_Reg(OSDM,0x2D); 
	printf("MAX7456 = %02x,%02x\r\n", MAX7456_Read_Reg(VM0+RADD1), MAX7456_Read_Reg(VM1+RADD1));
	
	
	
	// font reading test
	{
		char tbl[4] = {'X', ' ', '.', ' '};
		Max7456_Download_Char(20, character);
		printf("\n");
		for(i=0; i<54; i++)
		{
			//printf("%02x", character[i]);
			for(j=0; j<4; j++)
			{
				u8 c = (character[i] >> (6-j*2))&3;
				printf("%c", tbl[c]);
			}
			
			if (i>0 && i%3 == 2)
				printf("\n");
			
		}
	}
	
	// font writing test
	{
		memset(character, 0, 54);
		Max7456_Update_Char(0, character);
		for(i=0; i<18; i++)
		{
			character[i*3+0] = 0xAA;
			character[i*3+1] = 0xAA;
			character[i*3+2] = 0xAA;
			
			Max7456_Update_Char(i+1, character);

			character[i*3+0] = 0;
			character[i*3+1] = 0;
			character[i*3+2] = 0;
		}
	}

} 



void MAX7456_Write_Char(u16 add,u8 chr) 
{ 
	MAX7456_Write_Reg(DMAH, (add&0XFF00)>>8); 
	MAX7456_Write_Reg(DMAL, (add&0XFF)); 
	MAX7456_Write_Reg(DMDI,chr); 
} 


  
void MAX7456_WriteAtt_Char(u16 add,u8 attribute) 
{ 
	MAX7456_Write_Reg(DMAH, ((add&0XFF00)>>8)|2); 
	MAX7456_Write_Reg(DMAL, (add&0XFF)); 
	MAX7456_Write_Reg(DMDI,attribute); 
} 


void MAX7456_Write_Char_XY(u8 X,u8 Y,u8 chr) 
{ 
	u16 add=0; 
	add=((u16)Y)*30+X; 
	MAX7456_Write_Char(add,chr); 
} 


void MAX7456_WriteAtt_XY(u8 X,u8 Y,u8 chr) 
{ 
	u16 add=0; 
	add=((u16)Y)*30+X; 
	MAX7456_WriteAtt_Char(add,chr); 
} 

void MAX7456_Clear(void) 
{ 
	u16 memory_address = 0,i=0; 
    for (i = 0; i < 480; i++)  
        MAX7456_Write_Char(memory_address++, 0); 
} 

void MAX7456_ClearScreen(void) 
{ 
	uint16_t i; 
    for (i = 0; i < 480; i++)  
      if(screen[i])
      {
				MAX7456_Write_Char(i, screen[i] = '\0');
			}
}

void MAX7456_WriteScreen(void) 
{ 
	uint16_t i;
    for (i = 0; i < 480; i++)
			if(screen[i])
      {
				MAX7456_Write_Char(i, screen[i]);
				//screen[i] = '\0';
			}
}

void MAX7456_Write_ASCII_Char(u16 address, u8 c)  
{ 
    if (c == 32) c = 0; // remap space
		else if (c > 48 && c <= 57) c -= 48; // remap numbers 
    else if (c == '0') c = 10; // remap zero 
    else if (c >= 65 && c <= 90) c -= 54; // remap big letters 
    else if (c >= 97 && c <= 122) c -= 60; // remap small letters 
    else if (c == '(') c = 63; // remap 
    else if (c == ')') c = 64; // remap 
    else if (c == '.') c = 65; // remap 
    else if (c == '?') c = 66; // remap 
    else if (c == ';') c = 67; // remap 
    else if (c == ':') c = 68; // remap 
    else if (c == ',') c = 69; // remap 
    else if (c == '\'') c = 70; // remap 
    else if (c == '/') c = 71; // remap 
    else if (c == '"') c = 72; // remap 
    else if (c == '-') c = 73; // remap minus 
    else if (c == '<') c = 74; // remap 
    else if (c == '>') c = 75; // remap 
    else if (c == '@') c = 76; // remap 
    MAX7456_Write_Char(address, c); 
} 


void MAX7456_Write_ASCII_Chr(u8 X,u8 Y,u8 chr) 
{ 
	MAX7456_Write_ASCII_Char(((X++)+(((u16)Y*30))),chr); 
} 

void MAX7456_Write_ASCII_String(u8 X,u8 Y,u8 *Str) 
{ 
	while(*Str) 
	{ 
		MAX7456_Write_ASCII_Char(((X++)+(((u16)Y*30))),*Str); 
		Str++; 
	} 
} 

void MAX7456_Write_String(u16 address, u8 *Str)
{ 
	while(*Str) 
	{ 
		MAX7456_Write_Char(address++,*Str++);
	} 
}

void Max7456_Learn_Char(u8 number, const u8 *data)  
{ 
	u8 i, vm0;
	
	vm0 = MAX7456_Read_Reg(VM0 | RADD1);
	MAX7456_Write_Reg(VM0, vm0 & (~ENABLE_OSD));
	MAX7456_Wait_NVM_Ready();
	
    // select character to write (CMAH) 
    MAX7456_Write_Reg(CMAH, number); 

    for (i = 0; i < 54; i++) { 
        // select 4pixel byte of char (CMAL) 
        MAX7456_Write_Reg(CMAL, i); 

        // write 4pixel byte of char (CMDI) 
        MAX7456_Write_Reg(CMDI, data[i]); 
    } 

    // write to the NVM array from the shadow RAM (CMM) 
    MAX7456_Write_Reg(CMM, 0xa0); 

    // according to maxim writing to nvram takes about 12ms, lets wait longer 
	MAX7456_Wait_NVM_Ready();
	MAX7456_Write_Reg(VM0, vm0 | ENABLE_OSD);
} 

void Max7456_Download_Char(u8 number, u8 *data)
{ 
	u8 i, vm0; 

	vm0 = MAX7456_Read_Reg(VM0 | RADD1);
	MAX7456_Write_Reg(VM0, vm0 & (~ENABLE_OSD));
	MAX7456_Wait_NVM_Ready();

	// select character to write (CMAH) 
	MAX7456_Write_Reg(CMAH, number);
	MAX7456_Write_Reg(CMM, 0x50);
	MAX7456_Wait_NVM_Ready();

    for (i = 0; i < 54; i++) { 
        // select 4pixel byte of char (CMAL) 
        MAX7456_Write_Reg(CMAL, i); 

        // read pixel of char (CMDI) 
        data[i] = MAX7456_Read_Reg(CMDO); 
    } 
	MAX7456_Write_Reg(VM0, vm0 | ENABLE_OSD);
}

void Max7456_Update_Char(u8 number, const u8 *data)
{
	u8 read[54];
	int i;
	
	// 3 retries, to avoid eeprom wring by read failure.
	for(i=0; i<3; i++)
	{
		Max7456_Download_Char(number, read);
		
		if (memcmp(read, data, 54) == 0)
			return;
	}
	
	printf("WRITING CHARACTER %d\n", number);
	
	Max7456_Learn_Char(number, data);
}

u8	 Max7456_Get_System()
{
	return MAX7456_Read_Reg(0xA0) & 0x03;
}

void Max7456_Set_System(u8 system)
{
	if(system)
		MAX7456_Write_Reg(VM0,MAX7456_Read_Reg(VM0)|0x40); 
	else
		MAX7456_Write_Reg(VM0,MAX7456_Read_Reg(OSDBL_R_ADD) & (~0x40));
}

void Max7456_Display_AllChar(void) 
{ 
	u16 i=0; 
	for(i=0;i<25;i++) 
	MAX7456_Write_Char_XY(i, 1, i); 
	for(i=25;i<50;i++) 
	MAX7456_Write_Char_XY(i-25, 2, i); 
	for(i=50;i<75;i++) 
	MAX7456_Write_Char_XY(i-50, 3, i); 
	
	for(i=75;i<100;i++) 
	MAX7456_Write_Char_XY(i-75, 4, i); 
	for(i=100;i<125;i++) 
	MAX7456_Write_Char_XY(i-100, 5, i); 
	for(i=125;i<150;i++) 
	MAX7456_Write_Char_XY(i-125, 6, i); 
	
	for(i=150;i<175;i++) 
	MAX7456_Write_Char_XY(i-150, 7, i); 
	for(i=175;i<200;i++) 
	MAX7456_Write_Char_XY(i-175, 8, i); 
	for(i=200;i<225;i++) 
	MAX7456_Write_Char_XY(i-200, 9, i); 
	
	for(i=225;i<250;i++) 
	MAX7456_Write_Char_XY(i-225, 10, i); 
	
	for(i=250;i<=255;i++) 
	MAX7456_Write_Char_XY(i-250, 11, i); 
}
