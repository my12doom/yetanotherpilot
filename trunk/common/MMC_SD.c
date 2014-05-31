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

#include "MMC_SD.h"
#include "../mcu.h"
#include <stdio.h>

#define SPI_CS_Assert   GPIO_ResetBits(MMC_SD_CS_PORT, MMC_SD_CS) 
#define SPI_CS_Deassert GPIO_SetBits(MMC_SD_CS_PORT, MMC_SD_CS) 


/* spi low speed, below 400KHz */
void SPI_Low(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;

  	/* SPI1 Config */
  	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	SPI_Init(SPI1, &SPI_InitStructure);

  	/* SPI1 enable */
  	SPI_Cmd(SPI1, ENABLE);
        
        /*Configure PA.4(NSS)--------------------------------------------*/
        GPIO_InitStructure.GPIO_Pin =MMC_SD_CS;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
        
        GPIO_Init(MMC_SD_CS_PORT, &GPIO_InitStructure);
}

/* spi high speed, not exceed 25MHz */
void SPI_High(void)
{
        GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;
	
	//SPI_Low();
	//return;

  	/* SPI1 Config */
  	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	SPI_Init(SPI1, &SPI_InitStructure);

  	/* SPI1 enable */
  	SPI_Cmd(SPI1, ENABLE);
        
        /*Configure PA.4(NSS)--------------------------------------------*/
        GPIO_InitStructure.GPIO_Pin =MMC_SD_CS;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
        GPIO_Init(MMC_SD_CS_PORT, &GPIO_InitStructure);               
}

/* read and write one byte , full duplex */
uint8 SPI_WriteByte(uint8 val)
{
	/* Wait until the transmit buffer is empty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  /* Send the byte */
  SPI_I2S_SendData(SPI1, val);
  
  /* Wait until a data is received */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  /* Get the received data */
	return SPI_I2S_ReceiveData(SPI1);
}

/* sd send command */ 
uint8 MMC_SD_SendCommand(uint8 cmd, uint32 arg)
{
	uint8 r1;
	uint8 retry=0;
	
	SPI_WriteByte(0xff);
	
	SPI_WriteByte(cmd | 0x40);	/* send command */
	SPI_WriteByte(arg>>24);
	SPI_WriteByte(arg>>16);
	SPI_WriteByte(arg>>8);
	SPI_WriteByte(arg);
	SPI_WriteByte(0x95);
	
	while((r1 = SPI_WriteByte(0xff)) == 0xff)	/* wait response */
		if(retry++ > 0xfe)
		{
			printf("cmd failed\n");
			break;				/* time out error */
		}

	//SPI_WriteByte(0xff);				// extra 8 CLK

	return r1;								/* return state */
}

/* SD card initialization, include reset and configuration */
uint8 MMC_SD_Init(void)
{
	u16 i;
	uint8 retry = 0;
	uint8 r1 = 0;
	
	GPIO_InitTypeDef  GPIO_InitStructure;

	//uart_puts("ini sd\r\n");
	
	/* GPIO Periph clock enable */
	RCC_APB2PeriphClockCmd(MMC_SD_GPIO_PORTS | RCC_APB2Periph_GPIOB, ENABLE);
	/* SPI1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* config CS of SD */
	GPIO_InitStructure.GPIO_Pin = MMC_SD_CS;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(MMC_SD_CS_PORT, &GPIO_InitStructure);


	/* Configure SPI1 pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = MMC_SD_SCK | MMC_SD_MISO | MMC_SD_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(MMC_SD_SPI_PORT, &GPIO_InitStructure);


	/* set SPI clock speed */
	SPI_Low();

	SPI_CS_Assert;

        // 纯延时，等待SD卡上电完成
        for(i=0;i<0xf00;i++);

	do
	{
		for(i=0;i<100;i++) SPI_WriteByte(0xff);
		r1 = MMC_SD_SendCommand(0, 0);//发idle命令	//send idle command
		retry++;
		if(retry>0xfe){SPI_CS_Deassert; return 1;}//超时退出		//time out
	} while(r1 != 0x01);	


	retry = 0;
	do
	{
		r1 = MMC_SD_SendCommand(1, 0);//发active命令	//send active command
		retry++;
		if(retry>0xfe){SPI_CS_Deassert; return 2;}//超时退出		//time out
	} while(r1);
	
	SPI_High();		/* Use High Speed SPI*/
	
	r1 = MMC_SD_SendCommand(59, 0);//关crc		//disable CRC

	r1 = MMC_SD_SendCommand(16, 512);//设扇区大小512	//set sector size to 512
	SPI_CS_Deassert;
	return 0;//正常返回		//normal return

}

//读一个扇区		//read one sector
uint8 MMC_SD_ReadSingleBlock(uint32 sector, uint8* buffer)
{
	uint8 r1;
	uint16 i;
	uint16 retry=0;
	uint8_t token = 0;

	printf("read block %d\n", sector);
	SPI_High();		/* Use High Speed SPI*/

	SPI_CS_Assert;
	
	// wait for bus idle
	while (!SPI_WriteByte(0xff))
		;
	
	
	r1 = MMC_SD_SendCommand(17, sector<<9);//读命令	//read command
	
	if(r1 != 0x00)
		return r1;


	//等数据的开始	//wait to start recieve data
	while((token=SPI_WriteByte(0xff)) != 0xfe)
	{
		if(retry++ > 0xffe)
		{
			SPI_CS_Deassert;
			printf("read sector %u failed\n", sector);
			break;
		}
		
		if (token & 0x80 == 0)
			printf("token=%02x\n", token);
	}
	
	for(i=0; i<512; i++)//读512个数据	//read 512 bytes
	{
		*buffer++ = SPI_WriteByte(0xff);
	}

	SPI_WriteByte(0xff);//伪crc    //dummy crc
	SPI_WriteByte(0xff);
	
	SPI_CS_Deassert;
	SPI_WriteByte(0xff);// extra 8 CLK

	return 0;
}


//写一个扇区		//wirite one sector //not used in this application
uint8 MMC_SD_WriteSingleBlock(uint32 sector, const uint8* buffer)
{
	uint8 r1;
	uint16 i;
	uint16 retry=0;
	
	printf("write block %d\n", sector);
	
	SPI_High();		/* Use High Speed SPI*/

	SPI_CS_Assert;
	
	// wait for bus idle
	while (!SPI_WriteByte(0xff))
		;
	
	r1 = MMC_SD_SendCommand(24, sector<<9);//写命令	//send command
	if(r1 != 0x00)
	{
		printf("write cmd rejected with code %02x", r1);
		return r1;
	}

	
	SPI_WriteByte(0xff);
	SPI_WriteByte(0xff);
	SPI_WriteByte(0xff);

	SPI_WriteByte(0xfe);//发开始符			//send start byte "token"
	
	for(i=0; i<512; i++)//送512字节数据		//send 512 bytes data
	{
		SPI_WriteByte(*buffer++);
	}
	
	SPI_WriteByte(0xff);			//dummy crc
	SPI_WriteByte(0xff);
	
	r1 = SPI_WriteByte(0xff);
	
	if( (r1&0x1f) != 0x05)//等待是否成功	//judge if it successful
	{
		printf("write result =%02x", r1);
		SPI_CS_Deassert;
		return 1;
	}
	//等待操作完		//wait no busy
	while(SPI_WriteByte(0xff) != 0x00)
	{
		if(retry++ > 0xfffe)
		{
			SPI_CS_Deassert;
			return 2;
		}
	}
	
	// check results
	r1 = MMC_SD_SendCommand(13, 0);
	
	//printf("write result =%02x", r1);
	
	SPI_WriteByte(0xff);// extra 8 CLK
	
	SPI_CS_Deassert;
	SPI_WriteByte(0xff);// extra 8 CLK
	return 0;
}



uint32 MMC_SD_ReadCapacity(void)
{
	uint8 r1;
	uint16 i;
	uint16 temp;
	uint8 buffer[16];
	uint32 Capacity;
	uint16 retry =0;
	//uint8 retry=0;

	SPI_High();		/* Use High Speed SPI*/

	SPI_CS_Assert;
	r1 = MMC_SD_SendCommand(9, 0);//写命令	//send command  //READ CSD
	if(r1 != 0x00)
		return r1;
	

	while(SPI_WriteByte(0xff) != 0xfe)
	{
		if(retry++ > 0xfffe)
		{
			SPI_CS_Deassert;
			return 1;
		}
	}
	
	for(i=0;i<16;i++)
	{
		buffer[i]=SPI_WriteByte(0xff);
	}	

	SPI_WriteByte(0xff);
	SPI_WriteByte(0xff);
	
	SPI_WriteByte(0xff);
	
	SPI_CS_Deassert;

	SPI_WriteByte(0xff);// extra 8 CLK

/*********************************/
//	C_SIZE
	i = buffer[6]&0x03;
	i<<=8;
	i += buffer[7];
	i<<=2;
	i += ((buffer[8]&0xc0)>>6);

/**********************************/
//  C_SIZE_MULT

	r1 = buffer[9]&0x03;
	r1<<=1;
	r1 += ((buffer[10]&0x80)>>7);


/**********************************/
// BLOCKNR

	r1+=2;

	temp = 1;
	while(r1)
	{
		temp*=2;
		r1--;
	}
	
	Capacity = ((uint32)(i+1))*((uint32)temp);

/////////////////////////
// READ_BL_LEN

	i = buffer[5]&0x0f;

/*************************/
//BLOCK_LEN

	temp = 1;
	while(i)
	{
		temp*=2;
		i--;
	}
/************************/


/************** formula of the capacity ******************/
//
//  memory capacity = BLOCKNR * BLOCK_LEN
//	
//	BLOCKNR = (C_SIZE + 1)* MULT
//
//           C_SIZE_MULT+2
//	MULT = 2
//
//               READ_BL_LEN
//	BLOCK_LEN = 2
/**********************************************/

//The final result
	
	Capacity *= (uint32)temp;	 
	return Capacity;		
}
