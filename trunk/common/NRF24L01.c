#include "NRF24L01.h"
#include <stm32f10x_spi.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <string.h>
#include "misc.h"
#include "..\common\config.h"
#include "..\common\timer.h"


#if PCB_VERSION == 1 || defined(STATION)
#define NRF_CSN_HIGH(x) GPIO_SetBits(GPIOA,GPIO_Pin_1)
#define NRF_CSN_LOW(x) GPIO_ResetBits(GPIOA,GPIO_Pin_1)
#define NRF_CE_LOW(x) GPIO_ResetBits(GPIOA,GPIO_Pin_2)
#define NRF_CE_HIGH(x) GPIO_SetBits(GPIOA,GPIO_Pin_2)
#define NRF_Read_IRQ(x) GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)

#elif PCB_VERSION == 2
#define NRF_CSN_HIGH(x) GPIO_SetBits(GPIOA,GPIO_Pin_3)
#define NRF_CSN_LOW(x) GPIO_ResetBits(GPIOA,GPIO_Pin_3)
#define NRF_CE_LOW(x) GPIO_ResetBits(GPIOA,GPIO_Pin_15)
#define NRF_CE_HIGH(x) GPIO_SetBits(GPIOA,GPIO_Pin_15)
#define NRF_Read_IRQ(x) GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)

#elif PCB_VERSION == 3
#define NRF_CSN_HIGH(x) GPIO_SetBits(GPIOA,GPIO_Pin_15)
#define NRF_CSN_LOW(x) GPIO_ResetBits(GPIOA,GPIO_Pin_15)
#define NRF_CE_LOW(x) GPIO_ResetBits(GPIOC,GPIO_Pin_15)
#define NRF_CE_HIGH(x) GPIO_SetBits(GPIOC,GPIO_Pin_15)
#define NRF_Read_IRQ(x) GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)

#endif

#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define NRF_WRITE_REG   0x20  //写配置寄存器,低5位为寄存器地址
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前

#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;



#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define FIFO_STATUS     0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;

void power_off(void);
u8 SPI_NRF_RW(u8 dat);
u8 SPI_NRF_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes);
u8 SPI_NRF_ReadBuf(u8 reg,u8 *pBuf,u8 bytes);
u8 SPI_NRF_WriteReg(u8 reg,u8 dat);
u8 SPI_NRF_ReadReg(u8 reg);

u8 TX_ADDRESS[TX_ADR_WIDTH] = {0xb0,0x3d,0x12,0x34,0x01}; //发送地址
u8 RX_ADDRESS[RX_ADR_WIDTH] = {0xb0,0x3d,0x12,0x34,0x01}; //发送地址

u8 tx_queue[TX_QUEUE_SIZE * TX_QUEUE_ITEM_SIZE];
int tx_queue_count = 0;
int busy = 0;
int rxmode = 0;
int rxirq = 0;

static void lockEXTI3()
{
	NVIC_DisableIRQ(EXTI3_IRQn);
	__DSB();
	__ISB();
}

static void unlockEXTI3()
{
	NVIC_EnableIRQ(EXTI3_IRQn);
}

static void handleQueue(int result)
{
	lockEXTI3();
	
	if (busy)
	{
		unlockEXTI3();
		return;
	}

	if (tx_queue_count > 0)
	{
		nrf_callback cb = *(nrf_callback*)(tx_queue+TX_PLOAD_WIDTH+1);
		int user_data = *(int*)(tx_queue+TX_PLOAD_WIDTH+5);
		if (cb && result>0)
			cb(result, user_data);


		if ( result >0 && (result & TX_OK || !tx_queue[TX_PLOAD_WIDTH])) // check success or no confirm flag
		{
			tx_queue_count --;
			memmove(tx_queue, tx_queue+TX_QUEUE_ITEM_SIZE, TX_QUEUE_ITEM_SIZE * tx_queue_count);
		}
		busy = 1;
		
		//ce为低，进入待机模式1*/
		NRF_CE_LOW();

		//写数据到TX BUF 最大 32个字节*/
		SPI_NRF_WriteBuf(WR_TX_PLOAD, tx_queue, TX_PLOAD_WIDTH);

		//CE为高，txbuf非空，发送数据包 */
		NRF_CE_HIGH();

	}

	unlockEXTI3();
}

void NRF_Init(void)
{
	u8 state;
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//使能 GPIOB,GPIOD,复用功能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//使能 SPI1 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	//配置 SPI_NRF_SPI 的 SCK,MISO,MOSI 引脚，GPIOA^5,GPIOA^6,GPIOA^7
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
#ifdef STATION
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#else
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

	// 和  CSN 引脚: NSS GPIOA^1(PCB1.0) / GPIOA^3(PCB2.0)  / GPIOA^115(PCB3.0)
	//配置 CE 引脚，GPIOA^2(PCB1.0)/GPIOA^15(PCB2.0)/GPIOC^15(PCB3.0) 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#if PCB_VERSION == 1  || defined(STATION)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_1;
#elif PCB_VERSION == 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_3;
#elif PCB_VERSION == 3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//配置 SPI_NRF_SPI 的IRQ 引脚，GPIOA^3(PCB1.0) / GPIOB^2(PCB2.0) / GPIOA^4(PCB3.0)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //上拉输入
#if PCB_VERSION == 1  || defined(STATION)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#elif PCB_VERSION == 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#elif PCB_VERSION == 3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

	// 这是自定义的宏，用于拉高 csn 引脚，NRF 进入空闲状态
	NRF_CSN_HIGH();

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//数据大小 8 位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//时钟极性，空闲时为低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//第 1 个边沿有效，上升沿为采样时刻
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS 信号由软件产生
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//8 分频，9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);
	
	
#ifdef STATION
	GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
#endif
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

#ifndef STATION
	state = SPI_NRF_ReadReg(STATUS);
	SPI_NRF_WriteReg(NRF_WRITE_REG + STATUS, state);	// 清除TX_DS或MAX_RT中断标志
	SPI_NRF_WriteReg(FLUSH_TX, NOP);		//清除TX FIFO寄存器
#endif

	SPI_NRF_WriteReg(FLUSH_TX, NOP);
	SPI_NRF_WriteReg(FLUSH_RX, NOP);
}
int t3 = 0;
int tx_ok = 0;
int max_retry = 0;
void EXTI3_IRQHandler(void)
{
	u8 state;
	EXTI_ClearITPendingBit(EXTI_Line3);
	
	t3++;	
	
	if (rxmode)
	{
		rxirq = 1;
		return ;
	}
	
	lockEXTI3();
	// 读取状态寄存器的值
	state = SPI_NRF_ReadReg(STATUS);

	// 清除TX_DS或MAX_RT中断标志
	SPI_NRF_WriteReg(NRF_WRITE_REG + STATUS, state);

	SPI_NRF_WriteReg(FLUSH_TX, NOP);		//清除TX FIFO寄存器

	// 判断中断类型
	if(state & MAX_TX)						//达到最大重发次数
		max_retry++;

	else if(state & TX_OK)					//发送完成
		tx_ok++;
	else
		;
	busy = 0;
	unlockEXTI3();

	handleQueue(state);
}


u8 SPI_NRF_RW(u8 dat)
{
	// 当 SPI 发送缓冲器非空时等待 */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	// 通过 SPI2 发送一字节数据 */
	SPI_I2S_SendData(SPI1, dat);

	// 当 SPI 接收缓冲器为空时等待 */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	// Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);
}

u8 SPI_NRF_WriteBuf(u8 reg , u8 *pBuf, u8 bytes)
{
	u8 status, byte_cnt;
	NRF_CE_LOW();
	//置低 CSN，使能 SPI 传输*/
	NRF_CSN_LOW();

	//发送寄存器号*/
	status = SPI_NRF_RW(reg);

	//向缓冲区写入数据*/
	for(byte_cnt = 0; byte_cnt < bytes; byte_cnt++)
		SPI_NRF_RW(*pBuf++);    //写数据到缓冲区

	//CSN 拉高，完成*/
	NRF_CSN_HIGH();

	return (status);    //返回 NRF24L01 的状态
}

u8 SPI_NRF_ReadBuf(u8 reg, u8 *pBuf, u8 bytes)
{
	u8 status, byte_cnt;

	NRF_CE_LOW();
	//置低CSN，使能SPI传输*/
	NRF_CSN_LOW();

	//发送寄存器号*/
	status = SPI_NRF_RW(reg);

	//读取缓冲区数据*/
	for(byte_cnt = 0; byte_cnt < bytes; byte_cnt++)
		pBuf[byte_cnt] = SPI_NRF_RW(NOP); //从NRF24L01读取数据

	//CSN拉高，完成*/
	NRF_CSN_HIGH();

	return status;                //返回寄存器状态值
}

u8 SPI_NRF_WriteReg(u8 reg, u8 dat)
{
	u8 status;
	NRF_CE_LOW();
	//置低CSN，使能SPI传输*/
	NRF_CSN_LOW();

	//发送命令及寄存器号 */
	status = SPI_NRF_RW(reg);

	//向寄存器写入数据*/
	SPI_NRF_RW(dat);

	//CSN拉高，完成*/
	NRF_CSN_HIGH();

	//返回状态寄存器的值*/
	return(status);
}

u8 SPI_NRF_ReadReg(u8 reg)
{
	u8 reg_val;

	NRF_CE_LOW();
	//置低CSN，使能SPI传输*/
	NRF_CSN_LOW();

	//发送寄存器号*/
	SPI_NRF_RW(reg);

	//读取寄存器的值 */
	reg_val = SPI_NRF_RW(NOP);

	//CSN拉高，完成*/
	NRF_CSN_HIGH();

	return reg_val;
}

u8 NRF_Check(void)
{
	u8 buf[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	u8 buf1[5] = {0};
	u8 i;

	//写入 5 个字节的地址.  */
	SPI_NRF_WriteBuf(NRF_WRITE_REG + TX_ADDR, buf, 5);

	//读出写入的地址 */
	SPI_NRF_ReadBuf(TX_ADDR, buf1, 5);

	//比较*/
	for(i = 0; i < 5; i++)
	{
		if(buf1[i] != 0xC2)
			break;
	}

	if(i == 5)
		return 0 ;        //MCU 与 NRF 成功连接
	else
		return 1 ;        //MCU 与 NRF 不正常连接
}

void power_off()
{
	NRF_CE_LOW();
	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG, 0x0D);
	NRF_CE_HIGH();
	delayus(10);
}

void NRF_RX_Mode(void)

{
	rxmode = 1;
	busy = 0;
	power_off();
	NRF_CE_LOW();

	SPI_NRF_WriteBuf(NRF_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //写 RX 节点地址

	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_AA, 0x01); //使能通道 0 的自动应答

	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01); //使能通道 0 的接收地址

	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_CH, CHANAL);   //设置 RF 通信频率

	SPI_NRF_WriteReg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //选择通道 0的有效数据宽度

	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_SETUP, 0x27); //设置 TX 发射参数,0db增益,2Mbps,低噪声增益开启

	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG, 0x0f); //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式

	//CE 拉高，进入接收模式*/
	NRF_CE_HIGH();
	delayus(10);
}

void NRF_TX_Mode(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	rxmode = 0;
	busy = 0;
	power_off();
	NRF_CE_LOW();

	// Set EXTI
	/*
	EXTI_ClearITPendingBit(EXTI_Line3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	SPI_NRF_WriteBuf(NRF_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); //写TX节点地址

	SPI_NRF_WriteBuf(NRF_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK

	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_AA, 0x01);  //使能通道0的自动应答

	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01); //使能通道0的接收地址

	SPI_NRF_WriteReg(NRF_WRITE_REG + SETUP_RETR, 0x1a); //设置自动重发间隔时间:250us + 86us;最大自动重发次数:5次

	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_CH, CHANAL);    //设置RF通道为CHANAL

	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_SETUP, 0x27); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启

	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG, 0x0e); //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式,开启所有中断

	//CE拉高，进入发送模式*/
	NRF_CE_HIGH();
	delayus(10); //CE要拉高一段时间才进入发送模式
}

/*
* 函数名：NRF_Tx_Dat
* 描述  ：用于向NRF的发送缓冲区中写入数据
* 输入  ：txBuf：存储了将要发送的数据的数组，外部定义
* 输出  ：发送结果，成功返回TXDS,失败返回MAXRT或ERROR
* 调用  ：外部调用
*/
u8 NRF_Tx_DatEx(u8 *txbuf, int confirm, nrf_callback cb, int user_data)
{
	if (NRF_Read_IRQ() == 0)
		EXTI3_IRQHandler();
	
	lockEXTI3();	

	if (tx_queue_count < TX_QUEUE_SIZE &&!(tx_queue_count >= TX_QUEUE_SIZE/2 && getus()%2==0))		// 50% packet drop if queue is more than half full
	{			
		memcpy(tx_queue+TX_QUEUE_ITEM_SIZE*tx_queue_count, txbuf, TX_PLOAD_WIDTH);
		(tx_queue+TX_QUEUE_ITEM_SIZE*tx_queue_count)[TX_PLOAD_WIDTH] = confirm;		//confirm flag
		*((nrf_callback*)(tx_queue+TX_QUEUE_ITEM_SIZE*tx_queue_count + TX_PLOAD_WIDTH+1)) = cb;	// callback func
		*((int*)(tx_queue+TX_QUEUE_ITEM_SIZE*tx_queue_count + TX_PLOAD_WIDTH+5)) = user_data;	// user data


		tx_queue_count ++;

		unlockEXTI3();

		handleQueue(-1);

		return TX_OK;
	}
	else
	{
		unlockEXTI3();
		return TX_BUSY;
	}
}

u8 NRF_Tx_Dat(u8 *txbuf)
{
	return NRF_Tx_DatEx(txbuf, 0, NULL, 0);
}

void NRF_Handle_Queue(void)
{
	if (NRF_Read_IRQ() == 0)
		EXTI3_IRQHandler();
	//if (!busy)
		handleQueue(-1);			// no dequeueing, just sending
}


/*
* 函数名：NRF_Rx_Dat
* 描述  ：用于从NRF的接收缓冲区中读出数据
* 输入  ：rxBuf：用于接收该数据的数组，外部定义
* 输出  ：接收结果，
* 调用  ：外部调用
*/
u8 NRF_Rx_Dat(u8 *rxbuf)
{
	u8 state;
	int i = 0;

	if (!rxmode)
		return 0;

	NRF_CE_HIGH();         //进入接收状态
	//等待接收中断*/
	while(NRF_Read_IRQ() != 0 && i++ < 5000);
	if (i == 5000)
		return 0;

	NRF_CE_LOW();			//进入待机状态
	//读取status寄存器的值  */
	state = SPI_NRF_ReadReg(STATUS);

	// 清除中断标志*/
	SPI_NRF_WriteReg(NRF_WRITE_REG + STATUS, state);

	//判断是否接收到数据*/
	if(state & RX_OK)											//接收到数据
	{
		SPI_NRF_ReadBuf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);	//读取数据
		SPI_NRF_WriteReg(FLUSH_RX, NOP);						//清除RX FIFO寄存器
		return RX_OK;
	}
	else
		return 0;											//没收到任何数据
}
