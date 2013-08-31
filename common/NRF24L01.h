#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "stm32f10x.h"

// definition and prototypes
#define CHANAL 100		// 通信频道
#define TX_ADR_WIDTH    5   //5字节的地址宽度
#define RX_ADR_WIDTH    5   //5字节的地址宽度
#define TX_PLOAD_WIDTH  32  //20字节的用户数据宽度
#define RX_PLOAD_WIDTH  32  //20字节的用户数据宽度
#define TX_QUEUE_SIZE 50
#define MAX_TX  	0x10  //达到最大发送次数中断
#define TX_OK   	0x20  //TX发送完成中断
#define RX_OK   	0x40  //接收到数据中断
#define TX_BUSY   0x80  // 发送队列满

void SPI_NRF_Init(void);
u8 NRF_Check(void);
void NRF_RX_Mode(void);
void NRF_TX_Mode(void);
u8 NRF_Tx_Dat(u8 *txbuf);
u8 NRF_Rx_Dat(u8 *rxbuf);

extern u8 TX_ADDRESS[TX_ADR_WIDTH];//={0xb0,0x3d,0x12,0x34,0x01}; //发送地址
extern u8 RX_ADDRESS[RX_ADR_WIDTH];//={0xb0,0x3d,0x12,0x34,0x01}; //发送地址

#endif
