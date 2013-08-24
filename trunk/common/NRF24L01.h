#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "stm32f10x.h"

// definition and prototypes
#define CHANAL 40		// ͨ��Ƶ��
#define TX_ADR_WIDTH    5   //5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   //5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32  //20�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32  //20�ֽڵ��û����ݿ��
#define MAX_TX  	0x10  //�ﵽ����ʹ����ж�
#define TX_OK   	0x20  //TX��������ж�
#define RX_OK   	0x40  //���յ������ж�

void SPI_NRF_Init(void);
u8 NRF_Check(void);
void NRF_RX_Mode(void);
void NRF_TX_Mode(void);
u8 NRF_Tx_Dat(u8 *txbuf);
u8 NRF_Rx_Dat(u8 *rxbuf);

extern u8 TX_ADDRESS[TX_ADR_WIDTH];//={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
extern u8 RX_ADDRESS[RX_ADR_WIDTH];//={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ

#endif
