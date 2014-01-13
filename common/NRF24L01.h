#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "stm32f10x.h"


// definition and prototypes
#define CHANAL 100		// ͨ��Ƶ��
#define TX_ADR_WIDTH    5   //5�ֽڵĵ�ַ����
#define RX_ADR_WIDTH    5   //5�ֽڵĵ�ַ����
#define TX_PLOAD_WIDTH  32  //20�ֽڵ��û����ݿ���
#define RX_PLOAD_WIDTH  32  //20�ֽڵ��û����ݿ���
#define TX_QUEUE_SIZE 8
#define MAX_TX  	0x10  //�ﵽ����ʹ����ж�
#define TX_OK   	0x20  //TX��������ж�
#define RX_OK   	0x40  //���յ������ж�
#define TX_BUSY   0x80  // ���Ͷ�����

#ifdef __cplusplus
extern "C" {
#endif

void NRF_Init(void);
u8 NRF_Check(void);
void NRF_RX_Mode(void);
void NRF_TX_Mode(void);
void NRF_Handle_Queue(void);
u8 NRF_Tx_Dat(u8 *txbuf);
u8 NRF_Rx_Dat(u8 *rxbuf);

#ifdef __cplusplus
}
#endif

#endif