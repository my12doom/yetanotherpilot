#include "NRF24L01.h"
#include "stm32f10x_spi.h"
#include "..\common\timer.h"

#define NRF_CSN_HIGH(x) GPIO_SetBits(GPIOA,GPIO_Pin_1)
#define NRF_CSN_LOW(x) GPIO_ResetBits(GPIOA,GPIO_Pin_1)
#define NRF_CE_LOW(x) GPIO_ResetBits(GPIOA,GPIO_Pin_2)
#define NRF_CE_HIGH(x) GPIO_SetBits(GPIOA,GPIO_Pin_2)
#define NRF_Read_IRQ(x) GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)

#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define NRF_WRITE_REG   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ

#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;



#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define FIFO_STATUS     0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;

void power_off(void);
u8 SPI_NRF_RW(u8 dat);
u8 SPI_NRF_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes);
u8 SPI_NRF_ReadBuf(u8 reg,u8 *pBuf,u8 bytes);
u8 SPI_NRF_WriteReg(u8 reg,u8 dat);
u8 SPI_NRF_ReadReg(u8 reg);

u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; //���͵�ַ
u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; //���͵�ַ

void SPI_NRF_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//ʹ�� GPIOB,GPIOD,���ù���ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

	//ʹ�� SPI1 ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	//���� SPI_NRF_SPI �� SCK,MISO,MOSI ���ţ�GPIOA^5,GPIOA^6,GPIOA^7 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù���
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//���� SPI_NRF_SPI �� CE ���ţ�GPIOA^2 �� SPI_NRF_SPI �� CSN ����: NSS GPIOA^1*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//���� SPI_NRF_SPI ��IRQ ���ţ�GPIOA^3*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;  //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// �����Զ���ĺ꣬�������� csn ���ţ�NRF �������״̬ */
	NRF_CSN_HIGH();

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//��ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//���ݴ�С 8 λ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//ʱ�Ӽ��ԣ�����ʱΪ��
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//�� 1 ��������Ч��������Ϊ����ʱ��
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS �ź����������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//8 ��Ƶ��9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//��λ��ǰ
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	// Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE);
}


u8 SPI_NRF_RW(u8 dat)
{
	// �� SPI ���ͻ������ǿ�ʱ�ȴ� */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	// ͨ�� SPI2 ����һ�ֽ����� */
	SPI_I2S_SendData(SPI1, dat);

	// �� SPI ���ջ�����Ϊ��ʱ�ȴ� */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	// Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);
}

u8 SPI_NRF_WriteBuf(u8 reg , u8 *pBuf, u8 bytes)
{
	u8 status, byte_cnt;
	NRF_CE_LOW();
	//�õ� CSN��ʹ�� SPI ����*/
	NRF_CSN_LOW();

	//���ͼĴ�����*/
	status = SPI_NRF_RW(reg);

	//�򻺳���д������*/
	for(byte_cnt = 0; byte_cnt < bytes; byte_cnt++)
		SPI_NRF_RW(*pBuf++);    //д���ݵ�������

	//CSN ���ߣ����*/
	NRF_CSN_HIGH();

	return (status);    //���� NRF24L01 ��״̬
}

u8 SPI_NRF_ReadBuf(u8 reg, u8 *pBuf, u8 bytes)
{
	u8 status, byte_cnt;

	NRF_CE_LOW();
	//�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();

	//���ͼĴ�����*/
	status = SPI_NRF_RW(reg);

	//��ȡ����������*/
	for(byte_cnt = 0; byte_cnt < bytes; byte_cnt++)
		pBuf[byte_cnt] = SPI_NRF_RW(NOP); //��NRF24L01��ȡ����

	//CSN���ߣ����*/
	NRF_CSN_HIGH();

	return status;                //���ؼĴ���״ֵ̬
}

u8 SPI_NRF_WriteReg(u8 reg, u8 dat)
{
	u8 status;
	NRF_CE_LOW();
	//�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();

	//��������Ĵ����� */
	status = SPI_NRF_RW(reg);

	//��Ĵ���д������*/
	SPI_NRF_RW(dat);

	//CSN���ߣ����*/
	NRF_CSN_HIGH();

	//����״̬�Ĵ�����ֵ*/
	return(status);
}

u8 SPI_NRF_ReadReg(u8 reg)
{
	u8 reg_val;

	NRF_CE_LOW();
	//�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();

	//���ͼĴ�����*/
	SPI_NRF_RW(reg);

	//��ȡ�Ĵ�����ֵ */
	reg_val = SPI_NRF_RW(NOP);

	//CSN���ߣ����*/
	NRF_CSN_HIGH();

	return reg_val;
}

u8 NRF_Check(void)
{
	u8 buf[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	u8 buf1[5] = {0};
	u8 i;

	//д�� 5 ���ֽڵĵ�ַ.  */
	SPI_NRF_WriteBuf(NRF_WRITE_REG + TX_ADDR, buf, 5);

	//����д��ĵ�ַ */
	SPI_NRF_ReadBuf(TX_ADDR, buf1, 5);

	//�Ƚ�*/
	for(i = 0; i < 5; i++)
	{
		if(buf1[i] != 0xC2)
			break;
	}

	if(i == 5)
		return 0 ;        //MCU �� NRF �ɹ�����
	else
		return 1 ;        //MCU �� NRF ����������
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
	power_off();
	NRF_CE_LOW();

	SPI_NRF_WriteBuf(NRF_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //д RX �ڵ��ַ

	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_AA, 0x01); //ʹ��ͨ�� 0 ���Զ�Ӧ��

	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01); //ʹ��ͨ�� 0 �Ľ��յ�ַ

	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_CH, CHANAL);   //���� RF ͨ��Ƶ��

	SPI_NRF_WriteReg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //ѡ��ͨ�� 0����Ч���ݿ��

	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_SETUP, 0x27); //���� TX �������,0db����,2Mbps,���������濪��

	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG, 0x0f); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ

	//CE ���ߣ��������ģʽ*/
	NRF_CE_HIGH();
	delayus(10);
}

void NRF_TX_Mode(void)
{
	power_off();
	NRF_CE_LOW();

	SPI_NRF_WriteBuf(NRF_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); //дTX�ڵ��ַ

	SPI_NRF_WriteBuf(NRF_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK

	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_AA, 0x01);  //ʹ��ͨ��0���Զ�Ӧ��

	SPI_NRF_WriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01); //ʹ��ͨ��0�Ľ��յ�ַ

	SPI_NRF_WriteReg(NRF_WRITE_REG + SETUP_RETR, 0x1a); //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��

	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_CH, CHANAL);    //����RFͨ��ΪCHANAL

	SPI_NRF_WriteReg(NRF_WRITE_REG + RF_SETUP, 0x27); //����TX�������,0db����,2Mbps,���������濪��

	SPI_NRF_WriteReg(NRF_WRITE_REG + CONFIG, 0x0e); //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�

	//CE���ߣ����뷢��ģʽ*/
	NRF_CE_HIGH();
	delayus(10); //CEҪ����һ��ʱ��Ž��뷢��ģʽ
}

/*
* ��������NRF_Tx_Dat
* ����  ��������NRF�ķ��ͻ�������д������
* ����  ��txBuf���洢�˽�Ҫ���͵����ݵ����飬�ⲿ����
* ���  �����ͽ�����ɹ�����TXDS,ʧ�ܷ���MAXRT��ERROR
* ����  ���ⲿ����
*/
u8 NRF_Tx_Dat(u8 *txbuf)
{
	u8 state;

	//ceΪ�ͣ��������ģʽ1*/
	NRF_CE_LOW();

	//д���ݵ�TX BUF ��� 32���ֽ�*/
	SPI_NRF_WriteBuf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);

	//CEΪ�ߣ�txbuf�ǿգ��������ݰ� */
	NRF_CE_HIGH();

	//�ȴ���������ж� */
	while(NRF_Read_IRQ() != 0);

	//��ȡ״̬�Ĵ�����ֵ */
	state = SPI_NRF_ReadReg(STATUS);

	//���TX_DS��MAX_RT�жϱ�־*/
	SPI_NRF_WriteReg(NRF_WRITE_REG + STATUS, state);

	SPI_NRF_WriteReg(FLUSH_TX, NOP);		//���TX FIFO�Ĵ���

	//�ж��ж�����*/
	if(state & MAX_TX)						//�ﵽ����ط�����
		return MAX_TX;

	else if(state & TX_OK)					//�������
		return TX_OK;
	else
		return ERROR;						//����ԭ����ʧ��
}


/*
* ��������NRF_Rx_Dat
* ����  �����ڴ�NRF�Ľ��ջ������ж�������
* ����  ��rxBuf�����ڽ��ո����ݵ����飬�ⲿ����
* ���  �����ս����
* ����  ���ⲿ����
*/
u8 NRF_Rx_Dat(u8 *rxbuf)
{
	u8 state;
	int i = 0;
	NRF_CE_HIGH();         //�������״̬
	//�ȴ������ж�*/
	while(NRF_Read_IRQ() != 0 && i++ < 5000);
	if (i == 5000)
		return ERROR;

	NRF_CE_LOW();			//�������״̬
	//��ȡstatus�Ĵ�����ֵ  */
	state = SPI_NRF_ReadReg(STATUS);

	// ����жϱ�־*/
	SPI_NRF_WriteReg(NRF_WRITE_REG + STATUS, state);

	//�ж��Ƿ���յ�����*/
	if(state & RX_OK)											//���յ�����
	{
		SPI_NRF_ReadBuf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);	//��ȡ����
		SPI_NRF_WriteReg(FLUSH_RX, NOP);						//���RX FIFO�Ĵ���
		return RX_OK;
	}
	else
		return ERROR;											//û�յ��κ�����
}
