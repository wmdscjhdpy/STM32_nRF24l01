#include "nrf24l01.h"
#include "stm32f1xx_hal.h"
#include "spi.h"//<���ļ�����cube��ѡ"��������.c/.h����ֵ� ���û�й�ѡ��ѡ��msp�Ǹ�ͷ�ļ��Ѿ��������"
/** 
    * @brief Ƭѡ��(CSN�˿�)��io�����궨�� ��Ҫ�Լ��޸�io��Ϊ���Ӷ�Ӧ��io �����IO����cube����Ϊ��©(����)ģʽ
*/
#define NSS_H HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_SET)
#define NSS_L HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_RESET)

/** 
    * @brief NRF24l01��CE�� ��Ϊ��ͨio���� �޸ķ�ʽͬ��
*/
#define CE_H HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET)
#define CE_L HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET)

#define NRF_SPI hspi1//<ע��HAL�ⶨ��spiʱ��������,�ڴ˴��ĳ��Լ��ľ��
#define NRF_CH_VALUE	0x40 //Ƶ��ֵ ���ͺͽ��ն�Ƶ��Ҫһ��
uint8_t TX_ADDRESS[5]={0x12,0x34,0x56,0x78,0x9a};//<���͵�ַ,�����Զ���

//���Ĵ��� �쳣ʱwhile(1)
static uint8_t NRF_Read_Byte(uint8_t reg)
{
	uint8_t result;
	if(reg&0xE0)while(1);//�Ĵ��������� ������5λ �������Ĵ���!
	NSS_L;
		HAL_SPI_Transmit(&NRF_SPI,&reg,1,0xff);//���Ͷ�ȡָ��
		HAL_SPI_Receive(&NRF_SPI,&result,1,0xff);//��ȡָ��
	NSS_H;
	return result;
}
static void NRF_Write_Byte(uint8_t reg,uint8_t value)
{
	if(reg&0xE0)while(1);//�Ĵ��������� ������5λ �������Ĵ���!
	reg|=0x20;//����д��־
	NSS_L;
		HAL_SPI_Transmit(&NRF_SPI,&reg,1,0xff);
		HAL_SPI_Transmit(&NRF_SPI,&value,1,0xff);
	NSS_H;
}

static uint8_t NRF_Read_Bytes(uint8_t reg,uint8_t* rec,uint8_t size)
{
	uint8_t result;
	if(reg&0xE0)while(1);//�Ĵ��������� ������5λ �������Ĵ���!
	NSS_L;
		HAL_SPI_Transmit(&NRF_SPI,&reg,1,0xff);//���Ͷ�ȡָ��
		HAL_SPI_Receive(&NRF_SPI,rec,size,0xff);//��ȡָ��
	NSS_H;
	return result;
}

static void NRF_Write_Bytes(uint8_t reg,uint8_t* send,uint8_t size)
{
	if(reg&0xE0)while(1);//�Ĵ��������� ������5λ �������Ĵ���!
	reg|=0x20;//����д��־
	NSS_L;
		HAL_SPI_Transmit(&NRF_SPI,&reg,1,0xff);
		HAL_SPI_Transmit(&NRF_SPI,send,size,0xff);
	NSS_H;
}

void NRF24L01_Init_Tx()//���շ��Ͷ˳�ʼ��:��Ӧ���ź�,��ַ5�ֽ�,2Mbps,500us�ط�
{
	NRF_Write_Byte(SETUP_AW, 0x03); // ���õ�ַ���Ϊ 5bytes
	NRF_Write_Byte(FEATURE, 0x01); //ʹ��NO_ACK
	NRF_Write_Bytes(TX_ADDR, TX_ADDRESS, 5); // д�뷢�͵�ַ�� 5 �ֽ�
	//NRF_Write_Bytes(W_TX_PAYLOAD, buf, TX_PLOAD_WIDTH); // д TX FIFO
	NRF_Write_Byte(SETUP_RETR, 0x15); //�Զ��ط���ʱ�ȴ� 500us,�Զ��ط� 5 ��
	NRF_Write_Byte(RF_CH, NRF_CH_VALUE); // ѡ����Ƶ�ŵ�
	NRF_Write_Byte(RF_SETUP, 0x0e); // ���ݴ����� 2Mbps ������
	NRF_Write_Byte(CONFIG, 0x7e); //����Ϊ����ģʽ�� CRC�������ж�
	CE_H;//��ʼ����
}
void NRF24L01_Init_Rx()//���ս��ն˳�ʼ��:��Ӧ���ź�,��ַ5�ֽ�,2Mbps
{
	NRF_Write_Byte(SETUP_AW, 0x03); // ���õ�ַ���Ϊ 5bytes
	NRF_Write_Bytes(RX_ADDR_P0, TX_ADDRESS, 5); //����ͨ�� 0 ��ַ�ͷ����ַ��ͬ
	NRF_Write_Byte(EN_RXADDR, 0x01); // ʹ�ܽ���ͨ�� 0
	NRF_Write_Byte(RF_CH, NRF_CH_VALUE); // ѡ����Ƶ�ŵ�
	NRF_Write_Byte(RX_PW_P0, TX_PLOAD_WIDTH); //���ø��س��ȣ�ʹ�� PIPE0 ����
	NRF_Write_Byte(RF_SETUP, 0x0e); // ���ݴ����� 2Mbps ������
	NRF_Write_Byte(CONFIG, 0x7f); //����Ϊ����ģʽ�� CRC�������ж�
	CE_H;
}
void NRF24L01_TxPacket(uint8_t* TxPack)
{
	NRF_Write_Byte(FLUSH_TX,0);//�������
	NRF_Write_Bytes(WR_TX_PLOAD, TxPack, TX_PLOAD_WIDTH);
}
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t state = 0;

    state=NRF_Read_Byte(STATUS);  //��ȡ״̬�Ĵ�����ֵ
    NRF_Write_Byte(WRITE_REG_NRF+STATUS,state); //���TX_DS��MAX_RT�жϱ�־

    if(state&RX_OK)//���յ�����
    {
        NRF_Read_Bytes(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
        NRF_Write_Byte(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
        return 0;
    }

    return 1;//û�յ��κ�����
}