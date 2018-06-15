#ifndef _NRF24L01_H
#define _NRF24L01_H

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************NRF24L01*************************************
#define RX_DR   (g_rf_status&0x40)
#define TX_DS   (g_rf_status&0x20)
#define MAX_RT  (g_rf_status&0x10)
//24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5       //5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5       //5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  6   //32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  6   //32�ֽڵ��û����ݿ��

//***************************************NRF24L01�Ĵ���ָ��*******************************************************
#define READ_REG_NRF    0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define WRITE_REG_NRF   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61    // ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0    // д��������ָ��
#define FLUSH_TX        0xE1    // ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2    // ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3    // �����ظ�װ������ָ��
#define NOP             0xFF    // ����
//*************************************SPI(nRF24L01)�Ĵ�����ַ****************************************************
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define STATUS          0x07  // ״̬�Ĵ���

#define MAX_TX          0x10  //�ﵽ����ʹ����ж�
#define TX_OK           0x20  //TX��������ж�
#define RX_OK           0x40  //���յ������ж�

#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������
#define FEATURE					0x1D	//�����Ĵ���
//											Reserved 7:3 0 R/W ����
//											EN_DPL 2 0 R/W ʹ�ܶ�̬���س���
//											EN_ACK_PAYd 1 0 R/W ʹ��ACK����(���������ݵ�ACK��)
//											EN_DYN_ACK 0 0 R/W ʹ������W_TX_PAYLOAD_NOACK
//**************************************************************************************
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
