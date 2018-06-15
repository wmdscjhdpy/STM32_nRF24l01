#include "nrf24l01.h"
#include "stm32f1xx_hal.h"
#include "spi.h"//<该文件是在cube勾选"单独生成.c/.h后出现的 如果没有勾选则选择msp那个头文件把句柄括进来"
/** 
    * @brief 片选线(CSN端口)的io操作宏定义 需要自己修改io口为板子对应的io 建议该IO口在cube设置为开漏(上拉)模式
*/
#define NSS_H HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_SET)
#define NSS_L HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_RESET)

/** 
    * @brief NRF24l01的CE脚 作为普通io就行 修改方式同上
*/
#define CE_H HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET)
#define CE_L HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET)

#define NRF_SPI hspi1//<注意HAL库定义spi时会给出句柄,在此处改成自己的句柄
#define NRF_CH_VALUE	0x40 //频道值 发送和接收端频道要一样
uint8_t TX_ADDRESS[5]={0x12,0x34,0x56,0x78,0x9a};//<发送地址,可以自定义

//读寄存器 异常时while(1)
static uint8_t NRF_Read_Byte(uint8_t reg)
{
	uint8_t result;
	if(reg&0xE0)while(1);//寄存器有问题 超过了5位 检查输入寄存器!
	NSS_L;
		HAL_SPI_Transmit(&NRF_SPI,&reg,1,0xff);//发送读取指令
		HAL_SPI_Receive(&NRF_SPI,&result,1,0xff);//读取指令
	NSS_H;
	return result;
}
static void NRF_Write_Byte(uint8_t reg,uint8_t value)
{
	if(reg&0xE0)while(1);//寄存器有问题 超过了5位 检查输入寄存器!
	reg|=0x20;//加上写标志
	NSS_L;
		HAL_SPI_Transmit(&NRF_SPI,&reg,1,0xff);
		HAL_SPI_Transmit(&NRF_SPI,&value,1,0xff);
	NSS_H;
}

static uint8_t NRF_Read_Bytes(uint8_t reg,uint8_t* rec,uint8_t size)
{
	uint8_t result;
	if(reg&0xE0)while(1);//寄存器有问题 超过了5位 检查输入寄存器!
	NSS_L;
		HAL_SPI_Transmit(&NRF_SPI,&reg,1,0xff);//发送读取指令
		HAL_SPI_Receive(&NRF_SPI,rec,size,0xff);//读取指令
	NSS_H;
	return result;
}

static void NRF_Write_Bytes(uint8_t reg,uint8_t* send,uint8_t size)
{
	if(reg&0xE0)while(1);//寄存器有问题 超过了5位 检查输入寄存器!
	reg|=0x20;//加上写标志
	NSS_L;
		HAL_SPI_Transmit(&NRF_SPI,&reg,1,0xff);
		HAL_SPI_Transmit(&NRF_SPI,send,size,0xff);
	NSS_H;
}

void NRF24L01_Init_Tx()//按照发送端初始化:无应答信号,地址5字节,2Mbps,500us重发
{
	NRF_Write_Byte(SETUP_AW, 0x03); // 设置地址宽度为 5bytes
	NRF_Write_Byte(FEATURE, 0x01); //使能NO_ACK
	NRF_Write_Bytes(TX_ADDR, TX_ADDRESS, 5); // 写入发送地址， 5 字节
	//NRF_Write_Bytes(W_TX_PAYLOAD, buf, TX_PLOAD_WIDTH); // 写 TX FIFO
	NRF_Write_Byte(SETUP_RETR, 0x15); //自动重发延时等待 500us,自动重发 5 次
	NRF_Write_Byte(RF_CH, NRF_CH_VALUE); // 选择射频信道
	NRF_Write_Byte(RF_SETUP, 0x0e); // 数据传输率 2Mbps 及功率
	NRF_Write_Byte(CONFIG, 0x7e); //配置为发射模式、 CRC、屏蔽中断
	CE_H;//开始工作
}
void NRF24L01_Init_Rx()//按照接收端初始化:无应答信号,地址5字节,2Mbps
{
	NRF_Write_Byte(SETUP_AW, 0x03); // 设置地址宽度为 5bytes
	NRF_Write_Bytes(RX_ADDR_P0, TX_ADDRESS, 5); //接收通道 0 地址和发射地址相同
	NRF_Write_Byte(EN_RXADDR, 0x01); // 使能接收通道 0
	NRF_Write_Byte(RF_CH, NRF_CH_VALUE); // 选择射频信道
	NRF_Write_Byte(RX_PW_P0, TX_PLOAD_WIDTH); //设置负载长度，使用 PIPE0 接收
	NRF_Write_Byte(RF_SETUP, 0x0e); // 数据传输率 2Mbps 及功率
	NRF_Write_Byte(CONFIG, 0x7f); //配置为接收模式、 CRC、屏蔽中断
	CE_H;
}
void NRF24L01_TxPacket(uint8_t* TxPack)
{
	NRF_Write_Byte(FLUSH_TX,0);//清空数据
	NRF_Write_Bytes(WR_TX_PLOAD, TxPack, TX_PLOAD_WIDTH);
}
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t state = 0;

    state=NRF_Read_Byte(STATUS);  //读取状态寄存器的值
    NRF_Write_Byte(WRITE_REG_NRF+STATUS,state); //清除TX_DS或MAX_RT中断标志

    if(state&RX_OK)//接收到数据
    {
        NRF_Read_Bytes(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
        NRF_Write_Byte(FLUSH_RX,0xff);//清除RX FIFO寄存器
        return 0;
    }

    return 1;//没收到任何数据
}