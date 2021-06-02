/**
******************************************************************************
* @file    bus_spi.c
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   �����ļ���spiӲ����غ���.
******************************************************************************


*/


#include "bus_spi.h"

/**********************************************************************************
**������Ϣ ��SPIM_CSLow(SPI_TypeDef* SPIx)
**�������� :Ϊѡ����SPI ��������ڲ�NSS �ܽ�
**������� ��SPI_TypeDef* SPIx,��ѡ��SPI1,SPI2
**������� ����
***********************************************************************************/
void SPIM_CSLow(void)
{
    //Spi cs assign to this pin,select
    SPI_CSInternalSelected(SPI1, SPI_CS_BIT0, ENABLE);
}

/***********************************************************************************
**������Ϣ ��SPIM_CSHigh(SPI_TypeDef* SPIx)
**�������� :Ϊѡ����SPI ��������ڲ�NSS �ܽ�
**������� ��SPI_TypeDef* SPIx,��ѡ��SPI1,SPI2
**������� ����
************************************************************************************/
void SPIM_CSHigh(void)
{
    //Spi cs release
    SPI_CSInternalSelected(SPI1, SPI_CS_BIT0, DISABLE);
}

/***********************************************************************************
**������Ϣ ��SPIM_TXEn(SPI_TypeDef* SPIx)
**�������� :�ر� SPI ��˫��ģʽ�µ����ݴ��䷽��
**������� ��SPI_TypeDef* SPIx,��ѡ��SPI1,SPI2
**������� ����
************************************************************************************/
void SPIM_TXEn(SPI_TypeDef *SPIx)
{
    //Transmit Enable bit TXEN
    SPI_BiDirectionalLineConfig(SPIx, SPI_Direction_Tx);
}

/************************************************************************************
**������Ϣ ��SPIM_TXDisable(SPI_TypeDef* SPIx)
**�������� :�ر� SPI ��˫��ģʽ�µ����ݴ��䷽��
**������� ��SPI_TypeDef* SPIx,��ѡ��SPI1,SPI2
**������� ����
*************************************************************************************/
void SPIM_TXDisable(SPI_TypeDef *SPIx)
{
    //disable TXEN
    SPI_BiDirectionalLineConfig(SPIx, SPI_Disable_Tx);
}

/************************************************************************************
**������Ϣ ��SPIM_RXEn(SPI_TypeDef* SPIx)
**�������� :�ر� SPI ��˫��ģʽ�µ����ݴ��䷽��
**������� ��SPI_TypeDef* SPIx,��ѡ��SPI1,SPI2
**������� ����
************************************************************************************/
void SPIM_RXEn(SPI_TypeDef *SPIx)
{
    //enable RXEN
    SPI_BiDirectionalLineConfig(SPIx, SPI_Direction_Rx);
}

/***********************************************************************************
**������Ϣ ��SPIM_RXDisable(SPI_TypeDef* SPIx)
**�������� :�ر� SPI ��˫��ģʽ�µ����ݴ��䷽��
**������� ��SPI_TypeDef* SPIx,��ѡ��SPI1,SPI2
**������� ����
***********************************************************************************/
void SPIM_RXDisable(SPI_TypeDef *SPIx)
{
    //disable RXEN
    SPI_BiDirectionalLineConfig(SPIx, SPI_Disable_Rx);
}


/**********************************************************************************
**������Ϣ ��SPIMReadWriteByte(SPI_TypeDef* SPIx,unsigned char tx_data)
**�������� : ͨ������ SPIx �շ����� ,����ȫ˫��ģʽ(ͬʱ�շ�)
**������� ��SPI_TypeDef* SPIx,��ѡ��SPI1,SPI2  ;  tx_data
**������� ����
***********************************************************************************/
unsigned int SPI_RW(unsigned char tx_data)
{
    SPI_SendData(SPI1, tx_data);
    while (1)
    {
        if (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXAVL))
        {
            return SPI_ReceiveData(SPI1);
        }
    }
}


/***************************************************************************************
**������Ϣ ��void spi_init(unsigned short spi_baud_div)
**�������� : ��ʼ��SPI
**������� ��spi_baud_div
**������� ����
****************************************************************************************/
void spi_init(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  //SPI1 clk enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;          //spi1_cs  pa4
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // ���⸴�����
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;          //spi1_sck  pa5
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // ���⸴�����
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;          //spi1_mosi  pa7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // ���⸴�����
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;          //spi1_miso  pa6
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    SPI_Cmd(SPI1, DISABLE);
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_DataWidth = SPI_DataWidth_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;              // mode0 SPI_CPOL_Low, SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;            // mode3 SPI_CPOL_High,SPI_CPHA_2Edge
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = 12;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);                                  //Enables the specified SPI peripheral SPIʹ�ܡ�����ģʽ 8λ����ģʽ SPI �Ĳ�����
    SPIM_TXEn(SPI1);
    SPIM_RXEn(SPI1);
}





