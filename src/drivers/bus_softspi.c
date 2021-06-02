/**
******************************************************************************
* @file    bus_spi.c
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   驱动文件，模拟spi相关函数.
******************************************************************************

三线spi: 三线与四线的区别在于DATA线.  mosi 和miso 合并为一根data线 就是标准的三线


data: PA1
clk: PB3
cs: PB5


*/


#include "bus_softspi.h"
#include "targets.h"
#include "time.h"
#include "bus_spi.h"


#define MOSIHIGH gpioset( SPI_MOSI_PORT, SPI_MOSI_PIN)
#define MOSILOW gpioreset( SPI_MOSI_PORT, SPI_MOSI_PIN);
#define SCKHIGH gpioset( SPI_CLK_PORT, SPI_CLK_PIN);
#define SCKLOW gpioreset( SPI_CLK_PORT, SPI_CLK_PIN);

extern u32 old_freq;
extern uint8_t rgb_led;
extern unsigned char rx_select;


GPIO_InitTypeDef mosi_init_struct;
int mosi_out = 0;
#define WriteCOM 0x01
#define ReadData 0x00
/************************************************************************************
**函数信息 ：void mosi_input(void)
**功能描述 : 初始化softSPI
**输入参数 ：无
**输出参数 ：无

以下几个为模拟三线spi的读写函数

************************************************************************************/
void mosi_input(void)
{
    if (mosi_out)
    {
        mosi_out = 0;
        mosi_init_struct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct);
    }

}

void mosi_output(void)
{
    if (!mosi_out)
    {
        mosi_out = 1;
        mosi_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct);
    }

}


void spi_cson()
{
    SPI_SS_PORT->BRR = SPI_SS_PIN;
}

void spi_csoff()
{
    SPI_SS_PORT->BSRR = SPI_SS_PIN;
}

void spi_sendbyte(int data)
{
    mosi_output();
    for (int i = 7 ; i >= 0 ; i--)
    {
        if ((data >> i) & 1)
        {
            MOSIHIGH;
        }
        else
        {
            MOSILOW;
        }

        SCKHIGH;
        SCKLOW;
    }
}


int spi_recvbyte(void)
{
    int recv = 0;

    for (int i = 7 ; i >= 0 ; i--)
    {

        SCKHIGH;

        recv = recv << 1;

        recv = recv | ((SPI_MOSI_PORT->IDR & (int)SPI_MOSI_PIN) ? 1 : 0);

        SCKLOW;

    }

    return recv;
}


void RTC6705_cson()
{
    RTC6705_CS_PORT->BRR  = RTC6705_CS_PIN;
}

void RTC6705_csoff()
{
    RTC6705_CS_PORT->BSRR = RTC6705_CS_PIN;
}

void SoftSpiDelay(void)
{
    u8 Temp = 20;
    while (Temp)Temp--;
}
/***************************************************************************************************
    During write cycle (R/W = 1), the chip will sample the SPIDATA on the rising edge of SPICLK.
    Sampled data will be temporally stored in internal shift register. One the rising edge of SPILE,
    data in shift register will be latched into specific register according to the address.
***************************************************************************************************/
void RTC6705_Write(u8 Addr, u32 Data) //Write REG
{
    u8 SPI_Bitx = 25;           // 4 bit addr + 1 bit Write/Read + 20 bit data
    Data = (Data << 5) | 0x10 | (Addr & 0x0f);

    SoftSpiDelay();
    SCKLOW;
    SoftSpiDelay();
    RTC6705_cson();
    SoftSpiDelay();

    while (SPI_Bitx != 0)
    {
        SoftSpiDelay();
        if (Data & 0x01)
            MOSIHIGH;
        else
            MOSILOW;
        SoftSpiDelay();
        SoftSpiDelay();
        SCKHIGH;
        SoftSpiDelay();
        SoftSpiDelay();
        SCKLOW;
        Data >>= 1;
        SPI_Bitx--;
    }
    SoftSpiDelay();
    RTC6705_csoff();
    SoftSpiDelay();
}


static void rtc6705_write_register(uint8_t addr, uint32_t data)
{
    RTC6705_cson();
    delay(1);
    // send address
    for (int i = 0; i < 4; i++)
    {
        if ((addr >> i) & 1)
        {
            MOSIHIGH;
        }
        else
        {
            MOSILOW;
        }

        SCKHIGH;
        delay(1);
        SCKLOW;
        delay(1);
    }
    // Write bit
    MOSIHIGH;
    SCKHIGH;
    delay(1);
    SCKLOW;
    delay(1);
    for (int i = 0; i < 20; i++)
    {
        if ((data >> i) & 1)
        {
            MOSIHIGH;
        }
        else
        {
            MOSILOW;
        }
        SCKHIGH;
        delay(1);
        SCKLOW;
        delay(1);
    }
    RTC6705_csoff();
}

void rtc6705SoftSpiSetFrequency(uint16_t channel_freq)
{
    uint32_t freq = (uint32_t)channel_freq * 1000;
    freq /= 40;
    const uint32_t N = freq / 64;
    const uint32_t A = freq % 64;
    rtc6705_write_register(0, 400);
    rtc6705_write_register(1, (N << 7) | A);
}
/***************************************************************************************************
    RTC6705 During read cycle (R/W = 0),
    address and read/write control bit are sampled at rising edge of SPICLK,
    but the data bits are sent at the falling edge of SPICLK.
***************************************************************************************************/
u32 RTC6705_Read(u8 Addr)
{
    u8 i;
    u32 Data = 0;
    Addr &= 0x0f;
//    RTC6705_Write(0x0F,0x000000);//Reset RTC6705
    SoftSpiDelay();
    SCKLOW;
    SoftSpiDelay();
    RTC6705_cson();
    SoftSpiDelay();
    for (i = 0; i < 5; i++) // send 4 bit addr + 1 bit Read
    {
        SoftSpiDelay();
        if (Addr & 0x01)
            MOSIHIGH;
        else
            MOSILOW;
        SoftSpiDelay();
        SoftSpiDelay();
        SoftSpiDelay();
        SCKHIGH;
        SoftSpiDelay();
        SoftSpiDelay();
        SCKLOW;
        Addr >>= 1;
    }
    mosi_input();
    for (i = 0; i < 20; i++)
    {
        if (GPIO_ReadInputDataBit(SPI_MOSI_PORT, SPI_MOSI_PIN))
        {
            Data |= 0x01 << i;
        }
        SoftSpiDelay();
        SoftSpiDelay();
        SCKHIGH;
        SoftSpiDelay();
        SoftSpiDelay();
        SCKLOW;
    }
    mosi_output();
    SoftSpiDelay();
    RTC6705_csoff();
    SoftSpiDelay();
    return Data;
}
#define D_Fosc  0x08
#define D_RTC6705_SYN_RF_R_REG  0x190
void Set_RTC6705_Freq(u32 Freq)
{
//    RTC6705_Write(0x0F,0x000000); //Reset RTC6705
    Freq = (((((Freq * D_RTC6705_SYN_RF_R_REG) / D_Fosc) / 2) / 64) << 7)
           | (((((Freq * D_RTC6705_SYN_RF_R_REG) / D_Fosc) / 2) % 64) & 0x7F);
    RTC6705_Write(0x01, Freq);
}
/************************************************************************************
**函数信息 ：void softspi_init(void)
**功能描述 : 初始化softSPI
**输入参数 ：无
**输出参数 ：无
************************************************************************************/
void softspi_init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //ENABLE AFIO Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //通用推免输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = SPI_CLK_PIN;
    GPIO_Init(SPI_CLK_PORT, &GPIO_InitStructure);

    if(rx_select ==5)
    {
        GPIO_InitStructure.GPIO_Pin = SPI_SS_PIN;
        GPIO_Init(SPI_SS_PORT, &GPIO_InitStructure);

        spi_csoff();
    }

    GPIO_InitStructure.GPIO_Pin = RTC6705_CS_PIN;
    GPIO_Init(RTC6705_CS_PORT, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);    // JTAG Disable,Release the PB3

    mosi_init_struct.GPIO_Pin = SPI_MOSI_PIN;
    mosi_init_struct.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
    mosi_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct);
    SCKLOW;
    MOSILOW;
    RTC6705_Write(0x0F, 0x000000); //Reset RTC6705
    // Optional
    delay_ms(5);

    if (!old_freq)
        old_freq = 5733;
    Set_RTC6705_Freq(old_freq);
}




