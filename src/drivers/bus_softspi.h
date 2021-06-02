/**
******************************************************************************
* @file    bus_spi.h
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   softspi头文件，模拟spi相关函数声明.
******************************************************************************
*/

#include "hardware.h"


void softspi_init(void);

void mosi_input(void);

void mosi_output(void);

void spi_cson(void);

void spi_csoff(void);

void spi_sendbyte(int data);

int spi_recvbyte(void);

void Set_RTC6705_Freq(u32 Freq);

void rtc6705SoftSpiSetFrequency(uint16_t channel_freq);

