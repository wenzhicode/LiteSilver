/**
******************************************************************************
* @file    bus_spi.h
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   spiͷ�ļ���spiӲ����غ�������.
******************************************************************************
*/

#include "hardware.h"

void spi_init(void);


void SPIM_CSLow(void);

void SPIM_CSHigh(void);

void SPIM_TXEn(SPI_TypeDef *SPIx);

void SPIM_TXDisable(SPI_TypeDef *SPIx);

void SPIM_RXEn(SPI_TypeDef *SPIx);

void SPIM_RXDisable(SPI_TypeDef *SPIx);

unsigned int SPI_RW(unsigned char tx_data);

