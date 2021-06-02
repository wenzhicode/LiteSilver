/**
******************************************************************************
* @file    bus_spi.h
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   spi头文件，spi硬件相关函数声明.
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

