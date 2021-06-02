/**
******************************************************************************
* @file    bus_i2c.h
* @author  wz
* @version V0.0.1
* @date    25/05/2020
* @brief   i2c头文件，i2c硬件相关函数声明.
******************************************************************************
*/

#include "hardware.h"


void i2c_init(I2C_TypeDef *I2Cx, unsigned int uiI2C_speed);
void I2CTXEmptyCheck(I2C_TypeDef *I2Cx);
void I2CRXFullCheck(I2C_TypeDef *I2Cx);
void I2CTXByte(I2C_TypeDef *I2Cx, unsigned char temp);
unsigned char I2CRXByte(I2C_TypeDef *I2Cx);
void I2CRXGroup(I2C_TypeDef *I2Cx, u16 rx_count, u8 *data_buf);
void I2CMasterWrite(I2C_TypeDef *I2Cx, unsigned short mem_byte_addr, unsigned short tx_count, unsigned char *tx_data);
void I2CMasterRead(I2C_TypeDef *I2Cx, unsigned short mem_byte_addr, unsigned short rx_count, unsigned char *rx_data);

void I2CSetDeviceAddr(I2C_TypeDef *I2Cx, unsigned char deviceaddr);

