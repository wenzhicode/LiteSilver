/**
******************************************************************************
* @file    flash.h
* @author
* @version V0.0.1
* @date    30/06/2020
* @brief   头文件，flash相关函数声明.
******************************************************************************
*/



#include "hardware.h"


#define FMC_HEADER 0x12AA0001


void flash_load(void);

void flash_save(void);



