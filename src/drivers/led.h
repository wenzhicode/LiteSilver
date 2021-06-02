/**
******************************************************************************
* @file    led.h
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   led头文件，led相关函数声明.
******************************************************************************
*/


#include "hardware.h"
#include "targets.h"




void ledflash(uint32_t period, int duty);


void ledon(uint8_t val);

void ledoff(uint8_t val);

void led_process(void);

