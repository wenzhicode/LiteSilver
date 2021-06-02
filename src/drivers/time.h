/**
******************************************************************************
* @file    time.h
* @author  wz
* @version V0.0.1
* @date    25/05/2020
* @brief   time头文件，time相关函数声明.
******************************************************************************
*/

#pragma once

#include "hardware.h"



#define TICK_PER_SECOND 1000
#define TICK_US         1000


static inline int32_t cmpTimeUs(u32 a, u32 b)
{
    return (int32_t)(a - b);
}

uint32_t gettime(void);

void sysTick_init(void);

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

uint32_t millis(void);

void delay(uint32_t data);




