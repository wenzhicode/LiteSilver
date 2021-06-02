/**
******************************************************************************
* @file    time.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   固件更新文件，iap相关函数.
******************************************************************************
*/

#include "time.h"


volatile uint32_t sysTickUptime = 0;
volatile uint32_t sysTickValStamp = 0;


/*************************************************************************************
**函数信息 ：void sysTick_init(void)
**功能描述 ：滴答定时器，1us中断
**输入参数 ：无
**输出参数 ：无
**************************************************************************************/
void sysTick_init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1);
    }

    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);//SysTick中断优先级设置
}


void SysTick_Handler(void)
{
    sysTickUptime++;
    sysTickValStamp = SysTick->VAL;
}


uint32_t millis(void)
{
    return sysTickUptime;
}


uint32_t gettime(void)
{
    register uint32_t ms, cycle_cnt;

    do
    {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    }
    while (ms != sysTickUptime || cycle_cnt > sysTickValStamp);

    return (ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD);

}


void delay_us(uint32_t us)
{
    uint64_t now = gettime();
    while ((gettime() - now) < us);
}


void delay_ms(uint32_t ms)
{
    delay_us(ms * 1000);
}

void delay(uint32_t data)
{
    volatile uint32_t count;
    count = data * 7;
    while (count--);
}

