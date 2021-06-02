/**
******************************************************************************
* @file    time.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   �̼������ļ���iap��غ���.
******************************************************************************
*/

#include "time.h"


volatile uint32_t sysTickUptime = 0;
volatile uint32_t sysTickValStamp = 0;


/*************************************************************************************
**������Ϣ ��void sysTick_init(void)
**�������� ���δ�ʱ����1us�ж�
**������� ����
**������� ����
**************************************************************************************/
void sysTick_init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1);
    }

    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);//SysTick�ж����ȼ�����
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

