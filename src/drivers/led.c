/**
******************************************************************************
* @file    led.c
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   驱动文件，led硬件相关函数.
******************************************************************************
*/


#include "led.h"
#include "time.h"
#include "util.h"
#include "defines.h"

#define LEDALL 15
#define LED_BRIGHTNESS 15

int ledlevel2 = 0;
unsigned long lastledtime;
float lastledbrightness = 0;

void ledoff(uint8_t val)
{
    if (val & 1)  GPIO_SetBits(GPIOB, GPIO_Pin_2);
}

void ledon(uint8_t val)
{
    if (val & 1)  GPIO_ResetBits(GPIOB, GPIO_Pin_2);
}




void ledflash(uint32_t period, int duty)
{
    if (gettime() % period > (period * duty) >> 4)
    {
        ledon(LEDALL);
    }
    else
    {
        ledoff(LEDALL);
    }
}

uint8_t led_pwm(uint8_t pwmval)
{
    static float ds_integrator = 0;
    unsigned int time = gettime();
    unsigned int ledtime = time - lastledtime;

    lastledtime = time;

    float desiredbrightness = pwmval * (1.0f / 15.0f);

    limitf(&ds_integrator, 2);

    ds_integrator += (desiredbrightness - lastledbrightness) * ledtime * (1.0f  / (float) LOOPTIME);

    if (ds_integrator > 0.49f)
    {
        ledon(255);
        lastledbrightness = 1.0f;
    }
    else
    {
        ledoff(255);
        lastledbrightness = 0;
    }
    return 0;
}

extern int lowbatt;
extern int rxmode;
extern int failsafe;
int ledcommand = 3;
int ledblink = 0;
unsigned long ledcommandtime = 0;

void led_process(void)
{
    if (lowbatt)
        ledflash(500000, 8);
    else
    {
        if (rxmode == RXMODE_BIND)
        {
            // bind mode
            ledflash(100000, 12);
        }
        else
        {
            // non bind
            if (failsafe)
            {
                ledflash(500000, 15);
            }
            else
            {
                int leds_on = 1;
                if (ledcommand)
                {
                    if (!ledcommandtime)
                        ledcommandtime = gettime();
                    if (gettime() - ledcommandtime > 500000)
                    {
                        ledcommand = 0;
                        ledcommandtime = 0;
                    }
                    ledflash(100000, 8);
                }
                else if (ledblink)
                {
                    unsigned long time = gettime();
                    if (!ledcommandtime)
                    {
                        ledcommandtime = time;
                        if (leds_on) ledoff(255);
                        else ledon(255);
                    }
                    if (time - ledcommandtime > 500000)
                    {
                        ledblink--;
                        ledcommandtime = 0;
                    }
                    if (time - ledcommandtime > 300000)
                    {
                        if (leds_on) ledon(255);
                        else  ledoff(255);
                    }
                }
                else if (leds_on)
                {
                    if (LED_BRIGHTNESS != 15)
                        led_pwm(LED_BRIGHTNESS);
                    else ledon(255);
                }
                else ledoff(255);
            }
        }
    }
}

