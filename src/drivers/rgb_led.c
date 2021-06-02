/**
******************************************************************************
* @file    dshot.c
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   驱动文件，rgb硬件相关函数.
******************************************************************************
*/


#include "defines.h"
#include "time.h"
#include "util.h"
#include <math.h>
#include "rgb_led.h"

// normal flight rgb colour - LED switch ON
#define RGB_VALUE_INFLIGHT_ON RGB( 0 , 0 , 255 )

// normal flight rgb colour - LED switch OFF
#define RGB_VALUE_INFLIGHT_OFF RGB( 0 , 0 , 0 )

//  colour before bind
#define RGB_VALUE_BEFORE_BIND RGB( 0 , 128 , 128 )


#define RGB_FILTER_TIME FILTERCALC( 1000*DOWNSAMPLE , RGB_FILTER_TIME_MICROSECONDS)
#define RGB( r , g , b ) ( ( ((int)g&0xff)<<16)|( ((int)r&0xff)<<8)|( (int)b&0xff ))

extern unsigned char rx_select;
extern unsigned char low_vol;
extern int rxmode;
extern int failsafe;
extern int ledcommand;
extern unsigned long ledcommandtime;
extern char aux[];
static int light=0;
uint8_t rgb_led=0;

int led_num=0;
int rx_num=0;

#define RGBHIGH gpioset( RGB_PORT, RGB_PIN)
#define RGBLOW gpioreset( RGB_PORT, RGB_PIN);

#pragma push

#pragma Otime
#pragma O2

void delay1a()
{
    uint8_t count = 4;
    while (count--);
}

void delay1b()
{
    uint8_t count = 4;
    while (count--);
}

void delay2a()
{
    uint8_t count = 2;
    while (count--);
}

void delay2b()
{
    uint8_t count = 6;
    while (count--);
}

void rgb_send( int data)
{
    for ( int i =23 ; i >=0 ; i--)
    {
        if (  (data>>i)&1  )
        {
            RGBHIGH;
            delay1a();
            RGBLOW;
            delay1b();
        }
        else
        {
            RGBHIGH;
            delay2a();
            RGBLOW;
            delay2b();
        }
    }
}
#pragma pop
float kr_position = 0;
int kr_dir = 0;

int rgb_led_value[RGB_LED_NUMBER];
#define DOWNSAMPLE 16

// speed of movement
float KR_SPEED = 0.005f * DOWNSAMPLE;



void rgb_led_set_one( int led_number, int rgb )
{
    rgb_led_value[led_number] = rgb;
}

// knight rider style led movement
void rgb_knight_rider( void)
{
    if ( kr_dir )
    {
        kr_position+= KR_SPEED;
        if ( kr_position > RGB_LED_NUMBER - 1 )
            kr_dir =!kr_dir;
    }
    else
    {
        kr_position-= KR_SPEED;
        if ( kr_position < 0 )
            kr_dir =!kr_dir;
    }

// calculate led value
    for ( int i = 0 ; i < RGB_LED_NUMBER ; i++)
    {
        float led_bright = fabsf( (float) i - kr_position);
        if ( led_bright > 1.0f) led_bright = 1.0f;
        led_bright = 1.0f - led_bright;

        // set a green background as well, 32 brightness
        rgb_led_set_one( i, RGB( (led_bright*255.0f), (32.0f-led_bright*32.0f), 0) );

    }

}

void rgb_ledflash(int color1, int color2, uint32_t period, int duty)
{
    if ( gettime() % period > (period*duty)>>4 )
    {
        rgb_send( color1 );
    }
    else
    {
        rgb_send( color2 );
    }

}



void rgb_led_lvc(void)
{
    /*
    上电时电压过低( < 3.3v)，初始不通过：红色 red 闪两次  RGB( 255 , 0 , 0 )

    1初始话完成，飞行过程中电压低(上电时 >3.3v, 飞行过程中 < 3.3v) :红色 red 慢闪 RGB( 255 , 0 , 0 )

    2初始化完成，还没有遥控信息，或者重新进入对频：绿色 green 慢闪烁 RGB( 0,160,0 )
        D16 FCC:
        D16 LBT:
        D8:
        SBUS:
        BAYANG:

    3初始化完成，有遥控信息，但没有信号， 或者飞行过程中信号丢失： 橙色 orange 慢闪  RGB(255,165,0)

    4初始化完成，可以正常飞行状态：设置0：蓝色 blue 常亮 RGB(0,127,255)；设置1：慢闪

    5解锁时，油门不在最低处 ：白色 white RGB(255,255,255) 慢闪

    优先级：

    */

    if(low_vol)
    {
        rgb_ledflash(RGB( 255, 0, 0 ), RGB( 0, 0, 0 ),500000, 8);
    }
    else
    {
        if(failsafe)
        {
            switch(ledcommand)
            {
            case 2:
                switch(rx_select)
                {
                case 0:
                    if(rx_num<20)
                    {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 80) {
                        rgb_send(0);
                        rx_num++;
                    }
                    else
                    {
                        rx_num = 0;
                    }
                    break;

                case 1:
                    if(rx_num<20)
                    {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 30) {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 40) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 100)
                    {
                        rx_num++;
                        rgb_send(0);
                    }
                    else
                    {
                        rx_num = 0;
                    }
                    break;

                case 2:
                    if(rx_num<20)
                    {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 30) {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 50) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 60)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 80) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 140)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else
                    {
                        rx_num = 0;
                    }
                    break;

                case 3:
                    if(rx_num<20)
                    {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 30) {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 50) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 60)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 80) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 90)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 110)
                    {
                        rgb_send( RGB( 0,160,0 ));
                        rx_num++;
                    }
                    else if(rx_num < 170)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else
                    {
                        rx_num = 0;
                    }
                    break;

                case 4:
                    if(rx_num<20)
                    {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 30) {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 50) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 60)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 80) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 90)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 110)
                    {
                        rgb_send( RGB( 0,160,0 ));
                        rx_num++;
                    }
                    else if(rx_num < 120)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 140)
                    {
                        rgb_send( RGB( 0,160,0 ));
                        rx_num++;
                    }
                    else if(rx_num < 200)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else
                    {
                        rx_num = 0;
                    }
                    break;
                case 5:
                    if(rx_num<20)
                    {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 30) {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 50) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 60)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 80) {
                        rx_num++;
                        rgb_send( RGB( 0,160,0 ));
                    }
                    else if(rx_num < 90)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 110)
                    {
                        rgb_send( RGB( 0,160,0 ));
                        rx_num++;
                    }
                    else if(rx_num < 120)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 140)
                    {
                        rgb_send( RGB( 0,160,0 ));
                        rx_num++;
                    }
                    else if(rx_num < 150)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else if(rx_num < 170)
                    {
                        rgb_send( RGB( 0,160,0 ));
                        rx_num++;
                    }
                    else if(rx_num < 230)
                    {
                        rgb_send(0);
                        rx_num++;
                    }
                    else
                    {
                        rx_num = 0;
                    }
                    break;
                }
                break;

            case 3:
                rgb_ledflash(RGB(255,165,0), RGB( 0, 0, 0 ),500000, 8);
                break;
            }
        }
        else
        {
            if(ledcommand == 4)
            {
                if(rgb_led)
                {
                    led_num ++;

                    if(led_num <10)
                    {
                        rgb_send(RGB( 255, 0, 0 ));
                    }
                    else if(led_num <20)
                    {
                        rgb_send(RGB( 0,160,0 ));
                    }
                    else if(led_num < 30)
                    {
                        rgb_send(RGB(255,165,0));
                    }
                    else if(led_num<40)
                    {
                        rgb_send(RGB(0,127,255));
                    }
                    else if(led_num<50)
                    {
                        led_num =0;
                    }
                }
                else
                {
                    rgb_send(RGB(0,127,255));
                }
            }
            else if(ledcommand ==5)
            {
                rgb_ledflash(RGB(255,255,255), RGB( 0, 0, 0 ),500000, 8);
            }
        }
    }
}



void RGBInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = RGB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RGB_PORT, &GPIO_InitStructure);
    GPIO_SetBits(RGB_PORT,RGB_PIN);
    rgb_send(255);
    delay_ms(100);
    rgb_send(65535);
    delay_ms(100);
    rgb_send(65280);
    delay_ms(100);
    rgb_send(255);
    delay_ms(100);
    rgb_send(65535);
    delay_ms(100);
    rgb_send(65280);
}






