/**
******************************************************************************
* @file    main.c
* @author  wz
* @version V0.0.1
* @date    25/05/2020
* @brief   程序入口文件.
******************************************************************************
MCU:MM32F103KBU6
k:32pins
B:128k
U:QFN
6: -40 - +85°C

时钟：96MHz

两组串口：串口2用于osd，串口1暂定

一组I2C：mpu6050,气压计，最高400kHz

一组SPI：接收机，四线

定时器：3个16bit通用寄存器TIME2\3\4，一个高级TIME1

一组USB：2.0标准，全速12M模式

两组10通道ADC：

*/



#include "MM32F103.h"
#include "config.h"
#include "serial_uart.h"
#include "stdio.h"
#include "hardware.h"
#include "bus_i2c.h"
#include "mpu6500.h"
#include "serial_usb.h"
#include "usb.h"
#include "time.h"
#include "dshot.h"
#include "defines.h"
#include "adc.h"
#include "spl06001.h"
#include "imu.h"
#include "filter_sp.h"
#include "filter_kalman.h"
#include "mavlink.h"
#include "maths.h"
#include "blackbox.h"
#include "control.h"
#include "arm_const_structs.h"
#include "arm_math.h"
#include "msp.h"
#include "misc_gpio.h"
#include "scheduler.h"
#include "task.h"
#include "bus_spi.h"
#include "gyro.h"
#include "acc.h"
#include "rx_serial_dsm.h"
#include "rx_spi.h"
#include "util.h"
#include "rgb_led.h"
#include "IIR_filter.h"
#include "osd.h"
#include "rx_serial_sbus.h"
#include "bus_softspi.h"
#include "flash.h"
#include "led.h"
#include "pid.h"
#include "rx_bayang.h"
#include "failsafe.h"


#define YEAR ((((__DATE__ [7] - '0') * 10 + (__DATE__ [8] - '0')) * 10 + (__DATE__ [9] - '0')) * 10 + (__DATE__ [10] - '0'))  
  
#define MONTH (__DATE__ [2] == 'n' ? 0 \  
    : __DATE__ [2] == 'b' ? 1 \  
    : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 2 : 3) \  
    : __DATE__ [2] == 'y' ? 4 \  
    : __DATE__ [2] == 'n' ? 5 \  
    : __DATE__ [2] == 'l' ? 6 \  
    : __DATE__ [2] == 'g' ? 7 \  
    : __DATE__ [2] == 'p' ? 8 \  
    : __DATE__ [2] == 't' ? 9 \  
    : __DATE__ [2] == 'v' ? 10 : 11)  
  
#define DAY ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10 + (__DATE__ [5] - '0'))  
  
#define MDK_HOUR  ((__TIME__[0]-'0')*10 + __TIME__[1]-'0')
#define MDK_MIN   ((__TIME__[3]-'0')*10 + __TIME__[4]-'0')
#define MDK_SEC   ((__TIME__[6]-'0')*10 + __TIME__[7]-'0')


unsigned char pid_noinit=0;
unsigned char lipo_last=0;
unsigned char sbus_dsm = 0;
extern float vbattfilt;
extern float vreffilt;
extern float lipo_cell_count;
extern unsigned char rx_select;
extern unsigned char rx_reboot;

extern bool frSkySpiInit(rx_spi_protocol_e spiProtocol);


uint16_t year,month,day;
uint8_t mdk_hour,mdk_min,mdk_sec;



uint32_t  init_time=0;

int main(void)
{
    //中断向量表重定义
    SCB->VTOR = FLASH_BASE | 0x3000;

    delay(1000);

    //滴答定时器
    sysTick_init();
    delay_ms(10);

    //MISC gpio 的初始化
    misc_gpioInit();

    delay_ms(10);

    //USB，虚拟串口vcp
    usb_GpioInit();
    USB_Init();

    flash_load();

    if(rx_select ==  4)
    {
        //dsm 进入绑定时间点需要在上电的20ms - 140ms间
        dsm_bind();
    }

    /*配置ADC1循环转换模式*/
    adc_init();

    RGBInit();

    //串口2，开启DMA
    uart2_initwBaudRate(115200);

    osd_init();

    int count = 0;

    while (count < 5000)
    {
        float bootadc = adc_read(0) * vreffilt;
        lpf(&vreffilt, adc_read(1), 0.9968f);
        lpf(&vbattfilt, bootadc, 0.9968f);
        count++;
    }
    for (int i = 6 ; i > 0 ; i--)
    {
        float cells = i;
        if (vbattfilt / cells > 3.3f)
        {
            lipo_cell_count = cells;
            break;
        }
    }

//    if (vbattfilt / lipo_cell_count < 3.3f)
//    {
//        failloop(2);
//    }

    if(lipo_cell_count != lipo_last)
    {
        lipo_last = lipo_cell_count;
        pid_noinit = 1;
    }

    delay_ms(100);

    //dshot
    dshot_init();
    for (u8 i = 0; i < 4; i++)
    {
        pwm_set(i, 0);
    }
    delay_ms(100);

    //SPI
    spi_init();

    delay_ms(10);
    softspi_init();

    mpu6500_init();


    switch (rx_select)
    {
    case 0:
        frSkySpiInit(1);
        rx_reboot = 0;
        break;

    case 1:
        frSkySpiInit(3);
        rx_reboot = 0;
        break;

    case 2:
        frSkySpiInit(0);
        rx_reboot = 0;
        break;

    case 3:
        sbus_dsm = 1;
        sbus_init();
        rx_reboot = 2;
        break;

    case 4:
        sbus_dsm = 0;
        dsm_init();
        rx_reboot = 3;
        break;

    case 5:
        rx_init();
        rx_reboot=4;
        break;
    }

    //初始化姿态
    gyro_cal();
    imu_init();

    pid_init();


    IIRFilter_Init();

    tasksInit();

    init_time = millis();
    
    year = YEAR;
    month = MONTH + 1;
    day = DAY;
    mdk_hour = MDK_HOUR;
    mdk_min = MDK_MIN;
    mdk_sec = MDK_SEC;
      

    while (1)
    {
        scheduler();
    }
}




