/**
******************************************************************************
* @file     failsafe.c
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief    failsafe文件， failsafe相关函数.
******************************************************************************
*/


#include "failsafe.h"
#include "rgb_led.h"
#include "dshot.h"
#include "time.h"






/********************************************************************
**函数信息 ：failloop(int val)
**功能描述 ：硬件错误或者软件错误将进入此循环
**输入参数 ：val 错误号
**输出参数 ：无

 2 - low battery at powerup - if enabled by config
 3 - radio chip not detected
 4 - Gyro not found
 5 - clock , intterrupts , systick
 6 - loop time issue
 7 - i2c error
 8 - i2c error main loop
 9 - baro not found
********************************************************************/
void failloop(int val)
{
    for (int i = 0 ; i <= 3 ; i++)
    {
        pwm_set(i, 0);
    }

    while (1)
    {
        for (int i = 0 ; i < val; i++)
        {
            rgb_send(65280);
            delay(200000);
            rgb_send(0);
            delay(200000);
        }
        delay(800000);
    }
}



