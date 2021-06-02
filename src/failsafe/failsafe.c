/**
******************************************************************************
* @file     failsafe.c
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief    failsafe�ļ��� failsafe��غ���.
******************************************************************************
*/


#include "failsafe.h"
#include "rgb_led.h"
#include "dshot.h"
#include "time.h"






/********************************************************************
**������Ϣ ��failloop(int val)
**�������� ��Ӳ���������������󽫽����ѭ��
**������� ��val �����
**������� ����

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



