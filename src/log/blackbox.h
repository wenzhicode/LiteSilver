/**
******************************************************************************
* @file    blackbox.h
* @author
* @version V0.0.1
* @date    4/06/2020
* @brief   ͷ�ļ���blackbox��غ�������.
******************************************************************************
*/


#include "hardware.h"




typedef struct blackboxMainState_s
{
    uint32_t time;

    int16_t gyroADC[3];
    int16_t accADC[3];

} blackboxMainState_t;



void blackboxUpdate(void);


void blackboxStart(void);
