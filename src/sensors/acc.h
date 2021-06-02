/**
******************************************************************************
* @file    acc.h
* @author
* @version V0.0.1
* @date    13/06/2020
* @brief   头文件，acc相关函数声明.
******************************************************************************
*/


#include "hardware.h"





typedef struct accDev_s
{
    float acc_1G_rec;
    int16_t ADCRaw[3];

} accDev_t;


typedef struct acc_s
{
    accDev_t dev;
    uint16_t sampleRateHz;
    float accADC[3];
} acc_t;


typedef struct int16_flightDynamicsTrims_s
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t calibrationCompleted;
} flightDynamicsTrims_def_t;

typedef union flightDynamicsTrims_u
{
    int16_t raw[4];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;


void acc_init(void);

void accUpdate(void);



