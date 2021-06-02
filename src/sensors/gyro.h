/**
******************************************************************************
* @file    gyro.h
* @author
* @version V0.0.1
* @date    13/06/2020
* @brief   头文件，gyro相关函数声明.
******************************************************************************
*/


#include "hardware.h"
#include "filter_pt1.h"
#include "filter_biquad.h"


typedef struct stdev_s
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;


typedef struct gyroDev_s
{
    // scalefactor
    float gyroZero[3];
    float gyroADC[3];                           // gyro data after calibration and alignment
    int32_t gyroADCRawPrevious[3];
    int16_t gyroADCRaw[3];                      // raw data from sensor
} gyroDev_t;

typedef struct gyroCalibration_s
{
    float sum[3];
    stdev_t var[3];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct gyroSensor_s
{
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;
    float gyroADC[3];
    float sampleSum[3];

    pt1Filter_t lowpassFilter[3];
    pt1Filter_t lowpass2Filter[3];


} gyroSensor_t;



void gyro_init(void);

void gyroUpdateSensor(gyroSensor_t *gyroSensor);
void gyro_filter(void);








