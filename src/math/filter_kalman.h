/**
******************************************************************************
* @file    filter_kalman.h
* @author
* @version V0.0.1
* @date    1/06/2020
* @brief   ͷ�ļ���kalman�˲���غ�������.
******************************************************************************

*/

#include "MM32F103.h"
#include "SYSTEM_MM32F103.h"


#define KALMAN_Q 0.02

#define KALMAN_R 7.0000


typedef struct
{
    float x_est_last ;
    float P_last ;
    float Q;
    float R;
} kalmanFilter;


float KalmanFilter(const float ResrcData, float ProcessNiose_Q, float MeasureNoise_R);

void kalman_init(void);

float lpffilter(float in, int num);
float lpffilter2(float in, int num);


