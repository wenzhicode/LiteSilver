#ifndef __STM32F10X_ALGORITHM_FILTER_H__
#define __STM32F10X_ALGORITHM_FILTER_H__

#include "stdint.h"

#define IMU_SAMPLE_RATE         200.0f

#define IMU_FILTER_CUTOFF_FREQ  30.0f

#define M_PI_F                               3.1415926

typedef struct
{
    uint8_t available;
    float k_1[3];
    float k_2[3];

    float xv_1[3];
    float xv_2[3];
    float out;
} Butter_LP_float;


extern void IIRFilter_Init(void);
float Butter_run(Butter_LP_float *filter, float newdata);

extern Butter_LP_float POS_Acc_Filter[3];
extern Butter_LP_float EF_Acc_Filter;
#endif
