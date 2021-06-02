/**
******************************************************************************
* @file    filter_biquad.h
* @author
* @version V0.0.1
* @date    13/06/2020
* @brief   头文件，biquad滤波相关函数声明.
******************************************************************************

*/


#include "MM32F103.h"
#include "SYSTEM_MM32F103.h"
#include "math.h"




#define M_PI_FLOAT  3.14159265f
#define M_LN2_FLOAT 0.69314718f
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/


/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s
{
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

typedef enum
{
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;


void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);


void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);

void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);


void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);

float biquadFilterApply(biquadFilter_t *filter, float input);
