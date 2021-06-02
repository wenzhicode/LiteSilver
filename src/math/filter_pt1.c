/**
******************************************************************************
* @file    filter_pt1.c
* @author
* @version V0.0.1
* @date    13/06/2020
* @brief   pt1文件，pt1滤波相关函数.
******************************************************************************

pt1低通滤波

*/

#include "filter_pt1.h"



float pt1FilterGain(float f_cut, float dT)
{
    float RC = 1 / (2 * M_PI_FLOAT * f_cut);
    return dT / (RC + dT);
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    filter->k = k;
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}




