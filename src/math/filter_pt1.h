/**
******************************************************************************
* @file    filter_pt1.h
* @author
* @version V0.0.1
* @date    13/06/2020
* @brief   头文件，pt1滤波相关函数声明.
******************************************************************************

*/

#pragma once

#include "MM32F103.h"
#include "SYSTEM_MM32F103.h"


#define M_PI_FLOAT  3.14159265f
#define M_LN2_FLOAT 0.69314718f

typedef struct pt1Filter_s
{
    float state;
    float k;
} pt1Filter_t;


float pt1FilterApply(pt1Filter_t *filter, float input);

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);

void pt1FilterInit(pt1Filter_t *filter, float k);

float pt1FilterGain(float f_cut, float dT);



