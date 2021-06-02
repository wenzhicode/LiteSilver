/**
******************************************************************************
* @file    filter_lpf.h
* @author
* @version V0.0.1
* @date    1/06/2020
* @brief   头文件，lpf滤波相关函数声明.
******************************************************************************

*/


#include "MM32F103.h"
#include "SYSTEM_MM32F103.h"


float LowPassFilter_apply(float sample, float cutoff_freq, float dt);

#define M_PI_F          (3.14159265358f)
#define M_TWOPI_F       (M_PI_F * 2.0f)

#define CONSTRAINT(in, min, max)  (in > max ? max : (in < min ? min : in))
