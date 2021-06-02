/**
******************************************************************************
* @file    hardware.h
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   math文件，滤波系数计算等函数.
******************************************************************************
*/

#include <math.h>
#include "util.h"
#include <inttypes.h>

// calculates the coefficient for lpf filter, times in the same units
float lpfcalc(float sampleperiod, float filtertime)
{
    float ga = 1.0f - sampleperiod / filtertime;
    if (ga > 1.0f)
        ga = 1.0f;
    if (ga < 0.0f)
        ga = 0.0f;
    return ga;
}


// calculates the coefficient for lpf filter
float lpfcalc_hz(float sampleperiod, float filterhz)
{
    float ga = 1.0f - sampleperiod * filterhz;
    if (ga > 1.0f)
        ga = 1.0f;
    if (ga < 0.0f)
        ga = 0.0f;
    return ga;
}
void hpf(float *out, float delta_in, float coeff)
{
    *out = (*out + delta_in) * coeff;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{

    return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;

}

void lpf(float *out, float in, float coeff)
{
    *out = (*out) * coeff + in * (1 - coeff);
}

void limitf(float *input, const float limit)
{
    if (*input > limit) *input = limit;
    if (*input < - limit) *input = - limit;
}

/***************************************************************
**函数信息 ：rcexpo ( float in , float exp )
**功能描述 ：摇杆曲线
**输入参数 ：in摇杆值   exp 曲线率
**输出参数 ：变换后的摇杆值

弱化低摇杆值响应，增强高摇杆值响应
y = a* x^3 + (1-a)*x

****************************************************************/
float rcexpo(float in, float exp)
{
    float ans;
    if (exp > 1) exp = 1;
    if (exp < -1) exp = -1;
    ans = in * in * in * exp + in * (1 - exp);
    limitf(&ans, 1.0);
    return ans;
}

void constrain(float *out, float min, float max)
{
    if (*out < min) *out = min;
    else if (*out > max) *out = max;
}

float fastsin(float x)
{
    float sin1;
//always wrap input angle to -PI..PI
    while (x < -3.14159265f)
        x += 6.28318531f;

    while (x >  3.14159265f)
        x -= 6.28318531f;


//compute sine
    if (x < 0)
        sin1 = (1.27323954f + .405284735f * x) * x;
    else
        sin1 = (1.27323954f - .405284735f * x) * x;

    return sin1;

}


float fastcos(float x)
{
    x += 1.57079632f;
    return fastsin(x);
}

