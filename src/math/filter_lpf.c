/**
******************************************************************************
* @file    filter_lpf.c
* @author
* @version V0.0.1
* @date    1/06/2020
* @brief   lpf文件，lpf滤波相关函数.
******************************************************************************

一阶低通滤波

*/

#include "filter_lpf.h"

/**********************************************************************************
**函数信息 ：LowPassFilter_apply(float sample, float cutoff_freq, float dt)
**功能描述 : 一阶低通滤波
**输入参数 ： sample 采样数据  ，cutoff_freq  截止频率 ,dt 采样间隔
**输出参数 ： 滤波后数据
***********************************************************************************/
float LowPassFilter_apply(float sample, float cutoff_freq, float dt)
{
    float alpha = 0.0f, _output;

    if (cutoff_freq <= 0.0f || dt <= 0.0f)
    {
        _output = sample;
        return _output;
    }
    float rc = 1.0f / (M_TWOPI_F * cutoff_freq);
    alpha = CONSTRAINT(dt / (dt + rc), 0.0f, 1.0f);
    _output += (sample - _output) * alpha;
    return _output;
}


