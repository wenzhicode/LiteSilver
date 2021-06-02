/**
******************************************************************************
* @file    filter_lpf.c
* @author
* @version V0.0.1
* @date    1/06/2020
* @brief   lpf�ļ���lpf�˲���غ���.
******************************************************************************

һ�׵�ͨ�˲�

*/

#include "filter_lpf.h"

/**********************************************************************************
**������Ϣ ��LowPassFilter_apply(float sample, float cutoff_freq, float dt)
**�������� : һ�׵�ͨ�˲�
**������� �� sample ��������  ��cutoff_freq  ��ֹƵ�� ,dt �������
**������� �� �˲�������
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


