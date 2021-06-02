
#include "IIR_filter.h"
#include "math.h"


//°ÍÌØÎÖË¹ÂË²¨Æ÷
uint8_t BUT_IIR_calc_freq_f(float *k, const float sample_freq, const float cutoff_freq, const float cp)
{
    const float Pi = 3.1415926535897932384626433832795f;
    if ((cutoff_freq <= 0.0001f) || (sample_freq <= 2.1f * cutoff_freq))
    {
        // no filtering
        return 0;
    }
    float cos_PI_cp = cosf(Pi * cp);

    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(Pi / fr);
    float ohm2 = ohm * ohm;
    float c = 1.0f + 2.0f * cos_PI_cp * ohm + ohm2;
    float inv_c = 1.0f / c;
    k[0] = ohm2 * inv_c;
    k[1] = 2.0f * (ohm2 - 1.0f) * inv_c;
    k[2] = (1.0f - 2.0f * cos_PI_cp * ohm + ohm2) * inv_c;

    return 1;
}

uint8_t Butter_setCutoffFrequency(Butter_LP_float *filter, float sample_freq, float cutoff_freq)
{
    filter->available = BUT_IIR_calc_freq_f(filter->k_1, sample_freq, cutoff_freq, 1.0f / 8);
    BUT_IIR_calc_freq_f(filter->k_2, sample_freq, cutoff_freq, 3.0f / 8);
    return filter->available;
}

void Butter_init(Butter_LP_float *filter, float sample_freq, float cutoff_freq)
{
    filter->xv_1[0] = filter->xv_1[1] = filter->xv_1[2] = 0;
    filter->xv_2[0] = filter->xv_2[1] = filter->xv_2[2] = 0;
    filter->out = 0;

    Butter_setCutoffFrequency(filter, sample_freq, cutoff_freq);
}


float Butter_run(Butter_LP_float *filter, float newdata)
{
    if (filter->available)
    {
        filter->xv_1[2] = filter->xv_1[1];
        filter->xv_1[1] = filter->xv_1[0];
        filter->xv_1[0] = newdata * filter->k_1[0] - filter->k_1[1] * filter->xv_1[1] - filter->k_1[2] * filter->xv_1[2];
        float out1 = filter->xv_1[0] + 2.0f * filter->xv_1[1] + filter->xv_1[2];

        filter->xv_2[2] = filter->xv_2[1];
        filter->xv_2[1] = filter->xv_2[0];
        filter->xv_2[0] = out1 * filter->k_2[0] - filter->k_2[1] * filter->xv_2[1] - filter->k_2[2] * filter->xv_2[2];
        filter->out = filter->xv_2[0] + 2.0f * filter->xv_2[1] + filter->xv_2[2];
    }
    else
        filter->out = newdata;
    return filter->out;
}

Butter_LP_float POS_Acc_Filter[3];
Butter_LP_float EF_Acc_Filter;

void IIRFilter_Init(void)
{
    Butter_init(&POS_Acc_Filter[0], 1000, 30);
    Butter_init(&POS_Acc_Filter[1], 1000, 30);
    Butter_init(&POS_Acc_Filter[2], 1000, 30);

    Butter_init(&EF_Acc_Filter, 1000, 20);
}
