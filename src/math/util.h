/**
******************************************************************************
* @file    hardware.h
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   math文件，滤波系数计算等函数.
******************************************************************************
*/

void constrain(float *out, float min, float max);
float lpfcalc(float sampleperiod, float filtertime);
float lpfcalc_hz(float sampleperiod, float filterhz);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void lpf(float *out, float in, float coeff);
void hpf(float *out, float in, float coeff);
float rcexpo(float x, float exp);

void limitf(float *input, const float limit);

float fastsin(float x);
float fastcos(float x);


void limit180(float *);


