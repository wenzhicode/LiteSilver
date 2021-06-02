/**
******************************************************************************
* @file    filter_iir.h
* @author
* @version V0.0.1
* @date    1/06/2020
* @brief   头文件，iir滤波相关函数声明.
******************************************************************************

*/

#include "MM32F103.h"
#include "SYSTEM_MM32F103.h"

typedef struct
{
    float           fc;
    float           a1;
    float           a2;
    float           b0;
    float           b1;
    float           b2;
    float           y_1;
    float           y_2;
} IIR_coeff_Typedef;

#define M_PI_F          (3.14159265358f)

float get_iir_output(IIR_coeff_Typedef *coeff, float sample);

void cal_iir_coeff(IIR_coeff_Typedef *coeff, float fs, float fc);

