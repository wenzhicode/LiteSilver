/**
******************************************************************************
* @file    filter_sp.h
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   ͷ�ļ���sp�˲���غ�������.
******************************************************************************
*/

#include "hardware.h"


typedef struct
{

    float v[2];

} spFilter;

void splpf_init(void);

float splpf(float in, int num);


