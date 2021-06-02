/**
******************************************************************************
* @file    filter_sp.h
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   头文件，sp滤波相关函数声明.
******************************************************************************
*/

#include "hardware.h"


typedef struct
{

    float v[2];

} spFilter;

void splpf_init(void);

float splpf(float in, int num);


