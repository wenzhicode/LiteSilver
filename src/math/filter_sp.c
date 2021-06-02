/**
******************************************************************************
* @file    filter_sp.c
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   滤波文件，sp滤波相关函数.
******************************************************************************
*/


#include "filter_sp.h"

spFilter spfilter[3];

void splpf_init(void)
{
    spfilter[0].v[0] = 0.0;
    spfilter[1].v[0] = 0.0;
    spfilter[2].v[0] = 0.0;

}


float splpf(float in, int num)
{

    spfilter[num].v[0] = spfilter[0].v[1];
    spfilter[num].v[1] = (6.749703162983405891e-2f * in)
                         + (0.86500593674033188218f * spfilter[num].v[0]);
    return
        (spfilter[num].v[0] + spfilter[num].v[1]);

}



