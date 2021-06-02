/**
******************************************************************************
* @file    adc.h
* @author
* @version V0.0.1
* @date    1/06/2020
* @brief   头文件，adc相关函数声明.
******************************************************************************

*/

#include "hardware.h"


#define ADC_REF_VOLTAGE 3.3

#define VOLTAGE_DIVIDER_R1 100000
#define VOLTAGE_DIVIDER_R2 10000

#define ADC_SCALEFACTOR ((float)ADC_REF_VOLTAGE/4096)*((float)VOLTAGE_DIVIDER_R1 + (float)VOLTAGE_DIVIDER_R2)*(1/(float)VOLTAGE_DIVIDER_R2)

void adc_init(void);
float adc_read(int channel);

#define CURR_K (1000/(12000*0.001))
