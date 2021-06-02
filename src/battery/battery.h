/**
******************************************************************************
* @file    battery.h
* @author
* @version V0.0.1
* @date    14/06/2020
* @brief   头文件，电池相关函数声明.
******************************************************************************
*/

#pragma once

#include "hardware.h"
#include "filter_pt1.h"


typedef struct voltageMeterADCState_s
{
    uint16_t voltageDisplayFiltered;         // battery voltage in 0.01V steps (filtered)
    uint16_t voltageUnfiltered;       // battery voltage in 0.01V steps (unfiltered)
    pt1Filter_t displayFilter;
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    uint16_t voltageSagFiltered;      // battery voltage in 0.01V steps (filtered for vbat sag compensation)
    pt1Filter_t sagFilter;            // filter for vbat sag compensation
#endif
} voltageMeterADCState_t;






