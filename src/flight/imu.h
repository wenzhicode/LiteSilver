/**
******************************************************************************
* @file    imu.h
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   头文件，姿态相关函数声明.
******************************************************************************
*/

#include "hardware.h"




#define ACC_1G 1.0f


#define FASTFILTER 0.05 //onground filter
#define PREFILTER 0.5 //in_air prefilter
#define FILTERTIME 2    //in_air fusion filter

// accel magnitude limits for drift correction
#define ACC_MIN 0.7f
#define ACC_MAX 1.3f



void imu_init(void);

void imu_calc(void);

float calcmagnitude(float vector[3]);

void imu_filter(void);

