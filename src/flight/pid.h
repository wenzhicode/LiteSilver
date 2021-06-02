/**
******************************************************************************
* @file    pid.h
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   头文件，pid相关函数声明.
******************************************************************************
*/

#include "hardware.h"




#define PID_VC_FACTOR 1.33f


void rotateErrors(void);

float pid(int x);

float lpf2(float in, int num);

void pid_precalc(void);

void pid_init(void);
