/**
******************************************************************************
* @file    angle_pid.c
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   angle_pid文件，angle_pid相关函数.
******************************************************************************

外环PID

*/


#include "angle_pid.h"
#include "math.h"
#include "util.h"


#define OUTLIMIT_FLOAT (apidkp1[0]+apidkp2[0])   //set angle pid output limit to sum of both P terms just in case

float apidoutput1[3];
float apidoutput2[3];
float angleerror[3];
float lasterror[3];
float apidoutput[3];


// Leveling algorithm coefficients for small errors  (normal flying)
float apidkp1[3] = { 10.00 };  // P TERM GAIN ROLL + PITCH
float apidkd1[3] = { 3.0 };    // D TERM GAIN ROLL + PITCH

// Leveling algorithm coefficients for large errors  (stick banging or collisions)
float apidkp2[3] = { 5.00 };   // P TERM GAIN ROLL + PITCH
float apidkd2[3] = { 0.0 };    // D TERM GAIN ROLL + PITCH


float apid(int x)
{

    // P term 1 weighted
    apidoutput1[x] = (1 - fabsf(angleerror[x])) * angleerror[x] * apidkp1[0] ;

    // P term 2 weighted
    apidoutput2[x] = fabsf(angleerror[x]) * angleerror[x] * apidkp2[0];

    extern float timefactor;
    // D term 1 weighted + P term 1 weighted
    apidoutput1[x] += (angleerror[x] - lasterror[x]) * apidkd1[0] * (1 - fabsf(angleerror[x])) * timefactor;

    // D term 2 weighted + P term 2 weighted
    apidoutput2[x] += ((angleerror[x] - lasterror[x]) * apidkd2[0] * fabsf(angleerror[x]) * timefactor);

    // apidoutput sum
    apidoutput[x] = apidoutput1[x] + apidoutput2[x];

    lasterror[x] = angleerror[x];
    limitf(&apidoutput[x], OUTLIMIT_FLOAT);

    return apidoutput[x];
}







