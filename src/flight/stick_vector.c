/**
******************************************************************************
* @file    stick_vector.c
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   stick_vector文件，stick_vector相关函数.
******************************************************************************

计算估计出的姿态角跟摇杆映射的期望角度的误差值

*/


#include "stick_vector.h"
#include "defines.h"
#include "util.h"
#include "maths.h"
#include <math.h>
#include <string.h>


float errorvect[3];
// cache the last result so it does not get calculated everytime
float last_rx[2] = {13.13f, 12.12f};
float stickvector[3] = { 0, 0, 1};



extern float GEstG[3];
extern char aux[];

#define max_angle LEVEL_MAX_ANGLE


void stick_vector(float rx_input[], float maxangle)
{
    // only compute stick rotation if values changed
    if (last_rx[0] == rx_input[0] && last_rx[1] == rx_input[1])
    {

    }
    else
    {
        last_rx[0] = rx_input[0];
        last_rx[1] = rx_input[1];

        float pitch, roll;

        // rotate down vector to match stick position
        pitch = rx_input[1] * max_angle * DEGTORAD ;
        roll = rx_input[0] * max_angle * DEGTORAD ;

        stickvector[0] = fastsin(roll);
        stickvector[1] = fastsin(pitch);
        stickvector[2] = fastcos(roll) * fastcos(pitch);


        float   mag2 = (stickvector[0] * stickvector[0] + stickvector[1] * stickvector[1]);

        if (mag2 > 0.001f)
        {
            mag2 = Q_rsqrt(mag2 / (1 - stickvector[2] * stickvector[2]));
        }
        else mag2 = 0.707f;

        stickvector[0] *= mag2;
        stickvector[1] *= mag2;
    }

    // find error between stick vector and quad orientation
    // vector cross product
    errorvect[1] = -((GEstG[1] * stickvector[2]) - (GEstG[2] * stickvector[1]));
    errorvect[0] = (GEstG[2] * stickvector[0]) - (GEstG[0] * stickvector[2]);

    // some limits just in case
    limitf(&errorvect[0], 1.0);
    limitf(&errorvect[1], 1.0);

}



