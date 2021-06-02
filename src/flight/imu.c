/**
******************************************************************************
* @file    imu.c
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   imu文件，姿态相关函数.
******************************************************************************
*/

#include "imu.h"
#include "util.h"
#include "time.h"
#include "maths.h"
#include "gyro.h"
#include "mpu6500.h"
#include "IIR_filter.h"
#include "defines.h"

float GEstG[3] = { 0, 0, ACC_1G };
float attitude[3];

extern float gyro[3];
extern float accel[3];
extern float accel_one[3];
extern float accelcal[3];
extern float gyronew[3];
extern float accelraw[3];
extern float looptime;
extern int onground;
extern char aux[16];



/**************************************************************************
**函数信息 ：imu_init(void)
**功能描述 ：初始化姿态，加速度值填充
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void imu_init(void)
{
    // init the gravity vector with accel values
    for (int xx = 0; xx < 100; xx++)
    {
        mpu6500_readAcc();

        for (int x = 0; x < 3; x++)
        {
            lpf(&GEstG[x], accel[x], 0.85);
        }
        delay_us(1000);
    }
}

/**************************************************************************
**函数信息 ：calcmagnitude(float vector[3])
**功能描述 ：计算三轴加速度的矢量和
**输入参数 ：三轴加速度
**输出参数 ：矢量和
**************************************************************************/
float calcmagnitude(float vector[3])
{
    float accmag = 0;
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        accmag += vector[axis] * vector[axis];
    }
    accmag = 1.0f / Q_rsqrt(accmag);
    return accmag;
}

/**************************************************************************
**函数信息 ：imu_calc(void)
**功能描述 ：姿态估计
**输入参数 ：无
**输出参数 ：无

旋转矩阵，小角度近似估计

**************************************************************************/
void imu_calc(void)
{

    float deltaGyroAngle[3];

    for (int i = 0 ; i < 3 ; i++)
    {
        deltaGyroAngle[i] = (gyro[i]) * looptime;
    }


    GEstG[2] = GEstG[2] - (deltaGyroAngle[0]) * GEstG[0];
    GEstG[0] = (deltaGyroAngle[0]) * GEstG[2] +  GEstG[0];


    GEstG[1] =  GEstG[1] + (deltaGyroAngle[1]) * GEstG[2];
    GEstG[2] = -(deltaGyroAngle[1]) * GEstG[1] +  GEstG[2];


    GEstG[0] = GEstG[0] - (deltaGyroAngle[2]) * GEstG[1];
    GEstG[1] = (deltaGyroAngle[2]) * GEstG[0] +  GEstG[1];


//extern float stickvector[3];
    extern int onground;
    if (onground)       //happyhour bartender - quad is ON GROUND and disarmed
    {
        // calc acc mag
        float accmag = calcmagnitude(&accel[0]);
        if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G))
        {
            // normalize acc
            for (int axis = 0; axis < 3; axis++)
            {
                accel[axis] = accel[axis] * (ACC_1G / accmag);
            }

            float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FASTFILTER);
            for (int x = 0; x < 3; x++)
            {
                lpf(&GEstG[x], accel[x], filtcoeff);
            }
        }
    }
    else            //lateshift bartender - quad is IN AIR and things are getting wild
    {
#ifdef PREFILTER
        // hit accel[3] with a sledgehammer
        float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)PREFILTER);
        for (int x = 0; x < 3; x++)
        {
            static float accel_filt[3];
            lpf(&accel_filt[x], accel[x], filtcoeff);
            accel[x] = accel_filt[x];
        }
#endif
        // calc mag of filtered acc
        float accmag = calcmagnitude(&accel[0]);
//    float stickmag = calcmagnitude(&stickvector[0]);
        if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G))   //The bartender makes the fusion if.....
        {
            // normalize acc
            for (int axis = 0; axis < 3; axis++)
            {
                accel[axis] = accel[axis] * (ACC_1G / accmag);
            }
            // filter accel on to GEstG
            float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FILTERTIME);
            for (int x = 0; x < 3; x++)
            {
                lpf(&GEstG[x], accel[x], filtcoeff);
            }
            //heal the gravity vector after nudging it with accel (this is the fix for the yaw slow down bug some FC experienced)
            float GEstGmag = calcmagnitude(&GEstG[0]);
            for (int axis = 0; axis < 3; axis++)
            {
                GEstG[axis] = GEstG[axis] * (ACC_1G / GEstGmag);
            }
        }
    }

    //俯仰角  滚转角
    attitude[0] = atan2approx(GEstG[0], GEstG[2]);
    attitude[1] = atan2approx(GEstG[1], GEstG[2]);
}






