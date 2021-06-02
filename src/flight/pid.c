/**
******************************************************************************
* @file    pid.c
* @author
* @version V0.0.1
* @date    2/06/2020
* @brief   pid文件，pid相关函数.
******************************************************************************
*/


#include "pid.h"
#include "defines.h"
#include "config.h"
#include "math.h"
#include "util.h"
#include "filter_sp.h"
#include "flash.h"

//pid profile A                      Roll  PITCH  YAW
float stickAcceleratorProfileA[3] = { 0.0, 0.0, 0.0};
float stickTransitionProfileA[3]  = { 0.0, 0.0, 0.0};

//pid profile B                      Roll  PITCH  YAW
float stickAcceleratorProfileB[3] = { 1.5, 1.5, 1.0};
float stickTransitionProfileB[3]  = { 0.3, 0.3, 0.0};

float pidkp[3] ;
float pidki[3] ;
float pidkd[3] ;

/// output limit
const float outlimit[3] = { 0.8, 0.8, 0.4 };
// limit of integral term (abs)
const float integrallimit[3] = { 0.8, 0.8, 0.4 };


float *pids_array[3] = {pidkp, pidki, pidkd};
float ierror[3] = { 0, 0, 0};
float pidoutput[3];
float setpoint[3];
static float lasterror[3];
float v_compensation = 1.00;
static float lasterror2[3];
float timefactor;

static float lastrate[3];
static float lastsetpoint[3];

static float two_one_minus_alpha = 2 * FILTERCALC(0.001, (1.0f / DTERM_LPF_2ND_HZ));
static float one_minus_alpha_sqr = (FILTERCALC(0.001, (1.0f / DTERM_LPF_2ND_HZ))) * (FILTERCALC(0.001, (1.0f / DTERM_LPF_2ND_HZ)));
static float alpha_sqr = (1 - FILTERCALC(0.001, (1.0f / DTERM_LPF_2ND_HZ))) * (1 - FILTERCALC(0.001, (1.0f / DTERM_LPF_2ND_HZ)));
static float last_out[3], last_out2[3];

extern float error[3];
extern float setpoint[3];
extern float looptime;
extern float gyro[3];
extern int onground;
extern float looptime;
extern int in_air;
extern char aux[16];
extern float vbattfilt;
extern float lipo_cell_count;
extern unsigned char pid_noinit;



/**************************************************************************
**函数信息 ：void pid_init(void)
**功能描述 : 根据电池节数初始化PID值
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void pid_init(void)
{
    if(pid_noinit)
    {
        pid_noinit = 0;
        if (lipo_cell_count == 1)
        {
            pidkp[0] = 0.1438;
            pidkp[1] = 0.1438;
            pidkp[2] = 0.2436;

            pidki[0] = 1.4;
            pidki[1] = 1.4;
            pidki[2] = 1.4;

            pidkd[0] = 0.5666;
            pidkd[1] = 0.5666;
            pidkd[2] = 0.05;

        }
        else if (lipo_cell_count == 2)
        {
            pidkp[0] = 0.10;
            pidkp[1] = 0.10;
            pidkp[2] = 0.08;

            pidki[0] = 1.00;
            pidki[1] = 1.05;
            pidki[2] = 1.50;

            pidkd[0] = 0.37;
            pidkd[1] = 0.39;
            pidkd[2] = 0.05;
        }
        flash_save();
    }
}
//

/**************************************************************************
**函数信息 ：void pid_precalc(void)
**功能描述 : 预先计算pid使用到的系数
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void pid_precalc(void)
{
    timefactor = 0.0032f / looptime;

#ifdef PID_VOLTAGE_COMPENSATION

    v_compensation = mapf((vbattfilt / lipo_cell_count), 2.5, 3.85, PID_VC_FACTOR, 1.00);

    if (v_compensation > PID_VC_FACTOR)
        v_compensation = PID_VC_FACTOR;

    if (v_compensation < 1.00f)
        v_compensation = 1.00;

#ifdef LEVELMODE_PID_ATTENUATION
    if (aux[LEVELMODE]) v_compensation *= LEVELMODE_PID_ATTENUATION;
#endif

#endif
}


/**************************************************************************
**函数信息 ：float lpf2( float in, int num)
**功能描述 : 二阶低通滤波
**输入参数 ：in 输入值； num 项数
**输出参数 ：滤波后的值
**************************************************************************/
float lpf2(float in, int num)
{

    float ans = in * alpha_sqr + two_one_minus_alpha * last_out[num]
                - one_minus_alpha_sqr * last_out2[num];

    last_out2[num] = last_out[num];
    last_out[num] = ans;

    return ans;
}
#define SIMPSON_RULE_INTEGRAL

/**************************************************************************
**函数信息 ：float pid(int x )
**功能描述 : pid计算
**输入参数 ：x 项数 0 1 2 ， 即三个方向
**输出参数 ：项数对应的pid输出值
**************************************************************************/
float pid(int x)
{
    if ((aux[LEVELMODE]) && (!aux[RACEMODE]))
    {
        if ((onground) || (in_air == 0))
        {
            ierror[x] *= 0.98f;
        }
    }
    else
    {
        if (onground) ierror[x] *= 0.98f;
    }

// pid tuning via analog aux channels
#ifdef ANALOG_AUX_PIDS
    apply_analog_aux_to_pids();
#endif

#ifdef TRANSIENT_WINDUP_PROTECTION
    static float avgSetpoint[3];
    static int count[3];
    extern float splpf(float in, int num);

    if (x < 2 && (count[x]++ % 2) == 0)
    {
        avgSetpoint[x] = splpf(setpoint[x], x);
    }
#endif

    int iwindup = 0;
    if ((pidoutput[x] == outlimit[x]) && (error[x] > 0))
    {
        iwindup = 1;
    }

    if ((pidoutput[x] == -outlimit[x]) && (error[x] < 0))
    {
        iwindup = 1;
    }

#ifdef ANTI_WINDUP_DISABLE
    iwindup = 0;
#endif

#ifdef TRANSIENT_WINDUP_PROTECTION
    if (x < 2 && fabsf(setpoint[x] - avgSetpoint[x]) > 0.1f)
    {
        iwindup = 1;
    }
#endif

    if (!iwindup)
    {
#ifdef MIDPOINT_RULE_INTEGRAL
        // trapezoidal rule instead of rectangular
        ierror[x] = ierror[x] + (error[x] + lasterror[x]) * 0.5f *  pidki[x] * looptime;
        lasterror[x] = error[x];
#endif

#ifdef RECTANGULAR_RULE_INTEGRAL
        ierror[x] = ierror[x] + error[x] *  pidki[x] * looptime;
        lasterror[x] = error[x];
#endif

#ifdef SIMPSON_RULE_INTEGRAL
        // assuming similar time intervals
        ierror[x] = ierror[x] + 0.166666f * (lasterror2[x] + 4 * lasterror[x] + error[x]) *  pidki[x] * looptime;
        lasterror2[x] = lasterror[x];
        lasterror[x] = error[x];
#endif
    }

    limitf(&ierror[x], integrallimit[x]);


#ifdef ENABLE_SETPOINT_WEIGHTING
    // P term
    pidoutput[x] = error[x] * (b[x]) * pidkp[x];
    // b
    pidoutput[x] +=  - (1.0f - b[x]) * pidkp[x] * gyro[x];
#else
    // P term with b disabled
    pidoutput[x] = error[x] * pidkp[x];
#endif

    // I term
    pidoutput[x] += ierror[x];

    // D term
    // skip yaw D term if not set
    if (pidkd[x] > 0)
    {

#if (defined DTERM_LPF_1ST_HZ && !defined ADVANCED_PID_CONTROLLER)
        float dterm;
        static float lastrate[3];
        static float dlpf[3] = {0};

        dterm = - (gyro[x] - lastrate[x]) * pidkd[x] * timefactor;
        lastrate[x] = gyro[x];
        lpf(&dlpf[x], dterm, FILTERCALC(0.001, 1.0f / DTERM_LPF_1ST_HZ));
        pidoutput[x] += dlpf[x];
#endif

#if (defined DTERM_LPF_1ST_HZ && defined ADVANCED_PID_CONTROLLER)
        extern float rxcopy[4];
        float dterm;
        float transitionSetpointWeight[3];
        float stickAccelerator[3];
        float stickTransition[3];
        if (aux[PIDPROFILE])
        {
            stickAccelerator[x] = stickAcceleratorProfileB[x];
            stickTransition[x] = stickTransitionProfileB[x];
        }
        else
        {
            stickAccelerator[x] = stickAcceleratorProfileA[x];
            stickTransition[x] = stickTransitionProfileA[x];
        }
        if (stickAccelerator[x] < 1)
        {
            transitionSetpointWeight[x] = (fabs(rxcopy[x]) * stickTransition[x]) + (1 - stickTransition[x]);
        }
        else
        {
            transitionSetpointWeight[x] = (fabs(rxcopy[x]) * (stickTransition[x] / stickAccelerator[x])) + (1 - stickTransition[x]);
        }
        static float lastrate[3];
        static float lastsetpoint[3];
        static float dlpf[3] = {0};

        dterm = ((setpoint[x] - lastsetpoint[x]) * pidkd[x] * stickAccelerator[x] * transitionSetpointWeight[x] * timefactor) - ((gyro[x] - lastrate[x]) * pidkd[x] * timefactor);
        lastsetpoint[x] = setpoint [x];
        lastrate[x] = gyro[x];
        lpf(&dlpf[x], dterm, FILTERCALC(0.001, 1.0f / DTERM_LPF_1ST_HZ));
        pidoutput[x] += dlpf[x];
#endif

#if (defined DTERM_LPF_2ND_HZ && !defined ADVANCED_PID_CONTROLLER)
        float dterm;
        static float lastrate[3];
        float lpf2(float in, int num);

        dterm = - (gyro[x] - lastrate[x]) * pidkd[x] * timefactor;
        lastrate[x] = gyro[x];
        dterm = lpf2(dterm, x);
        pidoutput[x] += dterm;
#endif

#if (defined DTERM_LPF_2ND_HZ && defined ADVANCED_PID_CONTROLLER)
        extern float rxcopy[4];
        float dterm;
        float transitionSetpointWeight[3];
        float stickAccelerator[3];
        float stickTransition[3];

        stickAccelerator[x] = stickAcceleratorProfileA[x];
        stickTransition[x] = stickTransitionProfileA[x];

        if (stickAccelerator[x] < 1)
        {
            transitionSetpointWeight[x] = (fabs(rxcopy[x]) * stickTransition[x]) + (1 - stickTransition[x]);
        }
        else
        {
            transitionSetpointWeight[x] = (fabs(rxcopy[x]) * (stickTransition[x] / stickAccelerator[x])) + (1 - stickTransition[x]);
        }
        static float lastrate[3];
        static float lastsetpoint[3];
        float lpf2(float in, int num);

        dterm = ((setpoint[x] - lastsetpoint[x]) * pidkd[x] * stickAccelerator[x] * transitionSetpointWeight[x] * timefactor) - ((gyro[x] - lastrate[x]) * pidkd[x] * timefactor);
        lastsetpoint[x] = setpoint [x];
        lastrate[x] = gyro[x];
        dterm = lpf2(dterm, x);
        pidoutput[x] += dterm;
#endif

    }

#ifdef PID_VOLTAGE_COMPENSATION
    pidoutput[x] *= v_compensation;
#endif
    limitf(&pidoutput[x], outlimit[x]);

    return pidoutput[x];
}

/**************************************************************************
**函数信息 ：void rotateErrors(void)
**功能描述 : joelucid's yaw fix
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void rotateErrors(void)
{
#ifdef YAW_FIX
    // rotation around x axis:
    ierror[1] -= ierror[2] * gyro[0] * looptime;
    ierror[2] += ierror[1] * gyro[0] * looptime;

    // rotation around y axis:
    ierror[2] -= ierror[0] * gyro[1] * looptime;
    ierror[0] += ierror[2] * gyro[1] * looptime;

    // rotation around z axis:
    ierror[0] -= ierror[1] * gyro[2] * looptime;
    ierror[1] += ierror[0] * gyro[2] * looptime;
#endif
}




