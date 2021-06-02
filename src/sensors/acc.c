/**
******************************************************************************
* @file    gyro.c
* @author
* @version V0.0.1
* @date    13/06/2020
* @brief   gyro文件，gyro相关函数.
******************************************************************************
*/



#include "acc.h"
#include "mpu6500.h"
#include "math.h"
#include "filter_biquad.h"



#define CALIBRATING_ACC_CYCLES              400

acc_t acc;

static uint16_t calibratingA = 0;

flightDynamicsTrims_t accZero;
static flightDynamicsTrims_t *accelerationTrims = &accZero;
static biquadFilter_t accFilter[3];
static int accumulatedMeasurementCount;
static float accumulatedMeasurements[3];


void acc_init(void)
{
    calibratingA = 400;
    const uint32_t accSampleTimeUs = 1e6 / 1000;
    for (int axis = 0; axis < 3; axis++)
    {
        biquadFilterInitLPF(&accFilter[axis], 10, accSampleTimeUs);
    }



}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

bool accIsCalibrationComplete(void)
{
    return calibratingA == 0;
}

static void performAcclerationCalibration(void)
{
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++)
    {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle())
        {
            a[axis] = 0;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += acc.accADC[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        acc.accADC[axis] = 0;
        accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle())
    {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationTrims->raw[0] = (a[0] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[1] = (a[1] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationTrims->raw[2] = (a[2] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - 2048;
    }

    calibratingA--;
}


static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC[0] -= accelerationTrims->raw[0];
    acc.accADC[1] -= accelerationTrims->raw[1];
    acc.accADC[2] -= accelerationTrims->raw[2];
}


void accUpdate(void)
{
    mpu6500_readAcc();

    for (int axis = 0; axis < 3; axis++)
    {
        acc.accADC[axis] = acc.dev.ADCRaw[axis];
    }

    for (int axis = 0; axis < 3; axis++)
    {
        acc.accADC[axis] = biquadFilterApply(&accFilter[axis], acc.accADC[axis]);
    }

    if (!accIsCalibrationComplete())
    {
        performAcclerationCalibration();
    }

    applyAccelerationTrims(accelerationTrims);

    ++accumulatedMeasurementCount;
    for (int axis = 0; axis < 3; axis++)
    {
        accumulatedMeasurements[axis] += acc.accADC[axis];
    }
}

