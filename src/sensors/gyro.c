/**
******************************************************************************
* @file    gyro.c
* @author
* @version V0.0.1
* @date    13/06/2020
* @brief   gyro文件，gyro相关函数.
******************************************************************************
*/




#include "gyro.h"
#include "math.h"
#include "mpu6500.h"
#include "task.h"

extern void schedulerResetTaskStatistics(taskId_e taskId);

gyroSensor_t gyro;


void gyro_init(void)
{


    float gyroDt = 500 * 1e-6f;

    float gain = pt1FilterGain(250, gyroDt);

    for (int axis = 0; axis < 3; axis++)
    {
        pt1FilterInit(&gyro.lowpass2Filter[axis], gain);
    }

    gyroDt = 1000 * 1e-6f;
    gain = pt1FilterGain(200, gyroDt);

    for (int axis = 0; axis < 3; axis++)
    {
        pt1FilterInit(&gyro.lowpassFilter[axis], gain);
    }


    gyro.calibration.cyclesRemaining = 2500;
}



static bool isOnFinalGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 1;
}


bool isOnFirstGyroCalibrationCycle(const gyroCalibration_t *gyroCalibration)
{
    return gyroCalibration->cyclesRemaining == 2500;
}

void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1)
    {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    }
    else
    {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float devVariance(stdev_t *dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}


static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor)
{
    gyroSensor->calibration.cyclesRemaining = 2500;
}



void performGyroCalibration(gyroSensor_t *gyroSensor, uint8_t gyroMovementCalibrationThreshold)
{
    for (int axis = 0; axis < 3; axis++)
    {
        // Reset g[axis] at start of calibration
        if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration))
        {
            gyroSensor->calibration.sum[axis] = 0.0f;
            devClear(&gyroSensor->calibration.var[axis]);
            // gyroZero is set to zero until calibration complete
            gyroSensor->gyroDev.gyroZero[axis] = 0.0f;
        }

        // Sum up CALIBRATING_GYRO_TIME_US readings
        gyroSensor->calibration.sum[axis] += gyroSensor->gyroDev.gyroADCRaw[axis];
        devPush(&gyroSensor->calibration.var[axis], gyroSensor->gyroDev.gyroADCRaw[axis]);

        if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration))
        {
            const float stddev = devStandardDeviation(&gyroSensor->calibration.var[axis]);
            // DEBUG_GYRO_CALIBRATION records the standard deviation of roll
            // into the spare field - debug[3], in DEBUG_GYRO_RAW

            // check deviation and startover in case the model was moved
            if (gyroMovementCalibrationThreshold && stddev > gyroMovementCalibrationThreshold)
            {
                gyroSetCalibrationCycles(gyroSensor);
                return;
            }

            // please take care with exotic boardalignment !!
            gyroSensor->gyroDev.gyroZero[axis] = gyroSensor->calibration.sum[axis] / 2500;
        }
    }

    if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration))
    {
        schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
    }

    --gyroSensor->calibration.cyclesRemaining;
}


bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor)
{
    return gyroSensor->calibration.cyclesRemaining == 0;
}


void gyroUpdateSensor(gyroSensor_t *gyroSensor)
{
    mpu6500_readGyro();

    if (isGyroSensorCalibrationComplete(gyroSensor))
    {

        gyroSensor->gyroDev.gyroADC[0] = gyroSensor->gyroDev.gyroADCRaw[0] - gyroSensor->gyroDev.gyroZero[0];
        gyroSensor->gyroDev.gyroADC[1] = gyroSensor->gyroDev.gyroADCRaw[1] - gyroSensor->gyroDev.gyroZero[1];
        gyroSensor->gyroDev.gyroADC[2] = gyroSensor->gyroDev.gyroADCRaw[2] - gyroSensor->gyroDev.gyroZero[2];

        for (int i = 0; i < 3; i++)
        {
            gyroSensor->gyroADC[i] = gyroSensor->gyroDev.gyroADC[i] * 0.061035156f * 0.017453292f;
        }

        gyroSensor->sampleSum[0] = pt1FilterApply(&gyroSensor->lowpass2Filter[0], gyroSensor->gyroADC[0]);
        gyroSensor->sampleSum[1] = pt1FilterApply(&gyroSensor->lowpass2Filter[1], gyroSensor->gyroADC[1]);
        gyroSensor->sampleSum[2] = pt1FilterApply(&gyroSensor->lowpass2Filter[2], gyroSensor->gyroADC[2]);

    }
    else
    {

        performGyroCalibration(gyroSensor, 48);
    }


}

void gyro_filter(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        float gyroADCf = 0;
        gyroADCf = gyro.sampleSum[axis];

        gyroADCf = pt1FilterApply(&gyro.lowpassFilter[axis], gyroADCf);

    }

}


