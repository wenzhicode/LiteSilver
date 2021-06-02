/**
******************************************************************************
* @file    filter_kalman.c
* @author
* @version V0.0.1
* @date    1/06/2020
* @brief   kalman�ļ���kalman�˲���غ���.
******************************************************************************

            һάkalman�˲�

        Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
        R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��
*/

#include "filter_kalman.h"
#include "config.h"
#include "defines.h"

kalmanFilter filter[3];
kalmanFilter filter2[3];


#if 0
/**********************************************************************************
**������Ϣ ��float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
**�������� : kalman�˲�
**������� �� ResrcData ��������  ��Q R ����ϵ��
**������� �� �˲�������
***********************************************************************************/
float KalmanFilter(const float ResrcData, float ProcessNiose_Q, float MeasureNoise_R)
{

    float R = MeasureNoise_R;
    float Q = ProcessNiose_Q;

    static float x_last;
    float x_mid = x_last;
    float x_now;

    static float p_last;
    float p_mid ;
    float p_now;

    float kg;

    x_mid = x_last;                     //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid = p_last + Q;                 //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����

    /*
     *  �������˲��������Ҫ��ʽ
     */
    kg = p_mid / (p_mid + R);           //kgΪkalman filter��R Ϊ����
    x_now = x_mid + kg * (ResrcData - x_mid); //���Ƴ�������ֵ
    p_now = (1 - kg) * p_mid;           //����ֵ��Ӧ��covariance
    p_last = p_now;                     //����covariance ֵ
    x_last = x_now;                     //����ϵͳ״ֵ̬

    return x_now;

}

#endif

#if defined KALMAN_GYRO && defined GYRO_FILTER_PASS1
#define SOFT_KALMAN_GYRO_PASS1 GYRO_FILTER_PASS1
#endif

#if defined KALMAN_GYRO && defined GYRO_FILTER_PASS2
#define SOFT_KALMAN_GYRO_PASS2 GYRO_FILTER_PASS2
#endif

void kalman_init(void)
{
    filter[0].Q = 0.02;
    filter[0].R = 0.1;

    filter[1].Q = 0.02;
    filter[1].R = 0.1;

    filter[2].Q = 0.02;
    filter[2].R = 0.1;


#ifdef SOFT_KALMAN_GYRO_PASS1
    filter[0].R = filter[0].Q / (float)SOFT_KALMAN_GYRO_PASS1;
    filter[1].R = filter[1].Q / (float)SOFT_KALMAN_GYRO_PASS1;
    filter[2].R = filter[2].Q / (float)SOFT_KALMAN_GYRO_PASS1;
#endif

    filter2[0].Q = 0.02;
    filter2[0].R = 0.1;

    filter2[1].Q = 0.02;
    filter2[1].R = 0.1;

    filter2[2].Q = 0.02;
    filter2[2].R = 0.1;


#ifdef SOFT_KALMAN_GYRO_PASS2
    filter2[0].R = filter2[0].Q / (float)SOFT_KALMAN_GYRO_PASS2;
    filter2[1].R = filter2[1].Q / (float)SOFT_KALMAN_GYRO_PASS2;
    filter2[2].R = filter2[2].Q / (float)SOFT_KALMAN_GYRO_PASS2;
#endif

}


float lpffilter(float in, int num)
{
    //do a prediction
    float x_temp_est = filter[num].x_est_last;
    float P_temp = filter[num].P_last + filter[num].Q;

    float K = P_temp * (1.0f / (P_temp + filter[num].R));
    float x_est = x_temp_est + K * (in - x_temp_est);
    float P = (1 - K) * P_temp;

    //update our last's
    filter[num].P_last = P;
    filter[num].x_est_last = x_est;

    return x_est;
}

float lpffilter2(float in, int num)
{
    //do a prediction
    float x_temp_est = filter2[num].x_est_last;
    float P_temp = filter2[num].P_last + filter2[num].Q;

    float K = P_temp * (1.0f / (P_temp + filter2[num].R));
    float x_est = x_temp_est + K * (in - x_temp_est);
    float P = (1 - K) * P_temp;

    //update our last's
    filter2[num].P_last = P;
    filter2[num].x_est_last = x_est;

    return x_est;
}


























