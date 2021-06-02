/**
******************************************************************************
* @file    task.c
* @author
* @version V0.0.1
* @date    11/06/2020
* @brief   task文件，task相关函数.
******************************************************************************

定义任务

*/



#include "task.h"
#include "scheduler.h"
#include "string.h"
#include "blackbox.h"
#include "msp.h"
#include "mavlink.h"
#include "usbprop.h"
#include "stdbool.h"
#include "spl06001.h"
#include "gyro.h"
#include "acc.h"
#include "imu.h"
#include "battery.h"
#include "osd.h"
#include "control.h"
#include "rx_serial_dsm.h"
#include "rx_spi.h"
#include "time.h"
#include "defines.h"
#include "mpu6500.h"
#include "dshot.h"
#include "adc.h"
#include "util.h"
#include "serial_uart.h"
#include "rx_serial_sbus.h"
#include "rx_serial_dsm.h"
#include "led.h"
#include "rgb_led.h"
#include "cc2500.h"
#include "flash.h"
#include "rx_bayang.h"
#include "bus_softspi.h"
#include "configure.h"
#include "rx_bayang.h"
#include "bus_softspi.h"
#include "xn297.h"
#include "maths.h"


//variables
unsigned char txtest[3] = {1, 2, 3};
unsigned int Lenth = 3;
unsigned char rxTestInfoBuf[64];
unsigned int rxTestInfoLen = 0;
unsigned int uiCnt = 0;
extern volatile USB_STATE_t usb_state;

extern void setTaskEnabled(taskId_e taskId, bool enabled);


float temperature;
float presure;
u32 baro_height;

unsigned char IsApp = 0;
bool blackboxEnabled;

uint16_t averageSystemLoadPercent = 0;
uint8_t pidUpdateCounter = 0;

bool rxIsInFailsafeMode = true;
uint32_t needRxSignalBefore = 0;
uint32_t needRxSignalMaxDelayUs = (1000000 / 10);
bool auxiliaryProcessingRequired = false;
bool rxSignalReceived = false;
bool rxDataProcessingRequired = false;
timeUs_t rxNextUpdateAtUs = 0;

extern uint32_t totalWaitingTasksSamples;
extern uint32_t totalWaitingTasks;

extern void frSkyXSetRcData(uint16_t *rcData, const uint8_t *packet);
extern void frSkyDSetRcData(uint16_t *rcData, const uint8_t *packet);
extern int failsafe;
extern int ledcommand;
extern rx_spi_received_e frSkySpiDataReceived(uint8_t *packet);
extern float rx[4];
extern char aux[16];
extern int binding_while_armed ;
extern int rx_ready;
extern int ledcommand;

extern uint32_t  init_time;

uint8_t packet[35];
uint16_t rcRaw[18];     // interval [1000;2000]
uint16_t rcData[18];     // interval [1000;2000]
uint32_t rcInvalidPulsPeriod[18];

uint32_t lasttime, looptime1;
float looptime;
unsigned int lastlooptime;


extern rx_spi_received_e frSkySpiDataReceived(uint8_t *packet);


float pressure;

unsigned int pwm_count = 0;
char motorDir[4] = {0, 1, 1, 0};

extern uint8_t openLogBuff[64];


extern int rxmode;
extern int failsafe;
extern float attitude[3];
extern int16 accEf[3];

unsigned char flymode = 0;
unsigned char turtlemode = 0;
unsigned char aux6;
unsigned char aux7;
unsigned char aux8;
unsigned char turtle_change=0;
unsigned char low_vol=0;
float lowvol = 3.3;

float pressure;
int16 alt_temp;
static u8 isBaro = 0;

unsigned int vol = 0;
float electricCur = 0;
unsigned int cur = 0;

int onground = 1;
int in_air;
int armed_state;
int arming_release;
int binding_while_armed = 1;
char aux[16] = { 0, 0, 0, 0, 0, 0};
float rx[4] = {0};

int lowbatt = 1;

// filtered battery in volts
float vbattfilt = 0.0;
float vbatt_comp = 4.2;
// voltage reference for vcc compensation
float vreffilt = 1.0;
// average of all motors
float thrfilt = 0;

float lipo_cell_count = 1;
extern unsigned char low_battery;
unsigned char rx_reboot = 0;
static bool rx_received = false;

static u32 key_cnt = 0;
extern unsigned char rx_select;
extern uint32_t packetTimerUs;
extern uint32_t missingPackets;
extern int32_t timeoutUs;
extern uint8_t protocolState;
extern unsigned char rx_show;
extern unsigned char mode_show;
extern unsigned char osd_mode;
extern unsigned char vol_show;
extern sbusFrameData_t sbusFrameData;
extern int rcFrameComplete;
extern uint8_t rxSpiNewPacketAvailable;
extern int packetreceived;
extern unsigned char showcase;
extern uint8_t rssip;
extern unsigned char motor_select;
uint8_t init_motor_dir = 0;
extern unsigned char osd_mode;

extern unsigned char vtx_index;
extern unsigned char band_index;
extern u32 vtx_freq;
extern u32 old_freq;
extern unsigned char motor_change;
uint8_t crash = 0;
extern float attitude[3];
extern uint16_t chan[4];
#define DELAY_33_HZ (1000000 / 33)
extern unsigned char ep1_tx_flag;	

#define RX_MIN_USEC 885
#define RX_MAX_USEC 2115
#define RX_MID_USEC 1500
#define MAX_INVALID_PULS_TIME    300

static bool rxFlightChannelsValid = false;


#define DEFINE_TASK(taskNameParam, subTaskNameParam, checkFuncParam, taskFuncParam, desiredPeriodParam, staticPriorityParam) {  \
    .taskName = taskNameParam, \
    .subTaskName = subTaskNameParam, \
    .checkFunc = checkFuncParam, \
    .taskFunc = taskFuncParam, \
    .desiredPeriodUs = desiredPeriodParam, \
    .staticPriority = staticPriorityParam \
}


/********************************************************************************
**函数信息 ：taskGyroSample(timeUs_t currentTimeUs)
**功能描述 ：gyro更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskGyroSample(timeUs_t currentTimeUs)
{
    if (pidUpdateCounter % 2 == 0)
    {
        pidUpdateCounter = 0;
    }
    pidUpdateCounter++;
}

#ifdef USE_RX_FRSKY

/********************************************************************************
**函数信息 ：taskFiltering(timeUs_t currentTimeUs)
**功能描述 ：gyro 滤波更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskFiltering(timeUs_t currentTimeUs)
{
    mpu6500_readGyro();

    if(rx_reboot == 2)
    {
        sbus_check();
    }
}

/********************************************************************************
**函数信息 ：taskUpdateRxMain(timeUs_t currentTimeUs)
**功能描述 ：接收遥控数据 更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime)
{
    uint8_t frameStatus = 0;
    bool signalReceived = false;

    frameStatus = rxSpiFrameStatus();
    if (frameStatus & RX_FRAME_COMPLETE)
    {
        rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
        bool rxFrameDropped = (frameStatus & RX_FRAME_DROPPED) != 0;
        signalReceived = !(rxIsInFailsafeMode || rxFrameDropped);
        if (signalReceived) {
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }
    }

    if (signalReceived) {
        rxSignalReceived = true;
    } else if (currentTimeUs >= needRxSignalBefore) {
        rxSignalReceived = false;
    }
    if ((signalReceived) || cmpTimeUs(currentTimeUs, rxNextUpdateAtUs) > 0 ) {
        rxDataProcessingRequired = true;
    }

    return rxDataProcessingRequired; // data driven or 50Hz
}



bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= RX_MIN_USEC &&
            pulseDuration <= RX_MAX_USEC;
}

static uint16_t getRxfailValue(uint8_t channel)
{
    switch (channel) {
    case 0:
    case 1:
    case 3:
        return 1500;
        break;
    case 2:
        return 1000;
    }
    return 0;
}

void taskUpdateRxMain(timeUs_t currentTimeUs)
{
    if (!rxDataProcessingRequired || rx_reboot) {
        return ;
    }

    rxDataProcessingRequired = false;
    rxNextUpdateAtUs = currentTimeUs + DELAY_33_HZ;

    getRssiPercent();

    if (rxSpiNewPacketAvailable) {
        setRcData(rcRaw, packet);
        rxSpiNewPacketAvailable = false;
    }
    const uint32_t currentTimeMs = millis();
    const bool useValueFromRx = rxSignalReceived && !rxIsInFailsafeMode;

    rxFlightChannelsValid = true;
    for (int channel = 0; channel < 8; channel++) {
        uint16_t sample = rcRaw[channel];

        const bool validPulse = useValueFromRx && isPulseValid(sample);

        if (validPulse) {
            rcInvalidPulsPeriod[channel] = currentTimeMs + MAX_INVALID_PULS_TIME;
        } else {
            if (cmp32(currentTimeMs, rcInvalidPulsPeriod[channel]) < 0) {
                continue;           // skip to next channel to hold channel value MAX_INVALID_PULS_TIME
            } else {
//                sample = getRxfailValue(channel);   // after that apply rxfail value
                if (channel < 4) {
                    rxFlightChannelsValid = false;
                }
            }
        }

        rcData[channel] = sample;
    }
    if (rxFlightChannelsValid) {
        failsafe = 0;
        lasttime = gettime();

        rx[0] = rcData[0] - 1500;
        rx[1] = rcData[1] - 1500;
        rx[2] = rcData[3] - 1500;

        for (int i = 0 ; i < 3 ; i++)
        {
            rx[i] *= 0.002f;
            if (rx[i] < -1)
                rx[i] = -1;
            if (rx[i] > 1)
                rx[i] = 1;
        }

        rx[3] = (rcData[2] - 1000) / 1000.0f;

        if (rx[3] > 1) rx[3] = 1;
        if (rx[3] < 0) rx[3] = 0;

        chan[0] = rcData[4];
        chan[1] = rcData[5];
        chan[2] = rcData[6];
        chan[3] = rcData[7];

        aux[CHAN_5] = (rcData[4] > 1200) ? ((rcData[4] < 1800) ? 1 : 2) : 0;
        aux[CHAN_6] = (rcData[5] > 1200) ? ((rcData[5] < 1800) ? 1 : 2) : 0;
        aux[CHAN_7] = (rcData[6] > 1200) ? ((rcData[6] < 1800) ? 1 : 2) : 0;
        aux[CHAN_8] = (rcData[7] > 1200) ? ((rcData[7] < 1800) ? 1 : 2) : 0;

//        switch(flymode)
//        {
//        case 0:
//            if (ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
//            if (ANGLE_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ANGLE_EXPO_PITCH);
//            if (ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
//            break;

//        case 1:
//            if (ACRO_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
//            if (ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
//            if (ACRO_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ACRO_EXPO_YAW);
//            break;

//        case 2:
//            if (ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
//            if (ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
//            if (ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
//            break;

//        case 3:
//            if (ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
//            if (ANGLE_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ANGLE_EXPO_PITCH);
//            if (ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
//            break;

//        case 4:
//            if (ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
//            if (ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
//            if (ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
//            break;

//        }

        rx_ready = 1;
        rxmode = !RXMODE_BIND;
        ledcommand = 4;
    }
    else
    {
        if (gettime() - lasttime > 1000000)
        {
            rxIsInFailsafeMode = true;
            failsafe = 1;
            rx_ready = 0;

            rx[0] = 0;
            rx[1] = 0;
            rx[2] = 0;
            rx[3] = 0;

            if(ledcommand !=2)
                ledcommand = 3;
        }
    }
}


/********************************************************************************
**函数信息 ：taskCalculateAltitude(timeUs_t currentTimeUs)
**功能描述 ：位置 更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskCalculateAltitude(timeUs_t currentTimeUs)
{
    if(millis() - init_time >8000)
    {
        turtlemode = 0;
        if (osd_mode)
        {
            if (aux[LEVELMODE])
            {
                if (aux[RACEMODE])
                {
                    if (aux[HORIZON])
                    {
                        flymode = 2;
                    }
                    else
                    {
                        flymode = 3;
                    }
                }
                else
                {
                    if (aux[HORIZON])
                    {
                        flymode = 4;
                    }
                    else
                    {
                        flymode = 0;
                    }
                }
            }
            else
            {
                flymode = 1;

                if (aux[RACEMODE] && !aux[HORIZON])
                {
                    turtlemode = 1;
                    init_motor_dir = 0;
                }
            }

        }
        else
        {
            if (!aux[LEVELMODE])
            {
                flymode=0;
            }
            else if(aux[LEVELMODE] == 1)
            {
                flymode=4;
            }
            else
            {
                flymode=1;
            }
            if (aux7 != aux[RACEMODE])
            {
                aux7 = aux[RACEMODE];
                vtx_index++;
                if (vtx_index > 15 || vtx_index <8)
                    vtx_index = 8;

                vtx_freq = get_vtx(vtx_index);
                old_freq = vtx_freq;
                Set_RTC6705_Freq(vtx_freq);
            }
            turtlemode = 0;
            if (aux[HORIZON])
            {
                turtlemode = 1;
                init_motor_dir = 0;
            }
            if(flymode==0 && (attitude[0] > (65 + 10) || attitude[0] < -(65 + 10) || attitude[1] > (65 + 10) || attitude[1] <-(65 + 10)))
            {
                crash = 1;
            }
        }
    }
}

#endif

#ifdef USE_RX_SBUS_DSM_BY

/********************************************************************************
**函数信息 ：taskFiltering(timeUs_t currentTimeUs)
**功能描述 ：gyro 滤波更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskFiltering(timeUs_t currentTimeUs)
{
    uint8_t frameStatus = 0;

    mpu6500_readGyro();

    switch (rx_reboot)
    {
    case 2:
        sbus_check();
        break;

    case 3:
        dsm_check();
        break;

    case 4:
        checkrx();
        break;
    }

}

/********************************************************************************
**函数信息 ：taskUpdateRxMain(timeUs_t currentTimeUs)
**功能描述 ：接收遥控数据 更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskUpdateRxMain(timeUs_t currentTimeUs)
{
    turtlemode = 0;
    if (osd_mode)
    {
        if (aux[LEVELMODE])
        {
            if (aux[RACEMODE])
            {
                if (aux[HORIZON])
                {
                    flymode = 2;
                }
                else
                {
                    flymode = 3;
                }
            }
            else
            {
                if (aux[HORIZON])
                {
                    flymode = 4;
                }
                else
                {
                    flymode = 0;
                }
            }
        }
        else
        {
            flymode = 1;

            if (aux[RACEMODE] && !aux[HORIZON])
            {
                turtlemode = 1;
                init_motor_dir = 0;
            }
        }

    }
    else
    {
        if (!aux[LEVELMODE])
        {
            flymode=0;
        }
        else if(aux[LEVELMODE] == 1)
        {
            flymode=4;
        }
        else
        {
            flymode=1;
        }

        if (aux7 != aux[RACEMODE])
        {

            vtx_index++;
            if (vtx_index > 15 || vtx_index <8)
                vtx_index = 8;

            vtx_freq = get_vtx(vtx_index);
            old_freq = vtx_freq;
            Set_RTC6705_Freq(vtx_freq);

            flash_save();

            aux7 = aux[RACEMODE];

        }
        turtlemode = 0;
        if (aux[HORIZON])
        {
            turtlemode = 1;
            init_motor_dir = 0;
        }
        if(flymode==0 && (attitude[0] > (65 + 10) || attitude[0] < -(65 + 10) || attitude[1] > (65 + 10) || attitude[1] <-(65 + 10)))
        {
            crash = 1;
        }
    }
}
#endif

/********************************************************************************
**函数信息 ：taskMainPidLoop(timeUs_t currentTimeUs)
**功能描述 ：pid 更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskMainPidLoop(timeUs_t currentTimeUs)
{
    unsigned long time = gettime();
    looptime = ((uint32_t)(time - lastlooptime));

    looptime = looptime * 1e-6f;

    if (looptime <= 0.001f) looptime = 0.001f;
    lastlooptime = time;


    control();

    imu_calc();
}


/********************************************************************************
**函数信息 ：taskUpdateAccelerometer(timeUs_t currentTimeUs)
**功能描述 ：accel 更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskUpdateAccelerometer(timeUs_t currentTimeUs)
{

}


/********************************************************************************
**函数信息 ：imuUpdateAttitude(timeUs_t currentTimeUs)
**功能描述 ：姿态 更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void imuUpdateAttitude(timeUs_t currentTimeUs)
{

}



/********************************************************************************
**函数信息 ：taskUpdateBaro(timeUs_t currentTimeUs)
**功能描述 ：气压计 更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskUpdateBaro(timeUs_t currentTimeUs)
{

}



/********************************************************************************
**函数信息 ：osdUpdate(timeUs_t currentTimeUs)
**功能描述 ：osd更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无

osd / blackbox 共用串口2

*********************************************************************************/
void osdUpdate(timeUs_t currentTimeUs)
{
    vol = (int)(vbattfilt * 100);

    if( (vbattfilt / lipo_cell_count) < 3.3)
    {
        low_vol = 1;
    }
    else {
        low_vol = 0;
    }

    if (showcase == 0 && aux[ARMING])
    {       
        openLogBuff[0] = 0x0f;
        openLogBuff[0] |= 0 << 4;
        openLogBuff[1] = (aux[CHAN_5] << 0) | (aux[CHAN_6] << 2) | (aux[CHAN_7] << 4) | (aux[CHAN_8] << 6) ;
        openLogBuff[2] = (osd_mode<<0) | (failsafe << 1) | (turtlemode<<2) | flymode <<4 ;
        openLogBuff[3] = vol >> 8;
        openLogBuff[4] = vol & 0xFF;
        openLogBuff[5] = low_vol | rx_select << 4;

        openLogBuff[6] = 0;
        openLogBuff[6] = (vol_show << 0) | (rx_show << 1) | (mode_show << 2) | (rx[0] < 0 ? 1 : 0) << 4 | (rx[1] < 0 ? 1 : 0) << 5 | (rx[2] < 0 ? 1 : 0) << 6;

        openLogBuff[7]  = round(fabs(rx[0]) * 100);
        openLogBuff[8] =  round(fabs(rx[1]) * 100);
        openLogBuff[9] =  round(fabs(rx[2]) * 100);
        openLogBuff[10] =  round(fabs(rx[3]) * 100);

        openLogBuff[11] = rssip;
        openLogBuff[12] = 0;
        for (uint8_t i = 0; i < 12; i++)
            openLogBuff[12] += openLogBuff[i];

        UART2_DMA_Send();

    }
    else
    {
        osd_process();

        if (init_motor_dir < 200)
        {
            init_motor_dir++;

            if (motor_select)
            {
                if(!motor_change || (!turtlemode && turtle_change))
                {
                    turtle_change = 0;
                    motorDir[0] = 0;
                    motorDir[1] = 1;
                    motorDir[2] = 1;
                    motorDir[3] = 0;
                }
                if(turtlemode)
                {
                    turtle_change = 1;
                    motorDir[0] = 1;
                    motorDir[1] = 0;
                    motorDir[2] = 0;
                    motorDir[3] = 1;
                }
            }
            else
            {
                if(!motor_change || (!turtlemode && turtle_change))
                {
                    turtle_change = 0;
                    motorDir[0] = 1;
                    motorDir[1] = 0;
                    motorDir[2] = 0;
                    motorDir[3] = 1;
                }
                if(turtlemode)
                {
                    turtle_change = 1;
                    motorDir[0] = 0;
                    motorDir[1] = 1;
                    motorDir[2] = 1;
                    motorDir[3] = 0;
                }
            }

            for (int i = 20; i > 0; i--)
            {
                motor_dir(0, (motorDir[0] ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL));
                motor_dir(1, (motorDir[1] ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL));
                motor_dir(2, (motorDir[2] ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL));
                motor_dir(3, (motorDir[3] ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL));
            }
        }
    }
}


/********************************************************************************
**函数信息 ：batteryUpdateVoltage(timeUs_t currentTimeUs)
**功能描述 ：电压、电流更新 更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void batteryUpdateVoltage(timeUs_t currentTimeUs)
{
    float battadc = adc_read(0) * vreffilt;
    // read and filter internal reference
    lpf(&vreffilt, adc_read(1), 0.9968f);


    // average of all 4 motor thrusts
    // should be proportional with battery current
    extern float thrsum; // from control.c

    // filter motorpwm so it has the same delay as the filtered voltage
    // ( or they can use a single filter)
    lpf(&thrfilt, thrsum, 0.9968f);      // 0.5 sec at 1.6ms loop time

    float vbattfilt_corr = 4.2f * lipo_cell_count;
    // li-ion battery model compensation time decay ( 18 seconds )
    lpf(&vbattfilt_corr, vbattfilt, FILTERCALC(1000, 18000e3));

    lpf(&vbattfilt, battadc, 0.9968f);


#define CF1 0.25f

    float tempvolt = vbattfilt * (1.00f + CF1)  - vbattfilt_corr * (CF1);

    static float lastout[12];
    static float lastin[12];
    static float vcomp[12];
    static float score[12];
    static int z = 0;
    static int minindex = 0;
    static int firstrun = 1;

    if (thrfilt > 0.1f)
    {
        vcomp[z] = tempvolt + (float) z * 0.1f * thrfilt;

        if (firstrun)
        {
            for (int y = 0 ; y < 12; y++) lastin[y] = vcomp[z];
            firstrun = 0;
        }
        float ans;
        //  y(n) = x(n) - x(n-1) + R * y(n-1)
        //  out = in - lastin + coeff*lastout
        // hpf
        ans = vcomp[z] - lastin[z] + FILTERCALC(1000 * 12, 6000e3) * lastout[z];
        lastin[z] = vcomp[z];
        lastout[z] = ans;
        lpf(&score[z], ans * ans, FILTERCALC(1000 * 12, 60e6));
        z++;

        if (z >= 12)
        {
            z = 0;
            float min = score[0];
            for (int i = 0 ; i < 12; i++)
            {
                if ((score[i]) < min)
                {
                    min = (score[i]);
                    minindex = i;
                    // add an offset because it seems to be usually early
                    minindex++;
                }
            }
        }

    }

#undef VDROP_FACTOR
#define VDROP_FACTOR  minindex * 0.1f

    float hyst;
    if (lowbatt) hyst = HYST;
    else hyst = 0.0f;

    if ((tempvolt + (float) VDROP_FACTOR * thrfilt < (float) VBATTLOW + hyst)
            || (vbattfilt < (float) 2.7f))
        lowbatt = 1;
    else lowbatt = 0;

    vbatt_comp = tempvolt + (float) VDROP_FACTOR * thrfilt;
}


void batteryUpdateCurrentMeter(timeUs_t currentTimeUs)
{

}

uint32 usb_val;

/********************************************************************************
**函数信息 ：taskHandleSerial(timeUs_t currentTimeUs)
**功能描述 ：usb_vcp更新任务
**输入参数 ：currentTimeUs 当前时间
**输出参数 ：无
*********************************************************************************/
void taskHandleSerial(timeUs_t currentTimeUs)
{
//    mspSerialProcess();
    
    static int connect_or_not=0;  
    if(aux[CHAN_5] && ep1_tx_flag)
    {
        if(connect_or_not)
        {
            connect_or_not=0;  
            usb_val = USB->rTOP;
        
            usb_val &= ~USB_TOP_CONNECT;
            
            USB->rTOP = usb_val;
            USB->rPOWER = USB_POWER_SUSPEN ;  
            delay_ms(10);
            
            usb_val = USB->rTOP;
            usb_state = USB_STATE_SUSPENDED;
        }
    }
    else
    {
        if(connect_or_not==0)
        {
            connect_or_not = 1;
            USB->rTOP = USB_TOP_CONNECT | ((~USB_TOP_SPEED) & 0x01); //连接USB进入工作模式
            USB->rPOWER = USB_POWER_SUSPEN | USB_POWER_SUSP;        //由控制器决定挂起状态 
            delay_ms(10);
            usb_val = USB->rTOP;
            usb_state = USB_STATE_CONFIGURED;
        }
        serialProcess();
    }
}

void ledUpdate(timeUs_t currentTimeUs)
{
    rgb_led_lvc();
}

void taskSystemLoad(timeUs_t currentTimeUs)
{
    // Calculate system load
    if (totalWaitingTasksSamples > 0)
    {
        averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }
}

void taskKey(timeUs_t currentTimeUs)
{
    if ((GPIOB->IDR & GPIO_Pin_8) == (uint32_t)Bit_RESET)
    {
        delay_ms(10);
        while ((GPIOB->IDR & GPIO_Pin_8) == (uint32_t)Bit_RESET)
        {
            key_cnt ++;
            delay_ms(1);
        }
        if (key_cnt > 2000)
        {
            key_cnt = 0;

#ifdef USE_RX_FRSKY
            if(rx_select ==0)
            {
                rx_select = 2;
            }
            else
            {
                rx_select--;
            }
            protocolState = STATE_INIT;
            rxCc2500SpiConfigMutable.autoBind = true;
#endif

#ifdef USE_RX_SBUS_DSM_BY
						if(rx_select ==3)
            {
                rx_select = 5;
            }
            else
            {
                rx_select--;
            }
#endif

            flash_save();
            rx_reboot = rx_select;
            ledcommand = 2;
        }
        else if (key_cnt > 80)
        {
            rx_ready = 0;
            rxmode = RXMODE_BIND;
            ledcommand = 2;
            key_cnt = 0;
            packetTimerUs = 0;
            timeoutUs = 50;
            missingPackets = 0;
            protocolState = STATE_INIT;
            rxCc2500SpiConfigMutable.autoBind = true;
            if(rx_select == 5)
            {
                static uint8_t rxaddr[6] = { 0x2a, 0, 0, 0, 0, 0  };
                writeregs( rxaddr, sizeof(rxaddr) );
                xn_writereg(RF_CH, 0);
            }
        }
    }
}

/********************************************************************************
**函数信息 ：
**功能描述 ：任务结构体初始化
**输入参数 ：
**输出参数 ：无
*********************************************************************************/
#ifdef USE_RX_FRSKY

task_t tasks[TASK_COUNT] =
{
    [TASK_SYSTEM] = DEFINE_TASK("SYSTEM", "LOAD", NULL, taskSystemLoad, TASK_PERIOD_HZ(1), TASK_PRIORITY_MEDIUM_HIGH),
    [TASK_SERIAL] = DEFINE_TASK("SERIAL", NULL, NULL, taskHandleSerial, TASK_PERIOD_HZ(30), TASK_PRIORITY_LOW),
    [TASK_GYRO] = DEFINE_TASK("GYRO", NULL, NULL, taskGyroSample, TASK_PERIOD_HZ(2000), TASK_PRIORITY_REALTIME),
    [TASK_FILTER] = DEFINE_TASK("FILTER", NULL, NULL, taskFiltering, TASK_PERIOD_HZ(1000), TASK_PRIORITY_REALTIME),
    [TASK_PID] = DEFINE_TASK("PID", NULL, NULL, taskMainPidLoop, TASK_PERIOD_HZ(1000), TASK_PRIORITY_REALTIME),
//    [TASK_ACCEL] = DEFINE_TASK("ACC", NULL, NULL, taskUpdateAccelerometer, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM),
//    [TASK_ATTITUDE] = DEFINE_TASK("ATTITUDE", NULL, NULL, imuUpdateAttitude, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM),
    [TASK_RX] = DEFINE_TASK("RX", NULL, rxUpdateCheck, taskUpdateRxMain, TASK_PERIOD_HZ(33), TASK_PRIORITY_HIGH),
//    [TASK_BARO] = DEFINE_TASK("BARO", NULL, rxUpdateCheck, taskUpdateBaro, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
    [TASK_ALTITUDE] = DEFINE_TASK("ALTITUDE", NULL, NULL, taskCalculateAltitude, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
    [TASK_OSD] = DEFINE_TASK("OSD", NULL, NULL, osdUpdate, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
    [TASK_BATTERY_VOLTAGE] = DEFINE_TASK("BATTERY_VOLTAGE", NULL, NULL, batteryUpdateVoltage, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM), // Freq may be updated in tasksInit
//    [TASK_BATTERY_CURRENT] = DEFINE_TASK("BATTERY_CURRENT", NULL, NULL, batteryUpdateCurrentMeter, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM),
    [TASK_LED] = DEFINE_TASK("LED", NULL, NULL, ledUpdate, TASK_PERIOD_HZ(30), TASK_PRIORITY_LOW),
    [TASK_KEY] = DEFINE_TASK("KEY", NULL, NULL, taskKey, TASK_PERIOD_HZ(5), TASK_PRIORITY_LOW),
};


/********************************************************************************
**函数信息 ：tasksInit(void)
**功能描述 ：任务初始化
**输入参数 ：无
**输出参数 ：无
*********************************************************************************/
void tasksInit(void)
{
    schedulerInit();

    setTaskEnabled(TASK_SERIAL, true);

    setTaskEnabled(TASK_GYRO, true);

    setTaskEnabled(TASK_FILTER, true);

    setTaskEnabled(TASK_PID, true);

    setTaskEnabled(TASK_RX, true);

    setTaskEnabled(TASK_ALTITUDE, true);

    setTaskEnabled(TASK_OSD, true);

    setTaskEnabled(TASK_BATTERY_VOLTAGE, true);

    setTaskEnabled(TASK_LED, true);

    setTaskEnabled(TASK_KEY, true);


#if 0
    setTaskEnabled(TASK_PID, true);
    setTaskEnabled(TASK_RX, true);
    setTaskEnabled(TASK_SERIAL, true);
    setTaskEnabled(TASK_FILTER, true);
    setTaskEnabled(TASK_ACCEL, true);
    setTaskEnabled(TASK_ATTITUDE, true);
    setTaskEnabled(TASK_RX, true);
    setTaskEnabled(TASK_BARO, true);
    setTaskEnabled(TASK_ALTITUDE, true);
    setTaskEnabled(TASK_OSD, true);
#endif
}

#endif

#ifdef USE_RX_SBUS_DSM_BY

task_t tasks[TASK_COUNT] =
{
    [TASK_SYSTEM] = DEFINE_TASK("SYSTEM", "LOAD", NULL, taskSystemLoad, TASK_PERIOD_HZ(10), TASK_PRIORITY_MEDIUM_HIGH),
    [TASK_SERIAL] = DEFINE_TASK("SERIAL", NULL, NULL, taskHandleSerial, TASK_PERIOD_HZ(30), TASK_PRIORITY_LOW),
    [TASK_GYRO] = DEFINE_TASK("GYRO", NULL, NULL, taskGyroSample, TASK_PERIOD_HZ(2000), TASK_PRIORITY_REALTIME),
    [TASK_FILTER] = DEFINE_TASK("FILTER", NULL, NULL, taskFiltering, TASK_PERIOD_HZ(1000), TASK_PRIORITY_REALTIME),
    [TASK_PID] = DEFINE_TASK("PID", NULL, NULL, taskMainPidLoop, TASK_PERIOD_HZ(1000), TASK_PRIORITY_REALTIME),
//    [TASK_ACCEL] = DEFINE_TASK("ACC", NULL, NULL, taskUpdateAccelerometer, TASK_PERIOD_HZ(1000), TASK_PRIORITY_MEDIUM),
//    [TASK_ATTITUDE] = DEFINE_TASK("ATTITUDE", NULL, NULL, imuUpdateAttitude, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM),
    [TASK_RX] = DEFINE_TASK("RX", NULL, NULL, taskUpdateRxMain, TASK_PERIOD_HZ(30), TASK_PRIORITY_LOW),
//    [TASK_BARO] = DEFINE_TASK("BARO", NULL, rxUpdateCheck, taskUpdateBaro, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
//    [TASK_ALTITUDE] = DEFINE_TASK("ALTITUDE", NULL, NULL, taskCalculateAltitude, TASK_PERIOD_HZ(40), TASK_PRIORITY_LOW),
    [TASK_OSD] = DEFINE_TASK("OSD", NULL, NULL, osdUpdate, TASK_PERIOD_HZ(20), TASK_PRIORITY_LOW),
    [TASK_BATTERY_VOLTAGE] = DEFINE_TASK("BATTERY_VOLTAGE", NULL, NULL, batteryUpdateVoltage, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM), // Freq may be updated in tasksInit
//    [TASK_BATTERY_CURRENT] = DEFINE_TASK("BATTERY_CURRENT", NULL, NULL, batteryUpdateCurrentMeter, TASK_PERIOD_HZ(50), TASK_PRIORITY_MEDIUM),
    [TASK_LED] = DEFINE_TASK("LED", NULL, NULL, ledUpdate, TASK_PERIOD_HZ(50), TASK_PRIORITY_LOW),
    [TASK_KEY] = DEFINE_TASK("KEY", NULL, NULL, taskKey, TASK_PERIOD_HZ(5), TASK_PRIORITY_LOW),
};


/********************************************************************************
**函数信息 ：tasksInit(void)
**功能描述 ：任务初始化
**输入参数 ：无
**输出参数 ：无
*********************************************************************************/
void tasksInit(void)
{
    schedulerInit();

    setTaskEnabled(TASK_SERIAL, true);

    setTaskEnabled(TASK_GYRO, true);

    setTaskEnabled(TASK_FILTER, true);

    setTaskEnabled(TASK_PID, true);

    setTaskEnabled(TASK_RX, true);

    setTaskEnabled(TASK_OSD, true);

    setTaskEnabled(TASK_BATTERY_VOLTAGE, true);

    setTaskEnabled(TASK_LED, true);

    setTaskEnabled(TASK_KEY, true);
}
#endif

/********************************************************************************
**函数信息 ：getTask(unsigned taskId)
**功能描述 ：返回任务信息
**输入参数 ：taskId 任务ID
**输出参数 ：无
*********************************************************************************/
task_t *getTask(unsigned taskId)
{
    return &tasks[taskId];
}

