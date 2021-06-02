/**
******************************************************************************
* @file    mpu6500.c
* @author
* @version V0.0.1
* @date    12/06/2020
* @brief   驱动文件，imu硬件相关函数.
******************************************************************************
*/



#include "mpu6500.h"
#include "math.h"
#include "util.h"
#include "defines.h"
#include "IIR_filter.h"


float accel[3];
float accel_one[3];
float gyro[3];
float gyronew[3];
float accelraw[3];
int16 accEf[3];
float accelcal[3];
float gyrocal[3];

u8 mpu6000_id;

int16_t acclN[3]; 
int16_t gyroN[3]; 
int8_t temper=0;


float lpffilter(float in, int num);
float lpffilter2(float in, int num);

/********************************************************************************
**函数信息 ：mpu6500_readRegs(u8 reg, u8 length, u8 *data)
**功能描述 ：mpu6500读多个寄存器
**输入参数 ：reg 寄存器 length 个数  data 数据
**输出参数 ：无
*********************************************************************************/
void mpu6500_readRegs(u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
    SPIM_CSHigh();
    spl06_CSH;
    mpu6000_CSL;
    SPI_RW(reg | 0x80);
    for (count = 0; count < length; count++)
    {
        data[count] = SPI_RW(0xff);
    }
    mpu6000_CSH;
}


/********************************************************************************
**函数信息 ：
**功能描述 ：mpu6500读写寄存器
**输入参数 ：reg 寄存器
**输出参数 ：无
*********************************************************************************/
static u8 mpu6500_readReg(u8 reg)
{
    u8 data;
    SPIM_CSHigh();
    spl06_CSH;
    mpu6000_CSL;//CS拉低则为SPI模式
    SPI_RW(reg | 0x80); //发送寄存器地址+读命令
    data = SPI_RW(0xff);
    mpu6000_CSH;
    return data;
}

static void mpu6500_writeReg(u8 REG, u8 DATA)
{
    SPIM_CSHigh();
    spl06_CSH;
    mpu6000_CSL;//CS拉低则为SPI模式
    SPI_RW(REG & 0x7f); //发送寄存器地址+写命令
    SPI_RW(DATA);
    mpu6000_CSH;
}

u8 mpu6500_readID(void)
{
    return mpu6500_readReg(MPU_RA_WHO_AM_I);
}


/********************************************************************************
**函数信息 ：mpu6500_initialize(void)
**功能描述 ：mpu6500初始化
**输入参数 ：无
**输出参数 ：无
*********************************************************************************/
void mpu6500_init(void)
{
    uint32_t tmpreg = 0;
    tmpreg = SPI1->SPBRG;
    /* Clear spbrg bits */
    tmpreg &= (uint16_t)0x0000;
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    tmpreg |= (uint16_t) 96;
    /* Write to SPIx SPBRG */
    SPI1->SPBRG = tmpreg;
    delay_ms(100);

    mpu6500_writeReg(MPU_RA_PWR_MGMT_1, BIT_H_RESET); //Reset ing
    delay_ms(150);

    mpu6500_writeReg(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay_ms(150);

    mpu6500_writeReg(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ); //Internal 20MHz oscillator
    delay_ms(1);

    mpu6500_writeReg(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delay_ms(1);

    mpu6500_writeReg(MPU_RA_PWR_MGMT_2, 0x00);
    delay_ms(1);

    mpu6500_writeReg(MPU_RA_ACCEL_CONFIG, INV_FSR_16G << 3);//加速度度最大量程 +-2G
    delay_ms(1);

    mpu6500_writeReg(MPU_RA_SMPLRT_DIV, 0);   //带宽91
    delay_ms(1);

    mpu6500_writeReg(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3); //陀螺仪最大量程 +-2000度每秒
    delay_ms(1);

    mpu6500_writeReg(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);
    delay_ms(1);

    mpu6000_id = mpu6500_readID();

    tmpreg = SPI1->SPBRG;
    /* Clear spbrg bits */
    tmpreg &= (uint16_t)0x0000;
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    tmpreg |= (uint16_t) 8;
    /* Write to SPIx SPBRG */
    SPI1->SPBRG = tmpreg;
    delay_ms(100);
}


/**************************************************************************
**函数信息 ：mpu6500_readGyro(void)
**功能描述 ：mpu6500读取数据
**输入参数 ：无
**输出参数 ：无

这里只是读取每个轴的数据，飞控板上需要对坐标轴进行对齐才能得到正确的数据

**************************************************************************/
void mpu6500_readGyro(void)
{
    uint8_t buf[16];

    mpu6500_readRegs(MPU_RA_ACCEL_XOUT_H, 14, buf);

    accelraw[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accelraw[1] = -(int16_t)((buf[2] << 8) | buf[3]);
    accelraw[2] = - (int16_t)((buf[4] << 8) | buf[5]);

    accelraw[0] =accelraw[0] - accelcal[0];
    accelraw[1] =accelraw[1] - accelcal[1];
    accelraw[2] =accelraw[2] - accelcal[2];

     temper=((int16_t)(buf[6]<<8)+buf[7])/326.8f + 25.0f;
    
    //gyro
    gyronew[1] = (int16_t)((buf[8] << 8) | buf[9]);
    gyronew[0] = (int16_t)((buf[10] << 8) | buf[11]);
    gyronew[2] = (int16_t)((buf[12] << 8) | buf[13]);


    gyronew[0] = gyronew[0] - gyrocal[0];
    gyronew[1] = gyronew[1] - gyrocal[1];
    gyronew[2] = gyronew[2] - gyrocal[2];

    for (int i = 0; i < 3; i++)
    {
        acclN[i] = accelraw[i];   
        accel[i] = accelraw[i] * (1 / 2048.0f);
        
        gyroN[i] = gyronew[i];
        gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
        gyro[i] = lpffilter(gyronew[i], i);
        gyro[i] = lpffilter2(gyro[i], i);
    }
}



/**************************************************************************
**函数信息 ：mpu6500_readAcc(void)
**功能描述 ：mpu6500读取数据
**输入参数 ：无
**输出参数 ：无

这里只是读取每个轴的数据，飞控板上需要对坐标轴进行对齐才能得到正确的数据

**************************************************************************/
void mpu6500_readAcc(void)
{
    uint8_t buf[6];
    float accelraw[3];

    mpu6500_readRegs(MPU_RA_ACCEL_XOUT_H, 6, buf);


    accelraw[0] = (int16_t)((buf[0] << 8) + buf[1]);
    accelraw[1] = -(int16_t)((buf[2] << 8) + buf[3]);
    accelraw[2] = -(int16_t)((buf[4] << 8) + buf[5]);

    accelraw[0] -= accelcal[0];
    accelraw[1] -= accelcal[1];
    accelraw[2] -= accelcal[2];

    for (int i = 0; i < 3; i++)
    {
        accel[i] = accelraw[i] * (1 / 2048.0f);
    }
}

#define CAL_TIME 2e6

void gyro_cal(void)
{
    u8 data[6];
    float limit[3];
    unsigned long time = gettime();
    unsigned long timestart = time;
    unsigned long timemax = time;
    unsigned long lastlooptime = time;

    float gyro[3];

    for (int i = 0 ; i < 3 ; i++)
    {
        limit[i] = gyrocal[i];
    }
    while (time - timestart < CAL_TIME  &&  time - timemax < 15e6)
    {
        unsigned long looptime;
        looptime = time - lastlooptime;
        lastlooptime = time;
        if (looptime == 0) looptime = 1;

        mpu6500_readRegs(MPU_RA_GYRO_XOUT_H, 6, data);

        gyro[1] = (int16_t)((data[0] << 8) + data[1]);
        gyro[0] = (int16_t)((data[2] << 8) + data[3]);
        gyro[2] = (int16_t)((data[4] << 8) + data[5]);

        for (int i = 0 ; i < 3 ; i++)
        {
            if (gyro[i] > limit[i])  limit[i] += 0.1f;   // 100 gyro bias / second change
            if (gyro[i] < limit[i])  limit[i] -= 0.1f;

            limitf(&limit[i], 800);

            if (fabsf(gyro[i]) > 100 + fabsf(limit[i]))
            {
                timestart = gettime();
            }
            else
            {
                lpf(&gyrocal[i], gyro[i], lpfcalc((float) looptime, 0.5 * 1e6));
            }
        }
        while ((gettime() - time) < 1000) delay(10);
        time = gettime();
    }

    if (time - timestart < CAL_TIME)
    {
        for (int i = 0 ; i < 3; i++)
        {
            gyrocal[i] = 0;
        }
    }
}


void acc_cal(void)
{
    float accelraw[3];
    accelcal[2] = 2048;

    for (int y = 0; y < 500; y++)
    {
        static u8 buf[6];

        mpu6500_readRegs(MPU_RA_ACCEL_XOUT_H, 6, buf);


        accelraw[0] = (int16_t)((buf[0] << 8) + buf[1]);
        accelraw[1] = -(int16_t)((buf[2] << 8) + buf[3]);
        accelraw[2] = -(int16_t)((buf[4] << 8) + buf[5]);


        for (int x = 0; x < 3; x++)
        {
            lpf(&accelcal[x], accelraw[x], 0.92);
        }
        gettime();  // if it takes too long time will overflow so we call it here
    }
    accelcal[2] -= 2048;

    for (int x = 0; x < 3; x++)
    {
        limitf(&accelcal[x], 500);
    }
}




