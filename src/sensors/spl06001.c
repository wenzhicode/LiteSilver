/**
******************************************************************************
* @file    spl06_001.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   驱动文件，spl06硬件相关函数.
******************************************************************************
*/


#include "spl06001.h"
#include "bus_spi.h"
#include "time.h"
#include "defines.h"


#define s32 int32_w
#define int16_w short
#define int32_w int
#define uint8_w unsigned char


unsigned int spl06time;
unsigned int lastspl06time;


static struct       //内部出厂校准数据
{
    int16_w c0;
    int16_w c1;
    int32_w c00;
    int32_w c10;
    int16_w c01;
    int16_w c11;
    int16_w c20;
    int16_w c21;
    int16_w c30;
} spl06_calib_param;


struct
{
    uint8_w chip_id; /**<chip id*/
    int32_w i32rawPressure;//原始气压数据
    int32_w i32rawTemperature;//原始温度数据
    int32_w i32kP;    //气压补偿参数
    int32_w i32kT;//温度补偿参数
} spl06;



//读写配置
void spl0601_write(uint8_w REG, uint8_w DATA)
{
    mpu6000_CSH;
    SPIM_CSHigh();
    spl06_CSL;//CS拉低则为SPI模式
    SPI_RW(REG & 0x7f); //发送寄存器地址+写命令
    SPI_RW(DATA);
    spl06_CSH;
}
uint8_w spl0601_read(uint8_w REG)
{
    uint8_w data;
    mpu6000_CSH;
    SPIM_CSHigh();
    spl06_CSL;//CS拉低则为SPI模式
    SPI_RW(REG | 0x80); //发送寄存器地址+读命令
    data = SPI_RW(0xff);
    spl06_CSH;
    return data;
}

/*

什么是过采样，就是采样多组取一次数据，比如温度采样频率10HZ，设置过采样为8，
就是每个温度读取都是8组取平均，内部实际做了80HZ的数据采集，后台模式无过采样


过采样次数大于EMPERATURE_RATE_8_TIMES，应当允许数据被新的数据覆盖，内部拥有气压和温度共32级的FIFO，
在大于8次（也就是大于或等于16次过采样）的时候需要被新的数据覆盖，否则数据就会丢失

*/
void spl06_pressure_rate_config(u8 background_rate, u8 oversamply)
{
    u8 data;

    data = (background_rate << 4) | oversamply;
    if (oversamply > PRESSURE_RATE_8_TIMES)
    {
        u8 data;
        data = spl0601_read(CFG_REG);//读取原寄存器值
        data |= 0X04;//P-SHIFT位置1
        spl0601_write(CFG_REG, data);//重新写回寄存器
    }
    switch (oversamply)
    {
    case PRESSURE_RATE_2_TIMES:
        spl06.i32kP = 1572864;
        break;
    case PRESSURE_RATE_4_TIMES:
        spl06.i32kP  = 3670016;
        break;
    case PRESSURE_RATE_8_TIMES:
        spl06.i32kP  = 7864320;
        break;
    case PRESSURE_RATE_16_TIMES:
        spl06.i32kP = 253952;
        break;
    case PRESSURE_RATE_32_TIMES:
        spl06.i32kP = 516096;
        break;
    case PRESSURE_RATE_64_TIMES:
        spl06.i32kP = 1040384;
        break;
    case PRESSURE_RATE_128_TIMES:
        spl06.i32kP = 2088960;
        break;
    case PRESSURE_RATE_1_TIMES:
    default:
        spl06.i32kP = 524288;
        break;
    }
    spl0601_write(PRS_CFG_REG, data); //写入配置
}


// 温度采样率配置
void spl06_temperature_rate_config(u8 background_rate, u8 oversamply, u8 ext)
{
    u8 data;

    data = (ext << 7) | (background_rate << 4) | oversamply;
    if (oversamply > TEMPERATURE_RATE_8_TIMES)
    {
        u8 data;
        data = spl0601_read(CFG_REG); //读取原寄存器值
        data |= 0X08;//T-SHIFT位置1
        spl0601_write(CFG_REG, data);    //重新写回寄存器
    }
    switch (oversamply)
    {
    case TEMPERATURE_RATE_2_TIMES:
        spl06.i32kT = 1572864;
        break;
    case TEMPERATURE_RATE_4_TIMES:
        spl06.i32kT  = 3670016;
        break;
    case TEMPERATURE_RATE_8_TIMES:
        spl06.i32kT  = 7864320;
        break;
    case TEMPERATURE_RATE_16_TIMES:
        spl06.i32kT = 253952;
        break;
    case TEMPERATURE_RATE_32_TIMES:
        spl06.i32kT = 516096;
        break;
    case TEMPERATURE_RATE_64_TIMES:
        spl06.i32kT = 1040384;
        break;
    case TEMPERATURE_RATE_128_TIMES:
        spl06.i32kT = 2088960;
        break;
    case TEMPERATURE_RATE_1_TIMES:
    default:
        spl06.i32kT = 524288;
        break;
    }
    spl0601_write(TMP_CFG_REG, data); //写入配置
}



//获取传感器数据就位状态//传感器就绪状态
u8 spl06_get_measure_status(void)
{
    return spl0601_read(MEAS_CFG_REG);
}


//设置读取模式+读取方式
//参数为模式值
void spl06_set_measure_mode(u8 mode)
{
    spl0601_write(MEAS_CFG_REG, mode);
}


//获取产品ID//获取产品版本//由于版本在不同的传感器有不同，本历程只判断ID来识别SPL06
u8 spl06_get_chip_id(void)
{
    return spl0601_read(ID_REG);
}

//内部校准值//气压计解算以及温补使用//由内部出厂设定
void spl0601_get_calib_param(void)
{
    unsigned long h;
    unsigned long m;
    unsigned long l;
    h =  spl0601_read(0x10);
    l  =  spl0601_read(0x11);
    spl06_calib_param.c0 = (int16_w)h << 4 | l >> 4;
    spl06_calib_param.c0 = (spl06_calib_param.c0 & 0x0800) ? (0xF000 | spl06_calib_param.c0) : spl06_calib_param.c0;
    h =  spl0601_read(0x11);
    l  =  spl0601_read(0x12);
    spl06_calib_param.c1 = (int16_w)(h & 0x0F) << 8 | l;
    spl06_calib_param.c1 = (spl06_calib_param.c1 & 0x0800) ? (0xF000 | spl06_calib_param.c1) : spl06_calib_param.c1;
    h =  spl0601_read(0x13);
    m =  spl0601_read(0x14);
    l =  spl0601_read(0x15);
    spl06_calib_param.c00 = (int32_w)h << 12 | (int32_w)m << 4 | (int32_w)l >> 4;
    spl06_calib_param.c00 = (spl06_calib_param.c00 & 0x080000) ? (0xFFF00000 | spl06_calib_param.c00) : spl06_calib_param.c00;
    h =  spl0601_read(0x15);
    m =  spl0601_read(0x16);
    l =  spl0601_read(0x17);
    spl06_calib_param.c10 = (int32_w)h << 16 | (int32_w)m << 8 | l;
    spl06_calib_param.c10 = (spl06_calib_param.c10 & 0x080000) ? (0xFFF00000 | spl06_calib_param.c10) : spl06_calib_param.c10;
    h =  spl0601_read(0x18);
    l  =  spl0601_read(0x19);
    spl06_calib_param.c01 = (int16_w)h << 8 | l;
    h =  spl0601_read(0x1A);
    l  =  spl0601_read(0x1B);
    spl06_calib_param.c11 = (int16_w)h << 8 | l;
    h =  spl0601_read(0x1C);
    l  =  spl0601_read(0x1D);
    spl06_calib_param.c20 = (int16_w)h << 8 | l;
    h =  spl0601_read(0x1E);
    l  =  spl0601_read(0x1F);
    spl06_calib_param.c21 = (int16_w)h << 8 | l;
    h =  spl0601_read(0x20);
    l  =  spl0601_read(0x21);
    spl06_calib_param.c30 = (int16_w)h << 8 | l;
}


/***********************************************************************
 * 初始化
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
u8 spl0601_init(void)
{
    u8 spl06_start_status;
    //等待内部校准数据可用
    do
        spl06_start_status = spl06_get_measure_status();//读取气压计启动状态
    while ((spl06_start_status & MEAS_CFG_COEF_RDY) != MEAS_CFG_COEF_RDY);
    //读取内部校准值
    spl0601_get_calib_param();
    //等待传感器内部初始化完成
    do
        spl06_start_status = spl06_get_measure_status();//读取气压计启动状态
    while ((spl06_start_status & MEAS_CFG_SENSOR_RDY) != MEAS_CFG_SENSOR_RDY);
    //读取CHIP ID
    spl06.chip_id = spl06_get_chip_id();
    //判断读取的ID是否正确，这里只判断高4位的ID，不判断低4位的版本号
    if ((spl06.chip_id & 0xf0) != PRODUCT_ID)
        return FAILED;//如果ID读取失败，则返回失败
    //后台数据采样速率128HZ 过采样率32次
    spl06_pressure_rate_config(PRESSURE_RATE_128_TIMES, PRESSURE_RATE_32_TIMES);
    //后台数据采样速率32HZ 过采样率8次//设置传感器上的温度计作为温度采集
    spl06_temperature_rate_config(TEMPERATURE_RATE_32_TIMES, TEMPERATURE_RATE_8_TIMES, TEMPERATURE_RATE_TMP_EXT_EXTERNAL);
    //启动后台读取数据
    spl06_set_measure_mode(MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP);
    return SUCCESS;//初始化成功
}


/***********************************************************************
 * 获取原始温度值
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
void spl0601_get_raw_temp(void)
{
    uint8_w h[3] = {0};

    h[0] = spl0601_read(0x03);
    h[1] = spl0601_read(0x04);
    h[2] = spl0601_read(0x05);

    spl06.i32rawTemperature = (int32_w)h[0] << 16 | (int32_w)h[1] << 8 | (int32_w)h[2];
    spl06.i32rawTemperature = (spl06.i32rawTemperature & 0x800000) ? (0xFF000000 | spl06.i32rawTemperature) : spl06.i32rawTemperature;

}

/***********************************************************************
 * 获取原始气压值
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
void spl0601_get_raw_pressure(void)
{
    uint8_w h[3];

    h[0] = spl0601_read(0x00);
    h[1] = spl0601_read(0x01);
    h[2] = spl0601_read(0x02);

    spl06.i32rawPressure = (int32_w)h[0] << 16 | (int32_w)h[1] << 8 | (int32_w)h[2];
    spl06.i32rawPressure = (spl06.i32rawPressure & 0x800000) ? (0xFF000000 | spl06.i32rawPressure) : spl06.i32rawPressure;

}


/***********************************************************************
 * 温度解算值
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = spl06.i32rawTemperature / (float)spl06.i32kT;
    fTCompensate =  spl06_calib_param.c0 * 0.5 + spl06_calib_param.c1 * fTsc;
    return fTCompensate;
}

/***********************************************************************
 * 气压解算并进行温度补偿
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float spl0601_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    unsigned long time = gettime();
    spl06time = ((uint32_t)(time - lastspl06time));

    lastspl06time = time;


    fTsc = spl06.i32rawTemperature / (float)spl06.i32kT;
    fPsc = spl06.i32rawPressure / (float)spl06.i32kP;
    qua2 = spl06_calib_param.c10 + fPsc * (spl06_calib_param.c20 + fPsc * spl06_calib_param.c30);
    qua3 = fTsc * fPsc * (spl06_calib_param.c11 + fPsc * spl06_calib_param.c21);
    //qua3 = 0.9f *fTsc * fPsc * (spl06_calib_param.c11 + fPsc * spl06_calib_param.c21);

    fPCompensate = spl06_calib_param.c00 + fPsc * qua2 + fTsc * spl06_calib_param.c01 + qua3;
    //fPCompensate = spl06_calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06_calib_param.c01 + qua3;
    return fPCompensate;
}



/***********************************************************************
 * 获取温度值
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float user_spl0601_get_temperature()
{
    spl0601_get_raw_temp();//读取温度原始值
    return spl0601_get_temperature();//温度解算后的值
}
/***********************************************************************
 * 获取气压计温度补偿值
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float user_spl0601_get_presure()
{
    spl0601_get_raw_temp();//读取温度原始值

    spl0601_get_raw_pressure();//读取气压值原始值
    return spl0601_get_pressure();  //气压解算并经过温度补偿后的气压值
}

