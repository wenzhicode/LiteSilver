/**
******************************************************************************
* @file    spl06_001.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   �����ļ���spl06Ӳ����غ���.
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


static struct       //�ڲ�����У׼����
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
    int32_w i32rawPressure;//ԭʼ��ѹ����
    int32_w i32rawTemperature;//ԭʼ�¶�����
    int32_w i32kP;    //��ѹ��������
    int32_w i32kT;//�¶Ȳ�������
} spl06;



//��д����
void spl0601_write(uint8_w REG, uint8_w DATA)
{
    mpu6000_CSH;
    SPIM_CSHigh();
    spl06_CSL;//CS������ΪSPIģʽ
    SPI_RW(REG & 0x7f); //���ͼĴ�����ַ+д����
    SPI_RW(DATA);
    spl06_CSH;
}
uint8_w spl0601_read(uint8_w REG)
{
    uint8_w data;
    mpu6000_CSH;
    SPIM_CSHigh();
    spl06_CSL;//CS������ΪSPIģʽ
    SPI_RW(REG | 0x80); //���ͼĴ�����ַ+������
    data = SPI_RW(0xff);
    spl06_CSH;
    return data;
}

/*

ʲô�ǹ����������ǲ�������ȡһ�����ݣ������¶Ȳ���Ƶ��10HZ�����ù�����Ϊ8��
����ÿ���¶ȶ�ȡ����8��ȡƽ�����ڲ�ʵ������80HZ�����ݲɼ�����̨ģʽ�޹�����


��������������EMPERATURE_RATE_8_TIMES��Ӧ���������ݱ��µ����ݸ��ǣ��ڲ�ӵ����ѹ���¶ȹ�32����FIFO��
�ڴ���8�Σ�Ҳ���Ǵ��ڻ����16�ι���������ʱ����Ҫ���µ����ݸ��ǣ��������ݾͻᶪʧ

*/
void spl06_pressure_rate_config(u8 background_rate, u8 oversamply)
{
    u8 data;

    data = (background_rate << 4) | oversamply;
    if (oversamply > PRESSURE_RATE_8_TIMES)
    {
        u8 data;
        data = spl0601_read(CFG_REG);//��ȡԭ�Ĵ���ֵ
        data |= 0X04;//P-SHIFTλ��1
        spl0601_write(CFG_REG, data);//����д�ؼĴ���
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
    spl0601_write(PRS_CFG_REG, data); //д������
}


// �¶Ȳ���������
void spl06_temperature_rate_config(u8 background_rate, u8 oversamply, u8 ext)
{
    u8 data;

    data = (ext << 7) | (background_rate << 4) | oversamply;
    if (oversamply > TEMPERATURE_RATE_8_TIMES)
    {
        u8 data;
        data = spl0601_read(CFG_REG); //��ȡԭ�Ĵ���ֵ
        data |= 0X08;//T-SHIFTλ��1
        spl0601_write(CFG_REG, data);    //����д�ؼĴ���
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
    spl0601_write(TMP_CFG_REG, data); //д������
}



//��ȡ���������ݾ�λ״̬//����������״̬
u8 spl06_get_measure_status(void)
{
    return spl0601_read(MEAS_CFG_REG);
}


//���ö�ȡģʽ+��ȡ��ʽ
//����Ϊģʽֵ
void spl06_set_measure_mode(u8 mode)
{
    spl0601_write(MEAS_CFG_REG, mode);
}


//��ȡ��ƷID//��ȡ��Ʒ�汾//���ڰ汾�ڲ�ͬ�Ĵ������в�ͬ��������ֻ�ж�ID��ʶ��SPL06
u8 spl06_get_chip_id(void)
{
    return spl0601_read(ID_REG);
}

//�ڲ�У׼ֵ//��ѹ�ƽ����Լ��²�ʹ��//���ڲ������趨
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
 * ��ʼ��
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
u8 spl0601_init(void)
{
    u8 spl06_start_status;
    //�ȴ��ڲ�У׼���ݿ���
    do
        spl06_start_status = spl06_get_measure_status();//��ȡ��ѹ������״̬
    while ((spl06_start_status & MEAS_CFG_COEF_RDY) != MEAS_CFG_COEF_RDY);
    //��ȡ�ڲ�У׼ֵ
    spl0601_get_calib_param();
    //�ȴ��������ڲ���ʼ�����
    do
        spl06_start_status = spl06_get_measure_status();//��ȡ��ѹ������״̬
    while ((spl06_start_status & MEAS_CFG_SENSOR_RDY) != MEAS_CFG_SENSOR_RDY);
    //��ȡCHIP ID
    spl06.chip_id = spl06_get_chip_id();
    //�ж϶�ȡ��ID�Ƿ���ȷ������ֻ�жϸ�4λ��ID�����жϵ�4λ�İ汾��
    if ((spl06.chip_id & 0xf0) != PRODUCT_ID)
        return FAILED;//���ID��ȡʧ�ܣ��򷵻�ʧ��
    //��̨���ݲ�������128HZ ��������32��
    spl06_pressure_rate_config(PRESSURE_RATE_128_TIMES, PRESSURE_RATE_32_TIMES);
    //��̨���ݲ�������32HZ ��������8��//���ô������ϵ��¶ȼ���Ϊ�¶Ȳɼ�
    spl06_temperature_rate_config(TEMPERATURE_RATE_32_TIMES, TEMPERATURE_RATE_8_TIMES, TEMPERATURE_RATE_TMP_EXT_EXTERNAL);
    //������̨��ȡ����
    spl06_set_measure_mode(MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP);
    return SUCCESS;//��ʼ���ɹ�
}


/***********************************************************************
 * ��ȡԭʼ�¶�ֵ
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
 * ��ȡԭʼ��ѹֵ
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
 * �¶Ƚ���ֵ
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
 * ��ѹ���㲢�����¶Ȳ���
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
 * ��ȡ�¶�ֵ
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float user_spl0601_get_temperature()
{
    spl0601_get_raw_temp();//��ȡ�¶�ԭʼֵ
    return spl0601_get_temperature();//�¶Ƚ�����ֵ
}
/***********************************************************************
 * ��ȡ��ѹ���¶Ȳ���ֵ
 * @param[in]
 * @param[out]
 * @return
 **********************************************************************/
float user_spl0601_get_presure()
{
    spl0601_get_raw_temp();//��ȡ�¶�ԭʼֵ

    spl0601_get_raw_pressure();//��ȡ��ѹֵԭʼֵ
    return spl0601_get_pressure();  //��ѹ���㲢�����¶Ȳ��������ѹֵ
}

