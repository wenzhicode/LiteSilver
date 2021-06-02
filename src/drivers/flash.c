/**
******************************************************************************
* @file    flash.c
* @author
* @version V0.0.1
* @date    30/06/2020
* @brief   驱动文件，flash相关函数.
******************************************************************************
*/




#include "flash.h"
#include "flash_io.h"
#include "cc2500.h"
#include "stdlib.h"
#include "string.h"


extern uint8_t acc_checked;
extern float accelcal[3];
extern float *pids_array[3];
extern char motorDir[4];
extern unsigned char dshot_select;
extern unsigned char osd_mode;
extern unsigned int ratesValueRoll;
extern unsigned int ratesValuePitch;
extern unsigned int ratesValueYaw ;
extern unsigned char rx_select;
extern unsigned char motor_min;
extern u32 old_freq;
extern unsigned char rx_show;
extern unsigned char mode_show;
extern unsigned char vol_show;
extern uint8_t rgb_led;
extern unsigned char flymode;
extern char rfchannel[4];
extern char rxaddress[5];
extern int telemetry_enabled;
extern int rx_bind_load;
extern int rx_bind_enable;
extern unsigned char lipo_last;
extern unsigned char IsApp;
extern float lowvol;
extern uint8_t max_angle;


extern float rcRate[3];
extern float superExpo[3];
extern float Expo[3];


extern float hardcoded_pid_identifier;
float initial_pid_identifier = -10;
float saved_pid_identifier;



float flash_get_hard_coded_pid_identifier(void)
{
    float result = 0;

    for (int i = 0;  i < 3 ; i++)
    {
        for (int j = 0; j < 3 ; j++)
        {
            result += pids_array[i][j] * (i + 1) * (j + 1) * 0.932f;
        }
    }
    return result;
}


void flash_hard_coded_pid_identifier(void)
{
    initial_pid_identifier = flash_get_hard_coded_pid_identifier();
}


/**************************************************************************
**函数信息 ：flash_save()
**功能描述 ：存储变量
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void flash_save()
{
    FLASH_Unlock();
    fmc_erase();

    unsigned long addresscount = 0;
    writeword(addresscount++, FMC_HEADER);

//    writeword(addresscount++, IsApp);

    //cc2500
    unsigned long *p = (unsigned long *)&rxCc2500SpiConfigMutable;
    for (int i = 0; i < 15; i++)
    {
        writeword(addresscount++, (unsigned long)(*p++));

    }

    //pid
    fmc_write_float(addresscount++, initial_pid_identifier);
    for (int i = 0;  i < 3 ; i++)
    {
        for (int j = 0; j < 3 ; j++)
        {
            fmc_write_float(addresscount++, pids_array[i][j]);
        }
    }

    //acc cali
    writeword(addresscount++, acc_checked);
    fmc_write_float(addresscount++, accelcal[0]);
    fmc_write_float(addresscount++, accelcal[1]);
    fmc_write_float(addresscount++, accelcal[2]);

    fmc_write_float(addresscount++, rcRate[0]);
    fmc_write_float(addresscount++, rcRate[1]);
    fmc_write_float(addresscount++, rcRate[2]);
    
    fmc_write_float(addresscount++, superExpo[0]);
    fmc_write_float(addresscount++, superExpo[1]);
    fmc_write_float(addresscount++, superExpo[2]);
    
    fmc_write_float(addresscount++, Expo[0]);
    fmc_write_float(addresscount++, Expo[1]);
    fmc_write_float(addresscount++, Expo[2]);

    //motor dshot
    writeword(addresscount++, (motorDir[0] | motorDir[1] << 8 | motorDir[2] << 16 | motorDir[3] << 24));
    writeword(addresscount++, dshot_select);


    //rates
    writeword(addresscount++, ratesValueRoll);
    writeword(addresscount++, ratesValuePitch);
    writeword(addresscount++, ratesValueYaw);


    //osd mode
    writeword(addresscount++, osd_mode);

    //rx_select
    writeword(addresscount++, rx_select);

    //vtx_index
    writeword(addresscount++, old_freq);

    //motor_min
    writeword(addresscount++, motor_min);

    //rx_show
    writeword(addresscount++, rx_show);

    //mode_show
    writeword(addresscount++, mode_show);

    //vol_show
    writeword(addresscount++, vol_show);

    //rgb_led
    writeword(addresscount++, rgb_led);

    //flymode
    writeword(addresscount++, flymode);

    //lipo_last
    writeword(addresscount++, lipo_last);

    //bayang
    if ( rx_bind_enable )
    {
        writeword(80, rxaddress[4]|telemetry_enabled<<8);
        writeword(81, rxaddress[0]|(rxaddress[1]<<8)|(rxaddress[2]<<16)|(rxaddress[3]<<24));
        writeword(82, rfchannel[0]|(rfchannel[1]<<8)|(rfchannel[2]<<16)|(rfchannel[3]<<24));
    }
    else
    {
        // this will leave 255's so it will be picked up as disabled
    }

//    //lowvol
//    writeword(addresscount++, lowvol);
//
//    //max_angle
//    writeword(addresscount++, max_angle);

    writeword(255, FMC_HEADER);

    FLASH_Lock();
}



/**************************************************************************
**函数信息 ：flash_load()
**功能描述 ：加载变量
**输入参数 ：无
**输出参数 ：无

头尾有一个简单的校验

**************************************************************************/
void flash_load()
{
    unsigned long addresscount = 0;

    flash_hard_coded_pid_identifier();

    if (FMC_HEADER == fmc_read(addresscount++) && FMC_HEADER == fmc_read(255))
    {
//        IsApp = fmc_read(addresscount++);
//
//        if(IsApp > 1)
//            IsApp = 0;

        //cc2500
        unsigned long temp;
        unsigned long *p = (unsigned long *)&rxCc2500SpiConfigMutable;
        for (int i = 0; i < 15; i++)
        {
            temp = fmc_read(addresscount++);
            memcpy(p++, &temp, 4);
        }

        //pid
        saved_pid_identifier = fmc_read_float(addresscount++);
        if (saved_pid_identifier == initial_pid_identifier)
        {
            for (int i = 0;  i < 3 ; i++)
            {
                for (int j = 0; j < 3 ; j++)
                {
                    pids_array[i][j] = fmc_read_float(addresscount++);
                }
            }
        }
        else
        {
            addresscount += 9;
        }

        //acc cali
        acc_checked = fmc_read(addresscount++);
        if (acc_checked > 1)
        {
            acc_checked = 0;
        }
        accelcal[0] = fmc_read_float(addresscount++);
        accelcal[1] = fmc_read_float(addresscount++);
        accelcal[2] = fmc_read_float(addresscount++);
        
        /*
        extern float rcRate[3];
        extern float superExpo[3];
        extern float Expo[3];       
        */

        rcRate[0] = fmc_read_float(addresscount++);
        rcRate[1] = fmc_read_float(addresscount++);
        rcRate[2] = fmc_read_float(addresscount++);
        
        
        superExpo[0] = fmc_read_float(addresscount++);
        superExpo[1] = fmc_read_float(addresscount++);
        superExpo[2] = fmc_read_float(addresscount++);
        
        Expo[0] = fmc_read_float(addresscount++);
        Expo[1] = fmc_read_float(addresscount++);
        Expo[2] = fmc_read_float(addresscount++);
        
        for(int i=0;i<3;i++)
        {
            if(rcRate[i] > 2.55f)
            {
                rcRate[i] = 1.0f;
            }
            
            if(superExpo[i] > 1.0f)
            {
                superExpo[i] = 0.0f;
            }
            
            if(Expo[i] > 1.0f)
            {
                Expo[i] = 0.0f;
            }
            
        }
                
        // motor dshot
        temp = fmc_read(addresscount++);
        motorDir[0] = temp & 0x0F;
        motorDir[1] = (temp >> 8) & 0x0F;
        motorDir[2] = (temp >> 16) & 0x0F;
        motorDir[3] = (temp >> 24) & 0x0F;

        for (int i = 0; i < 4; i++)
        {
            if (motorDir[i] > 1)
            {
                motorDir[0] = 0;
                motorDir[1] = 1;
                motorDir[2] = 1;
                motorDir[3] = 0;
            }
        }

        temp = fmc_read(addresscount++);
        dshot_select = temp & 0x0F;
        if (dshot_select > 2)
            dshot_select = 2;

        //rates
        ratesValueRoll = fmc_read(addresscount++);
        if (ratesValueRoll > 2000)
            ratesValueRoll = 500;

        ratesValuePitch = fmc_read(addresscount++);
        if (ratesValuePitch > 2000)
            ratesValuePitch = 500;

        ratesValueYaw = fmc_read(addresscount++);
        if (ratesValueYaw > 1000)
            ratesValueYaw = 400;

        //osd mode
        temp = fmc_read(addresscount++);
        osd_mode = temp & 0x0F;
        if (osd_mode > 1)
            osd_mode = 1;

        //rx_select
        temp = fmc_read(addresscount++);
        rx_select = temp & 0x0F;
        if (rx_select > 5)
            rx_select = 0;

        //vtx_index
        temp = fmc_read(addresscount++);
        old_freq = temp & 0xFFFF;
        if (old_freq > 5945)
            old_freq = 5733;

        //motor_min
        temp = fmc_read(addresscount++);
        motor_min = temp & 0xFF;
        if (motor_min > 20)
            motor_min = 5;

        //rx_show
        temp = fmc_read(addresscount++);
        rx_show = temp & 0x0F;
        if (rx_show > 2)
            rx_show = 1;

        //mode_show
        temp = fmc_read(addresscount++);
        mode_show = temp & 0x0F;
        if (mode_show > 2)
            mode_show = 1;

        //vol_show
        temp = fmc_read(addresscount++);
        vol_show = temp & 0x0F;
        if (vol_show > 2)
            vol_show = 1;

        //rgb_led
        temp = fmc_read(addresscount++);
        rgb_led = temp & 0x0F;
        if (vol_show > 1)
            vol_show = 1;

        //flymode
        temp = fmc_read(addresscount++);
        flymode = temp & 0xFF;
        if (flymode > 10)
            flymode = 0;

        //lipo_last
        temp = fmc_read(addresscount++);
        lipo_last = temp & 0x0F;
        if (lipo_last > 2)
            lipo_last = 0;

        //bayang
        temp = fmc_read(82);
        int error = 0;
        for ( int i = 0 ; i < 4; i++)
        {
            if ( ((temp>>(i*8))&0xff  ) > 127)
            {
                error = 1;
            }
        }

        if( !error )
        {
            rx_bind_load = rx_bind_enable = 1;

            rxaddress[4] = fmc_read(80);

            telemetry_enabled = fmc_read(80)>>8;
            int temp = fmc_read(81);
            for ( int i = 0 ; i < 4; i++)
            {
                rxaddress[i] =  temp>>(i*8);
            }

            temp = fmc_read(82);
            for ( int i = 0 ; i < 4; i++)
            {
                rfchannel[i] =  temp>>(i*8);
            }
        }

        //lowvol
//        lowvol = fmc_read(addresscount++);
//        if (lowvol > 4.3)
//            lowvol = 3.3;
//
//        //max_angle
//        temp = fmc_read(addresscount++);
//        max_angle = temp & 0xFF;
//        if (max_angle > 90)
//            max_angle = 65;
    }
}




