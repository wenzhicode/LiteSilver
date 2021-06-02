/**
******************************************************************************
* @file    osd.c
* @author
* @version V0.0.1
* @date    14/06/2020
* @brief   osd文件，osd相关函数.
******************************************************************************


*/

#include "osd.h"
#include "item.h"
#include "defines.h"
#include "serial_uart.h"
#include "math.h"
#include <stdlib.h>
#include "bus_softspi.h"
#include "flash.h"
#include "mpu6500.h"
#include "cc2500.h"

#define Roll     0
#define Pitch    1
#define Yaw      2
#define Throttle 3

extern float rx[4];

menu_list setMenu, setMenuHead;
menu_list pidMenu, pidMenuHead;
menu_list motorMenu, motorMenuHead;
menu_list configMenu, configMenuHead;
menu_list receiverMenu, receiverMenuHead;
menu_list smartaudioMenu, smartaudioMenuHead;
menu_list currentMenu;

unsigned char vtx_index = 0;
unsigned char band_index = 0;
u32 vtx_freq = 0;
u32 old_freq = 0;

unsigned char motor_change = 0;
unsigned char vol_show = 1;
unsigned char rx_show = 1;
unsigned char mode_show = 1;
unsigned char motor_min = 3;
unsigned char dshot_select = 2;
unsigned char motor_select = 1;
unsigned char rx_select = 2;
unsigned char motor_index = 0;
unsigned char osd_mode = 0;
unsigned int ratesValueRoll = 600;
unsigned int ratesValuePitch = 600;
unsigned int ratesValueYaw = 400;
unsigned char low_battery = 33;
uint8_t acc_checked = 0;
unsigned char showcase = 0;
extern char aux[16];
extern int osd_cnt;
extern uint8_t openLogBuff[20];
extern int ledcommand;
extern uint8_t rgb_led;
extern unsigned int vol;
extern float pidkp[3];
extern float pidki[3];
extern float pidkd[3];
extern int failsafe;
extern char motorDir[4];
extern uint8_t rssip;
extern uint8_t init_motor_dir;
extern unsigned char flymode;
extern unsigned char aux6;
extern unsigned char aux7;
extern unsigned char aux8;
extern unsigned char turtlemode;
extern int rxmode;
extern uint32_t packetTimerUs;
extern uint32_t missingPackets;
extern int32_t timeoutUs;
extern uint8_t protocolState;
extern int rx_ready;
extern unsigned char low_vol;
unsigned char osd_mode_cnt=0;


#define AETR  ((-0.6f > rx[Yaw]) && (0.3f < rx[Throttle]) && (0.7f > rx[Throttle]) && (0.6f < rx[Pitch]) && (-0.3f < rx[Roll]) && (0.3f > rx[Roll]))

char down_flag = 0;
char up_flag = 0;
char right_flag = 0;
char left_flag = 0;

void getIndex()
{
    if ((rx[Pitch] < -0.6f) && (down_flag == 1))
    {
        currentMenu = currentMenu->next;
        down_flag = 0;
    }

    if ((rx[Pitch] > 0.6f) && (up_flag == 1))
    {
        currentMenu = currentMenu->prior;
        up_flag = 0;
    }

    if ((rx[Pitch]) < 0.6f && (rx[Pitch] > -0.6f))
    {
        up_flag = 1;
        down_flag = 1;
    }
    if ((rx[Roll]) < 0.6f && (rx[Roll] > -0.6f))
    {
        right_flag = 1;
        left_flag = 1;
    }
}

u32 get_vtx(u8 index)
{
    switch (index)
    {
    case 0:
        return 5865;
        break;

    case 1:
        return 5845;
        break;

    case 2:
        return 5825;
        break;

    case 3:
        return 5805;
        break;

    case 4:
        return 5785;
        break;

    case 5:
        return 5765;
        break;

    case 6:
        return 5745;
        break;

    case 7:
        return 5725;
        break;

    case 8:
        return 5733;
        break;

    case 9:
        return 5752;
        break;

    case 10:
        return 5771;
        break;

    case 11:
        return 5790;
        break;

    case 12:
        return 5809;
        break;

    case 13:
        return 5828;
        break;

    case 14:
        return 5847;
        break;

    case 15:
        return 5866;
        break;

    case 16:
        return 5705;
        break;

    case 17:
        return 5685;
        break;

    case 18:
        return 5665;
        break;

    case 19:
        return 5645;
        break;

    case 20:
        return 5885;
        break;

    case 21:
        return 5905;
        break;

    case 22:
        return 5925;
        break;

    case 23:
        return 5945;
        break;

    case 24:
        return 5740;
        break;

    case 25:
        return 5760;
        break;

    case 26:
        return 5780;
        break;

    case 27:
        return 5800;
        break;

    case 28:
        return 5820;
        break;

    case 29:
        return 5840;
        break;

    case 30:
        return 5860;
        break;

    case 31:
        return 5880;
        break;

    case 32:
        return 5658;
        break;

    case 33:
        return 5695;
        break;

    case 34:
        return 5732;
        break;

    case 35:
        return 5769;
        break;

    case 36:
        return 5806;
        break;

    case 37:
        return 5843;
        break;

    case 38:
        return 5880;
        break;

    case 39:
        return 5917;
        break;
    }
    return 5800;
}


void osd_process(void)
{
    switch (showcase)
    {
    case 0:
        if (!aux[ARMING] && AETR)
        {
            showcase = 1;

            unsigned char i = 0;
            for (i = 0; i < 3; i++)
            {
                pidMenu->fvalue = pidkp[i];
                pidMenu = pidMenu->next;
            }

            for (i = 0; i < 3; i++)
            {
                pidMenu->fvalue = pidki[i];
                pidMenu = pidMenu->next;
            }

            for (i = 0; i < 3; i++)
            {
                pidMenu->fvalue = pidkd[i];
                pidMenu = pidMenu->next;
            }
            pidMenu = pidMenuHead;
        }

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

        break;

    case 1:
        if (osd_mode)
        {
            getIndex();

            if ((rx[Roll] > 0.6f) && right_flag == 1)
            {
                switch (currentMenu->index)
                {
                case 0:
                    currentMenu = pidMenuHead;
                    showcase = 2;
                    break;
                case 1:
                    currentMenu = configMenuHead;
                    showcase = 3;
                    break;
                case 2:
                    currentMenu = receiverMenuHead;
                    showcase = 4;
                    break;
                case 3:
                    currentMenu = smartaudioMenuHead;
                    showcase = 5;
                    break;
                case 4:
                    currentMenu = motorMenuHead;
                    showcase = 6;
                    break;
//                case 5:
//                    currentMenu = infoMenuHead;
//                    showcase = 7;
//                    break;
                case 5:
                    flash_save();

                    showcase = 0;
                    currentMenu = setMenuHead;
                    down_flag = 0;
                    up_flag = 0;

                    NVIC_SystemReset();
                    break;
                case 6:
                    showcase = 0;
                    currentMenu = setMenuHead;
                    down_flag = 0;
                    up_flag = 0;
                    break;
                }
                right_flag = 0;
            }
        }
        else
        {

            if ((rx[Pitch] < -0.6f) && (down_flag == 1))
            {
                switch (currentMenu->index)
                {
                case 0:
                    currentMenu = currentMenu->next;
                case 1:
                    currentMenu = currentMenu->next;
                    break;

                case 2:
                    currentMenu = currentMenu->next;
                    currentMenu = currentMenu->next;
                    currentMenu = currentMenu->next;
                    break;

                case 5:
                    currentMenu = currentMenu->next;
                    break;

                case 6:
                    currentMenu = currentMenu->next;
                    break;
                }
                down_flag = 0;
            }

            if ((rx[Pitch] > 0.6f) && (up_flag == 1))
            {
                switch (currentMenu->index)
                {
                case 0:
                case 1:
                    currentMenu = currentMenu->prior;
                    break;

                case 2:
                    currentMenu = currentMenu->prior;
                    break;

                case 5:
                    currentMenu = currentMenu->prior;
                    currentMenu = currentMenu->prior;
                    currentMenu = currentMenu->prior;
                    break;

                case 6:
                    currentMenu = currentMenu->prior;
                    break;
                }
                up_flag = 0;
            }

            if ((rx[Pitch]) < 0.6f && (rx[Pitch] > -0.6f))
            {
                up_flag = 1;
                down_flag = 1;
            }
            if ((rx[Roll]) < 0.6f && (rx[Roll] > -0.6f))
            {
                right_flag = 1;
                left_flag = 1;
            }

            if ((rx[Roll] > 0.6f) && right_flag == 1)
            {
                switch (currentMenu->index)
                {
                case 0:
                case 1:
                    currentMenu = configMenuHead;
                    showcase = 3;
                    break;

                case 2:
                    currentMenu = receiverMenuHead;
                    showcase = 4;
                    break;
                case 5:
                    flash_save();

                    showcase = 0;
                    currentMenu = setMenuHead;
                    down_flag = 0;
                    up_flag = 0;
                    break;
                case 6:
                    showcase = 0;
                    currentMenu = setMenuHead;
                    down_flag = 0;
                    up_flag = 0;
                    break;
                }
                right_flag = 0;
            }

        }

        openLogBuff[0] = 0x0f;
        openLogBuff[0] |= showcase << 4;
        openLogBuff[1] = currentMenu->index;
        openLogBuff[2] = osd_mode;
        openLogBuff[3] = 0;
        openLogBuff[4] = 0;
        openLogBuff[5] = 0;
        openLogBuff[6] = 0;
        openLogBuff[7] = 0;
        openLogBuff[8] = 0;
        openLogBuff[9] = 0;
        openLogBuff[10] = 0;
        openLogBuff[11] = 0;
        openLogBuff[12] = 0;
        for (uint8_t i = 0; i < 12; i++)
            openLogBuff[12] += openLogBuff[i];

        UART2_DMA_Send();

        break;
    case 2:
        getIndex();
        if ((rx[Roll] > 0.6f) && right_flag == 1)
        {
            if (currentMenu->index < 9)
            {
                int a;
                currentMenu->fvalue += 0.01f;

                pidMenu = pidMenuHead;
                for (a = 0; a < 3; a++)
                {
                    pidkp[a] = pidMenu->fvalue;
                    pidMenu = pidMenu->next;
                }

                for (a = 0; a < 3; a++)
                {
                    pidki[a] = pidMenu->fvalue;
                    pidMenu = pidMenu->next;
                }

                for (a = 0; a < 3; a++)
                {
                    pidkd[a] = pidMenu->fvalue;
                    pidMenu = pidMenu->next;
                }
            }
            else
            {
                showcase = 1;
                pidMenu = pidMenuHead;
                currentMenu = setMenuHead;
            }
            right_flag = 0;
        }
        if ((rx[Roll] < -0.6f) && left_flag == 1)
        {
            if (currentMenu->index < 9)
            {
                int a;
                currentMenu->fvalue -= 0.01f;
                if (currentMenu->fvalue <= 0)
                {
                    currentMenu->fvalue = 0;
                }

                pidMenu = pidMenuHead;
                for (a = 0; a < 3; a++)
                {
                    pidkp[a] = pidMenu->fvalue;
                    pidMenu = pidMenu->next;
                }

                for (a = 0; a < 3; a++)
                {
                    pidki[a] = pidMenu->fvalue;
                    pidMenu = pidMenu->next;
                }

                for (a = 0; a < 3; a++)
                {
                    pidkd[a] = pidMenu->fvalue;
                    pidMenu = pidMenu->next;
                }
            }
            left_flag = 0;
        }

        openLogBuff[0] = 0x0f;
        openLogBuff[0] |= showcase << 4;
        openLogBuff[1] = currentMenu->index;
        openLogBuff[2] = round(pidkp[0] * 100);
        openLogBuff[3] = round(pidkp[1] * 100);
        openLogBuff[4] = round(pidkp[2] * 100);
        openLogBuff[5] = round(pidki[0] * 100);
        openLogBuff[6] = round(pidki[1] * 100);
        openLogBuff[7] = round(pidki[2] * 100);
        openLogBuff[8] = round(pidkd[0] * 100);
        openLogBuff[9] = round(pidkd[1] * 100);
        openLogBuff[10] = round(pidkd[2] * 100);
        openLogBuff[11] = 0;
        openLogBuff[12] = 0;
        for (uint8_t i = 0; i < 12; i++)
            openLogBuff[12] += openLogBuff[i];

        UART2_DMA_Send();

        break;

    case 3:
        if(osd_mode)
        {
            getIndex();
        }
        else
        {
            if ((rx[Pitch] < -0.6f) && (down_flag == 1))
            {
                switch (currentMenu->index)
                {
                case 0:
                    currentMenu = currentMenu->next;
                    break;

                case 1:
                    currentMenu = currentMenu->next;
                    break;

                case 2:
                    currentMenu = currentMenu->next;
                    currentMenu = currentMenu->next;
                    currentMenu = currentMenu->next;
                    currentMenu = currentMenu->next;
                    break;

                case 6:
                    currentMenu = currentMenu->next;
                    break;
                }
                down_flag = 0;
            }

            if ((rx[Pitch] > 0.6f) && (up_flag == 1))
            {
                switch (currentMenu->index)
                {
                case 0:
                    currentMenu = currentMenu->prior;
                    break;

                case 1:
                    currentMenu = currentMenu->prior;
                    break;

                case 2:
                    currentMenu = currentMenu->prior;
                    break;

                case 6:
                    currentMenu = currentMenu->prior;
                    currentMenu = currentMenu->prior;
                    currentMenu = currentMenu->prior;
                    currentMenu = currentMenu->prior;
                    break;
                }
                up_flag = 0;
            }
            if ((rx[Pitch]) < 0.6f && (rx[Pitch] > -0.6f))
            {
                up_flag = 1;
                down_flag = 1;
            }
            if ((rx[Roll]) < 0.6f && (rx[Roll] > -0.6f))
            {
                right_flag = 1;
                left_flag = 1;
            }
        }



        if(osd_mode)
        {
            if ((rx[Roll] > 0.6f) && right_flag == 1)
            {
                switch (currentMenu->index)
                {
                case 0:
                    osd_mode = !osd_mode;
                    if(!osd_mode)
                    {
                        rx_show = 0;
                    }
                    else
                    {
                        rx_show = 1;
                    }
                    aux6 = aux[LEVELMODE];
                    aux7 = aux[RACEMODE];
                    aux8 = aux[HORIZON];
                    break;

                case 1:
                    rgb_led = !rgb_led;
                    break;

                case 2:
                    gyro_cal();
                    acc_cal();

                    flash_save();
                    acc_checked = 1;
                    break;

                case 3:
                    ratesValuePitch += 10;
                    ratesValueRoll += 10;
                    break;

                case 4:
                    ratesValueYaw += 10;
                    break;

                case 5:
                    rx_select ++;
                    if (rx_select > 5)
                    {
                        rx_select = 0;
                    }
                    break;

                case 6:
                    currentMenu = setMenuHead;
                    showcase = 1;
                    break;
                }
                right_flag = 0;
            }
            if ((rx[Roll] < -0.6f) && left_flag == 1)
            {
                switch (currentMenu->index)
                {
                case 3:
                    ratesValuePitch -= 10;
                    ratesValueRoll -= 10;
                    break;

                case 4:
                    ratesValueYaw -= 10;
                    break;

                case 5:
                    if (rx_select == 0)
                    {
                        rx_select = 5;
                    }
                    else
                    {
                        rx_select --;
                    }
                    packetTimerUs = 0;
                    timeoutUs = 50;
                    missingPackets = 0;
                    rxCc2500SpiConfigMutable.autoBind = true;
                    rx_ready = 0;
                    rxmode = RXMODE_BIND;
                    ledcommand = 2;
                    break;

                }
                left_flag = 0;
            }
        }
        else {
            if ((rx[Roll] > 0.6f) && right_flag == 1)
            {
                switch (currentMenu->index)
                {
                case 0:
                    if(osd_mode_cnt ==3)
                    {
                        osd_mode_cnt = 0;
                        osd_mode = !osd_mode;
                    }

                    if(osd_mode_cnt ==2)
                        osd_mode_cnt++;

                    if(!osd_mode)
                    {
                        rx_show = 0;
                    }
                    else
                    {
                        rx_show = 1;
                    }
                    aux6 = aux[LEVELMODE];
                    aux7 = aux[RACEMODE];
                    aux8 = aux[HORIZON];
                    break;

                case 1:
                    rgb_led = !rgb_led;
                    break;

                case 2:
                    gyro_cal();
                    acc_cal();

                    flash_save();
                    acc_checked = 1;
                    break;

                case 6:
                    currentMenu = setMenuHead;
                    showcase = 1;
                    break;
                }
                right_flag = 0;
            }
            if ((rx[Roll] < -0.6f) && left_flag == 1)
            {
                switch (currentMenu->index)
                {
                case 0:
                    osd_mode_cnt++;
                    if(osd_mode_cnt>=2)
                        osd_mode_cnt = 2;
                    break;

                }
                left_flag = 0;
            }
        }
        openLogBuff[0] = 0x0f;
        openLogBuff[0] |= showcase << 4;
        openLogBuff[1] = currentMenu->index;
        openLogBuff[2] = osd_mode;
        openLogBuff[3] = ratesValuePitch >> 8;
        openLogBuff[4] = ratesValuePitch & 0xff;
        openLogBuff[5] = ratesValueYaw >> 8;
        openLogBuff[6] = ratesValueYaw & 0xff;
        openLogBuff[7] = acc_checked;
        openLogBuff[8] = rx_select;
        openLogBuff[9] = rgb_led;
        openLogBuff[10] = 0;
        openLogBuff[11] = 0;
        openLogBuff[12] = 0;
        for (uint8_t i = 0; i < 12; i++)
            openLogBuff[12] += openLogBuff[i];

        UART2_DMA_Send();

        break;

    case 4:
        getIndex();

        if ((rx[Roll] > 0.6f) && right_flag == 1)
        {
            switch (currentMenu->index)
            {
            case 0:
                rx_show = !rx_show;
                break;

            case 1:
                mode_show = !mode_show;
                break;

            case 2:
                vol_show = !vol_show;
                break;

            case 3:
                showcase = 1;
                currentMenu = setMenuHead;
                break;
            }
            right_flag = 0;
        }


        openLogBuff[0] = 0x0f;
        openLogBuff[0] |= showcase << 4;
        openLogBuff[1] = currentMenu->index;
        openLogBuff[2]  = rx_show;
        openLogBuff[3] =  mode_show;
        openLogBuff[4] =  vol_show;
        openLogBuff[5] =  0;
        openLogBuff[6] =  0;
        openLogBuff[7] =  0;
        openLogBuff[8] =  0;
        openLogBuff[9] =  0;
        openLogBuff[10] = 0;
        openLogBuff[11] = 0;
        openLogBuff[12] = 0;
        for (uint8_t i = 0; i < 12; i++)
            openLogBuff[12] += openLogBuff[i];

        UART2_DMA_Send();


        break;

    case 5:
        getIndex();

        if ((rx[Roll] > 0.6f) && right_flag == 1)
        {
            switch (currentMenu->index)
            {
            case 0:
                vtx_index ++;
                if (vtx_index > 39)
                    vtx_index = 0;

                vtx_freq = get_vtx(vtx_index);
                break;

            case 1:
                if(band_index<4)
                {
                    band_index++;
                    vtx_index +=8;
                }
                else {
                    band_index = 0;
                    vtx_index -=32;
                }
                vtx_freq = get_vtx(vtx_index);
                break;

            case 2:
                old_freq = vtx_freq;
                Set_RTC6705_Freq(vtx_freq);

                flash_save();
                break;

            case 3:
                currentMenu = setMenuHead;
                showcase = 1;
                break;
            }
            right_flag = 0;
        }
        if ((rx[Roll] < -0.6f) && left_flag == 1)
        {
            switch (currentMenu->index)
            {
            case 0:
                if (vtx_index == 0)
                {
                    vtx_index = 39;
                }
                else
                {
                    vtx_index --;
                }
                vtx_freq = get_vtx(vtx_index);
                break;

            case 1:
                if(band_index>0)
                {
                    band_index--;
                    vtx_index -=8;
                }
                else
                {
                    band_index = 4;
                    vtx_index +=32;
                }
                vtx_freq = get_vtx(vtx_index);
                break;

            default:
                break;
            }
            left_flag = 0;
        }
        if(vtx_index<8)
        {
            band_index = 0;
        }
        else if(vtx_index<16)
        {
            band_index = 1;
        }
        else if(vtx_index<24)
        {
            band_index = 2;
        }
        else if(vtx_index<32)
        {
            band_index = 3;
        }
        else {
            band_index = 4;
        }

        openLogBuff[0] = 0x0f;
        openLogBuff[0] |= showcase << 4;
        openLogBuff[1] = currentMenu->index;
        openLogBuff[2] = vtx_freq / 1000;
        openLogBuff[3] = vtx_freq % 1000 / 100;
        openLogBuff[4] = vtx_freq % 100 / 10;
        openLogBuff[5] = vtx_freq % 10;
        openLogBuff[6] = old_freq / 1000;
        openLogBuff[7] = old_freq % 1000 / 100;
        openLogBuff[8] = old_freq % 100 / 10;
        openLogBuff[9] = old_freq % 10;
        openLogBuff[10] = band_index;
        openLogBuff[11] = vtx_index - band_index*8;
        openLogBuff[12] = 0;
        for (uint8_t i = 0; i < 12; i++)
            openLogBuff[12] += openLogBuff[i];

        UART2_DMA_Send();

        break;

    case 6:
        getIndex();

        if ((rx[Roll] > 0.6f) && right_flag == 1)
        {
            switch (currentMenu->index)
            {
            case 0:
                motorDir[3] = !motorDir[3];
                init_motor_dir = 0;
                motor_change = 1;
                break;

            case 1:
                motorDir[1] = !motorDir[1];
                init_motor_dir = 0;
                motor_change = 1;
                break;

            case 2:
                motorDir[2] = !motorDir[2];
                init_motor_dir = 0;
                motor_change = 1;
                break;

            case 3:
                motorDir[0] = !motorDir[0];
                init_motor_dir = 0;
                motor_change = 1;
                break;

            case 4:
                motor_select = !motor_select;
                init_motor_dir = 0;
                motor_change =0;
                break;

            case 5:
                dshot_select ++;
                if (dshot_select > 2)
                    dshot_select = 0;
                break;

            case 6:
                motor_min++;
                break;

            case 7:
                currentMenu = setMenuHead;
                showcase = 1;
                break;
            }
            right_flag = 0;
        }
        if ((rx[Roll] < -0.6f) && left_flag == 1)
        {
            switch (currentMenu->index)
            {
            case 6:
                if (motor_min == 0)
                {
                }
                else
                {
                    motor_min--;
                }
                break;

            default:
                break;
            }
            left_flag = 0;
        }
        openLogBuff[0] = 0x0f;
        openLogBuff[0] |= showcase << 4;
        openLogBuff[1] = currentMenu->index;
        openLogBuff[2] = motorDir[0];
        openLogBuff[3] = motorDir[1];
        openLogBuff[4] = motorDir[2];
        openLogBuff[5] = motorDir[3];
        openLogBuff[6] = dshot_select;
        openLogBuff[7] = motor_min;
        openLogBuff[8] = motor_select;
        openLogBuff[9] = 0;
        openLogBuff[10] = 0;
        openLogBuff[11] = 0;
        openLogBuff[12] = 0;
        for (uint8_t i = 0; i < 12; i++)
            openLogBuff[12] += openLogBuff[i];

        UART2_DMA_Send();
        break;

//    case 7:
//        getIndex();

//        if ((rx[Roll] > 0.6f) && right_flag == 1)
//        {
//            switch (currentMenu->index)
//            {
//            case 0:
//                currentMenu = setMenuHead;
//                showcase = 1;
//                break;
//            }
//            right_flag = 0;
//        }
//        openLogBuff[0] = 0x0f;
//        openLogBuff[0] |= showcase << 4;
//        openLogBuff[1] = currentMenu->index;
//        openLogBuff[2] = 0;
//        openLogBuff[3] = 0;
//        openLogBuff[4] = 0;
//        openLogBuff[5] = 0;
//        openLogBuff[6] = 0;
//        openLogBuff[7] = 0;
//        openLogBuff[8] = 0;
//        openLogBuff[9] = 0;
//        openLogBuff[10] = 0;
//        openLogBuff[11] = 0;
//        openLogBuff[12] = 0;
//        for (uint8_t i = 0; i < 12; i++)
//            openLogBuff[12] += openLogBuff[i];

//        UART2_DMA_Send();
//        break;

    default:
        break;
    }
}


void osd_init(void)
{
    setMenu = createMenu(6, 0);
    setMenuHead = setMenu;

    pidMenu = createMenu(9, 1);
    pidMenuHead = pidMenu;

    configMenu = createMenu(6, 2);
    configMenuHead = configMenu;

    receiverMenu = createMenu(3, 3);
    receiverMenuHead = receiverMenu;

    smartaudioMenu = createMenu(3, 4);
    smartaudioMenuHead = smartaudioMenu;

    motorMenu = createMenu(7, 5);
    motorMenuHead = motorMenu;

    currentMenu = setMenu;
}



