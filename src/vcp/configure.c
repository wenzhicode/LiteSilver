/**
******************************************************************************
* @file    configure.c
* @author
* @version V0.0.1
* @date    9/05/2020
* @brief   configure文件，configure相关函数.
******************************************************************************

上位机与飞控的通信相关函数。协议是mavlink.

上位机发送命令过来，飞控才应答，不发不答，发一答一

*/

#include "configure.h"
#include "config.h"
#include "defines.h"
#include "time.h"
#include "mpu6500.h"
#include "flash.h"
#include "dshot.h"


extern uint16_t year;
extern uint16_t month;
extern uint16_t day;
extern uint8_t mdk_hour;
extern uint8_t mdk_min;
extern uint8_t mdk_sec;


//mavlink send defines
mavlink_message_t mavlink_msg;
uint8_t mavlink_buf[64];
unsigned int mavlink_len;

Mavlink_Messages currentMsg;

//mavlink receive defines
int num = 0;
uint8_t receBuf[64] = {0};

mavlink_message_t receivemsg;
mavlink_status_t status;

uint16_t chan[4]= {0};
uint8_t tab_index=6;
uint8_t sub_index=0;

uint8_t reset = 0;
uint8_t usb_msp=0;

extern float rx[];
extern float aux[];
extern unsigned char rx_select;
extern float lowvol;
extern float vbattfilt;
extern int failsafe;
extern float attitude[];

extern int16_t acclN[3];
extern int16_t gyroN[3];

extern int16_t motor_value[4];

extern int8_t temper;


uint8_t rxN;
uint8_t sensors;

uint8_t mdk_ver=2;
uint16_t dT[4];
uint8_t motor_test=0;

extern float rcRate[3];
extern float superExpo[3];
extern float Expo[3];

static uint8_t rspb = 0;

void serialProcess(void)
{
    num = CDC_Receive_BytesAvailable();
    if (num)
    {
        UsbVcomRec(receBuf, num);

        for (int i = 0; i < num; i++)
        {
            if (mavlink_parse_char(0, *(receBuf + i), &receivemsg, &status))
            {
                switch (receivemsg.msgid)
                {
                case MAVLINK_MSG_ID_config_tab:
                    mavlink_msg_config_tab_decode(&receivemsg,&currentMsg.tab_index);
                    tab_index = currentMsg.tab_index.tab_index;
                    sub_index = currentMsg.tab_index.sub_index;
                    break;

                case MAVLINK_MSG_ID_reboot:
                    mavlink_msg_reboot_decode(&receivemsg,&currentMsg.reboot);
                    reset = currentMsg.reboot.reset;
                    usb_msp = currentMsg.reboot.msp;
                    break;

                case MAVLINK_MSG_ID_calibrate:
                    gyro_cal();
                    acc_cal();

                    flash_save();
                    delay_ms(1);

                    mavlink_len = mavlink_msg_calibrate_pack(1,5,&mavlink_msg,1);
                    mavlink_len = mavlink_msg_to_send_buffer(mavlink_buf,&mavlink_msg);
                    UsbVcomSend(mavlink_buf,mavlink_len);
                    break;

                case MAVLINK_MSG_ID_motor_io:
                    mavlink_msg_motor_io_decode(&receivemsg,&currentMsg.motor_io);
                    motor_test = currentMsg.motor_io.mtest;

                    break;

                case MAVLINK_MSG_ID_motor_value:
                    mavlink_msg_motor_value_decode(&receivemsg,&currentMsg.motor_value);

                    dT[0] = currentMsg.motor_value.motor1;
                    dT[1] = currentMsg.motor_value.motor2;
                    dT[2] = currentMsg.motor_value.motor3;
                    dT[3] = currentMsg.motor_value.motor4;
                    break;

                case MAVLINK_MSG_ID_rates:
                    mavlink_msg_rates_decode(&receivemsg,&currentMsg.rates);

                    rspb = currentMsg.rates.rspb;

                    if(rspb)
                    {
                        rcRate[0] = currentMsg.rates.rrc * 0.01;
                        rcRate[1] = currentMsg.rates.prc * 0.01;
                        rcRate[2] = currentMsg.rates.yrc * 0.01;

                        superExpo[0] = currentMsg.rates.rr * 0.01;
                        superExpo[1] = currentMsg.rates.pr * 0.01;
                        superExpo[2] = currentMsg.rates.yr * 0.01;

                        Expo[0] = currentMsg.rates.re * 0.01;
                        Expo[1] = currentMsg.rates.pe * 0.01;
                        Expo[2] = currentMsg.rates.ye * 0.01;

                        flash_save();
                        delay_ms(10);
                    }
                    else
                    {
                        mavlink_msg_rates_pack(1,5,&mavlink_msg,0,rcRate[0]*100,superExpo[0]*100,Expo[0]*100,rcRate[1]*100,superExpo[1]*100,Expo[1]*100,rcRate[2]*100,superExpo[2]*100,Expo[2]*100);

                        mavlink_len = mavlink_msg_to_send_buffer(mavlink_buf,&mavlink_msg);
                        UsbVcomSend(mavlink_buf,mavlink_len);

                    }

                default:
                    break;

                }
            }
        }
    }

    switch(tab_index)
    {
    case 0:
        break;

    case 1:
        break;

    case 2:
        break;

    case 3:
        if(sub_index)
        {
            if(motor_test)
            {
                mavlink_len = mavlink_msg_motor_value_pack(1,5,&mavlink_msg,dT[0],dT[1],dT[2],dT[3]);
            }
            else
            {
                mavlink_len = mavlink_msg_motor_value_pack(1,5,&mavlink_msg,motor_value[0],motor_value[1],motor_value[2],motor_value[3]);
            }

            mavlink_len = mavlink_msg_to_send_buffer(mavlink_buf,&mavlink_msg);
            UsbVcomSend(mavlink_buf,mavlink_len);

        }
        else
        {

        }
        break;

    case 4:
        mavlink_len = mavlink_msg_tx_pack(1,5,&mavlink_msg,rx[0]*500,rx[1]*500,rx[2]*500,rx[3]*1000,chan[0],chan[1],chan[2],chan[3],attitude[0],attitude[1],temper);
        mavlink_len = mavlink_msg_to_send_buffer(mavlink_buf,&mavlink_msg);
        UsbVcomSend(mavlink_buf,mavlink_len);

        delay_ms(1);
        mavlink_len = mavlink_msg_attitude_pack(1,5,&mavlink_msg,acclN[0],acclN[1],acclN[2],gyroN[0],gyroN[1],gyroN[2]);
        mavlink_len = mavlink_msg_to_send_buffer(mavlink_buf,&mavlink_msg);
        UsbVcomSend(mavlink_buf,mavlink_len);
        break;

    case 5:
        break;

    case 6:
        rxN = (failsafe << 4) | rx_select;
        sensors = (0<<4) | (1<<2) | 1;
        mavlink_len = mavlink_msg_status_build_pack(1,5,&mavlink_msg,1,mdk_ver,year,month,day,mdk_hour,mdk_min,mdk_sec,0,vbattfilt,rxN,sensors);
        mavlink_len = mavlink_msg_to_send_buffer(mavlink_buf,&mavlink_msg);
        UsbVcomSend(mavlink_buf,mavlink_len);
        break;

    default:
        break;

    }

}




