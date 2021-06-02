/**
******************************************************************************
* @file    rx_serial_dsm.h
* @author
* @version V0.0.1
* @date    19/06/2020
* @brief   头文件，dsm接收相关函数声明.
******************************************************************************
*/


#include "hardware.h"






#define DSM_SCALE_PERCENT 150                                               //adjust this line to match the stick scaling % set in your transmitter
#define SERIAL_BAUDRATE 115200
#define SPEK_FRAME_SIZE 16
#define SPEKTRUM_NEEDED_FRAME_INTERVAL  5000
#define SPEKTRUM_MAX_FADE_PER_SEC       40
#define SPEKTRUM_FADE_REPORTS_PER_SEC   2
#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT     12
#define SPEKTRUM_1024_CHANNEL_COUNT     7




void dsm_init(void);

void dsm_bind(void);

void dsm_check(void);

