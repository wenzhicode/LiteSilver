/**
******************************************************************************
* @file    rx_serial_sbus.h
* @author
* @version V0.0.1
* @date    10/06/2020
* @brief   头文件，sbus接收相关函数声明.
******************************************************************************
*/



#include "hardware.h"



void sbus_init(void);


void sbus_check(void);


typedef struct sbusChannels_s
{
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
} __attribute__((__packed__)) sbusChannels_t;


#define SBUS_CHANNEL_DATA_LENGTH sizeof(sbusChannels_t)


#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)



#define SBUS_MAX_CHANNEL 18

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812


struct sbusFrame_s
{
    uint8_t syncByte;
    sbusChannels_t channels;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
     * and
     * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__((__packed__));

typedef union sbusFrame_u
{
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;

typedef struct sbusFrameData_s
{
    sbusFrame_t frame;
    uint32_t startAtUs;
    uint8_t position;
    bool done;
} sbusFrameData_t;







