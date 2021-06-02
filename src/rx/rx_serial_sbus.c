/**
******************************************************************************
* @file    rx_serial_sbus.c
* @author
* @version V0.0.1
* @date    10/06/2020
* @brief   sbus文件，sbus接收相关函数.
******************************************************************************

sbus串口通信 采用的是负逻辑，即低电平为“1”，高电平为“0”

波特率：100000（100k），注意：不兼容波特率115200。数据位8bit，1个停止位，无校验位，无流控制

每帧数据还有25字节。

每字节含有12个比特，使用1个起始位“0”，8个数据位，1个奇校验位（8个数据位中1的数量为奇数则此位为“1”否则为“0”），两个终止位“1”。
采用LSB first方式发送，即最低有效位（二进制数据右侧）先发。
帧头：1111 0000（二进制），帧尾：0000 0000（二进制）。
数据：从第1数据字节起，到第22字节，一共有数据位176个，它们按照顺序分别是通道1至通道16的舵机控制数据，每个通道占11比特。取值范围是0~2047。
第23字节我管它叫做“功能字节”，第0比特为数字通道1的值，第1比特为数字通道2的值，第2比特为丢帧信息，第3比特为失效保护开关，第4~7比特暂时保留没用


mm32f103 不支持串口硬件取反，需要外加取反电路


*/


#include "rx_serial_sbus.h"
#include "time.h"
#include "stdbool.h"
#include "serial_uart.h"
#include "defines.h"
#include "util.h"
#include "rx_spi.h"
#include "rx_serial_dsm.h"


#ifdef USE_SERIAL_SBUS

#define RX_BUFF_SIZE 26                         //SPEK_FRAME_SIZE 16  
uint8_t rx_buffer[RX_BUFF_SIZE];    //spekFrame[SPEK_FRAME_SIZE]
uint8_t rx_start = 0;
uint8_t rx_end = 0;
uint16_t rx_time[RX_BUFF_SIZE];

int framestarted = -1;
uint8_t framestart = 0;

int failsafe = 1;
int rxmode = 0;
int rx_ready = 0;
extern int ledcommand;
unsigned long time_lastrx;
unsigned long time_siglost = 0;
uint8_t last_rx_end = 0;
int last_byte = 0;
unsigned long time_lastframe;
int frame_received = 0;
int rx_state = 0;
int bind_safety = 0;
uint8_t data[25];
int channels[9];

sbusFrameData_t sbusFrameData;
static bool rxIsInFailsafeMode = true;
extern char aux[16];
extern float rx[4];
extern uint16_t chan[4];

#define SBUS_TIME_NEEDED_PER_FRAME    3000
#define SBUS_FRAME_BEGIN_BYTE 0x0F

uint16_t val[18];  // 通道值
uint8_t  rc_flag = 0;
int failsafe_siglost = 0;


void sbus_init(void)
{
    //初始化串口1 接收中断
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //使能UART1，GPIOA时钟


    //UART 初始化设置
    UART_InitStructure.UART_BaudRate = 100000;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_2;
    UART_InitStructure.UART_Parity = UART_Parity_Even;
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    UART_InitStructure.UART_Mode = UART_Mode_Rx;


    UART_Init(UART1, &UART_InitStructure); //初始化串口1
    UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);//开启串口接收中断
    UART_Cmd(UART1, ENABLE);                    //使能串口1


    //UART1_RX    GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10

    //UART1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器

    //rxmode = !RXMODE_BIND;
    framestarted = 0;
}





#if 0
uint8_t sbusChannelsDecode(uint8_t *buff)
{
    int ind = 0;
    val[0] = ((buff[ind + 1] | buff[ind + 2] << 8) & 0x07FF);
    val[1] = ((buff[ind + 2] >> 3 | buff[ind + 3] << 5) & 0x07FF);
    val[2] = ((buff[ind + 3] >> 6 | buff[ind + 4] << 2 | buff[ind + 5] << 10) & 0x07FF);
    val[3] = ((buff[ind + 5] >> 1 | buff[ind + 6] << 7) & 0x07FF);
    val[4] = ((buff[ind + 6] >> 4 | buff[ind + 7] << 4) & 0x07FF);
    val[5] = ((buff[ind + 7] >> 7 | buff[ind + 8] << 1 | buff[ind + 9] << 9) & 0x07FF);
    val[6] = ((buff[ind + 9] >> 2 | buff[ind + 10] << 6) & 0x07FF);
    val[7] = ((buff[ind + 10] >> 5 | buff[ind + 11] << 3) & 0x07FF);
    val[8] = ((buff[ind + 12] | buff[ind + 13] << 8) & 0x07FF);
    val[9] = ((buff[ind + 13] >> 3 | buff[ind + 14] << 5) & 0x07FF);
    val[10] = ((buff[ind + 14] >> 6 | buff[ind + 15] << 2 | buff[ind + 16] << 10) & 0x07FF);
    val[11] = ((buff[ind + 16] >> 1 | buff[ind + 17] << 7) & 0x07FF);
    val[12] = ((buff[ind + 17] >> 4 | buff[ind + 18] << 4) & 0x07FF);
    val[13] = ((buff[ind + 18] >> 7 | buff[ind + 19] << 1 | buff[ind + 20] << 9) & 0x07FF);
    val[14] = ((buff[ind + 20] >> 2 | buff[ind + 21] << 6) & 0x07FF);
    val[15] = ((buff[ind + 21] >> 5 | buff[ind + 22] << 3) & 0x07FF);

    rx[0] = (val[0] - 993) * 0.00122026f;
    rx[1] = (val[1] - 993) * 0.00122026f;
    rx[2] = (val[3] - 993) * 0.00122026f;

    rx[3] = (val[2] - 173) * 0.000610128f;

    if (rx[3] > 1) rx[3] = 1;
    if (rx[3] < 0) rx[3] = 0;

    aux[CHAN_5] = (val[4] > 993) ? 1 : 0;
    aux[CHAN_6] = (val[5] > 993) ? 1 : 0;
    aux[CHAN_7] = (val[6] > 993) ? 1 : 0;
    aux[CHAN_8] = (val[7] > 993) ? 1 : 0;

    failsafe = 0;
    extern int binding_while_armed ;
    binding_while_armed = 0;
    return RX_FRAME_COMPLETE;
}


#endif

uint8_t sbusChannelsDecode(const sbusChannels_t *channels)
{
    val[0] = channels->chan0;
    val[1] = channels->chan1;
    val[2] = channels->chan2;
    val[3] = channels->chan3;
    val[4] = channels->chan4;
    val[5] = channels->chan5;
    val[6] = channels->chan6;
    val[7] = channels->chan7;
    val[8] = channels->chan8;
    val[9] = channels->chan9;
    val[10] = channels->chan10;
    val[11] = channels->chan11;
    val[12] = channels->chan12;
    val[13] = channels->chan13;
    val[14] = channels->chan14;
    val[15] = channels->chan15;

    if (channels->flags & SBUS_FLAG_CHANNEL_17)
    {
        val[16] = SBUS_DIGITAL_CHANNEL_MAX;
    }
    else
    {
        val[16] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_CHANNEL_18)
    {
        val[17] = SBUS_DIGITAL_CHANNEL_MAX;
    }
    else
    {
        val[17] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_FAILSAFE_ACTIVE)
    {
        // internal failsafe enabled and rx failsafe flag set
        // RX *should* still be sending valid channel data (repeated), so use it.
        return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
    }

    if (channels->flags & SBUS_FLAG_SIGNAL_LOSS)
    {
        // The received data is a repeat of the last valid data so can be considered complete.
        return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
    }

    return RX_FRAME_COMPLETE;
}

void sbus_check(void)
{
    bool signalReceived = false;

    if (!sbusFrameData.done)
    {
        return ;
    }
    sbusFrameData.done = false;


    const uint8_t frameStatus = sbusChannelsDecode(&sbusFrameData.frame.frame.channels);


//    if (!(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
//        lastRcFrameTimeUs = sbusTimeLast;
//    }

    rxIsInFailsafeMode = (frameStatus & RX_FRAME_FAILSAFE) != 0;
    bool rxFrameDropped = (frameStatus & RX_FRAME_DROPPED) != 0;
    signalReceived = !(rxIsInFailsafeMode || rxFrameDropped);

    if (signalReceived)
    {

        rx[0] = (val[0] - 993) * 0.00122026f;
        rx[1] = (val[1] - 993) * 0.00122026f;
        rx[2] = (val[3] - 993) * 0.00122026f;

        rx[3] = (val[2] - 173) * 0.000610128f;

        if (rx[3] > 1) rx[3] = 1;
        if (rx[3] < 0) rx[3] = 0;

        chan[0] = val[4];
        chan[1] = val[5];
        chan[2] = val[6];
        chan[3] = val[7];

        aux[CHAN_5] = (val[4] > 500) ? ((val[4] < 1500) ? 1 : 2) : 0;
        aux[CHAN_6] = (val[5] > 500) ? ((val[5] < 1500) ? 1 : 2) : 0;
        aux[CHAN_7] = (val[6] > 500) ? ((val[6] < 1500) ? 1 : 2) : 0;
        aux[CHAN_8] = (val[7] > 500) ? ((val[7] < 1500) ? 1 : 2) : 0;

        if (aux[LEVELMODE])
        {
            if (aux[RACEMODE] && !aux[HORIZON])
            {
                if (ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
                if (ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
                if (ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
            }
            else if (aux[HORIZON])
            {
                if (ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
                if (ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
                if (ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
            }
            else
            {
                if (ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
                if (ANGLE_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ANGLE_EXPO_PITCH);
                if (ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
            }
        }
        else
        {
            if (ACRO_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
            if (ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
            if (ACRO_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ACRO_EXPO_YAW);
        }

        bind_safety++;
        if (bind_safety > 100)
        {
            rx_ready = 1;
            failsafe = 0;
            rxmode = !RXMODE_BIND;
            bind_safety = 101;
            if(ledcommand != 5)
                ledcommand = 4;
        }
        time_siglost = gettime();
        failsafe_siglost = 0;
    }
    else
    {
        if (gettime() - time_siglost > 1000000)
        {
            failsafe_siglost = 1;

            rx[0] = 0;
            rx[1] = 0;
            rx[2] = 0;
            rx[3] = 0;

            if(ledcommand !=2)
                ledcommand = 3;
        }

        failsafe = failsafe_siglost;

    }
}


extern unsigned char sbus_dsm;
extern uint8_t spekFramePosition;
extern  u32 spekTimeLast;
extern u32 spekTime;
extern u32 spekTimeInterval;

extern unsigned int dsmtime;
extern  unsigned int lastdsmtime;
extern volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
extern int rcFrameComplete;


void UART1_IRQHandler(void)
{

    if (UART_GetITStatus(UART1, UART_IT_RXIEN)  != RESET)
    {
        UART_ClearITPendingBit(UART1, UART_IT_RXIEN);

        if (sbus_dsm)
        {
            uint8_t c =  UART1->RDR;
            const uint32_t nowUs = gettime();

            const int32_t sbusFrameTime = cmpTimeUs(nowUs, sbusFrameData.startAtUs);


            if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500))
            {
                sbusFrameData.position = 0;
            }
            if (sbusFrameData.position == 0)
            {
                if (c != SBUS_FRAME_BEGIN_BYTE)
                {
                    return;
                }
                sbusFrameData.startAtUs = nowUs;
            }

            if (sbusFrameData.position < SBUS_FRAME_SIZE)
            {
                sbusFrameData.frame.bytes[sbusFrameData.position++] = (uint8_t)c;
                if (sbusFrameData.position < SBUS_FRAME_SIZE)
                {
                    sbusFrameData.done = false;
                }
                else
                {
                    sbusFrameData.done = true;
                }
            }
        }
        else
        {

            spekTime = gettime();
            spekTimeInterval = cmpTimeUs(spekTime, spekTimeLast);
            spekTimeLast = spekTime;

            if (spekTimeInterval > SPEKTRUM_NEEDED_FRAME_INTERVAL)
            {
                spekFramePosition = 0;
            }

            if (spekFramePosition < SPEK_FRAME_SIZE)
            {
                spekFrame[spekFramePosition++] = UART1->RDR;
                if (spekFramePosition < SPEK_FRAME_SIZE)
                {
                    rcFrameComplete = 0;
                }
                else
                {
                    rcFrameComplete = 1;
                }
            }

            spekFramePosition %= (SPEK_FRAME_SIZE);

        }
    }

}


#endif


