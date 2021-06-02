/**
******************************************************************************
* @file    rx_serial_dsm.c
* @author
* @version V0.0.1
* @date    10/06/2020
* @brief   dsm�ļ���dsm������غ���.
******************************************************************************

DSM��Digital Spread Spectrum Modulation����д��һ���������� DSM��DSM2��DSMX

ʹ�ñ�׼����Э�飬���ڲ�����115200������λ8bit��1��ֹͣλ����У��λ����������

���Э��򵥵öࡣ
ÿһ֡����ʹ������Byte��16�ֽڡ�
��1���ֽڱ�ʾ��֡״̬��Ϊ0��ʱ���ʾ��֡Ϊң�����ݣ�1��ʱ���ʾΪ�������ݡ�
����4���ֽ�Ϊͨ��ID����Ӧ���ջ���������0: Throttle, 1: Aileron, 2: Elevator, 3: Rudder, 4: Gear, 5: Aux1, 6: Aux2, 7: Aux3��
��11���ֽ���0~2047��ͨ�����ݡ�

1023Ϊ�м�ֵ��Ӧ���1.5ms��PWM�źţ�0��Ӧ0.75ms��2047��Ӧ2.25ms


*/




#include "rx_serial_dsm.h"
#include "config.h"
#include "serial_uart.h"
#include "time.h"
#include "defines.h"
#include "rx.h"
#include "util.h"


#ifdef USE_SERIAL_DSMX


// global use rx variables
extern float rx[4];
extern char aux[16];
extern char lastaux[16];
extern char auxchange[16];
extern int ledcommand;
extern int failsafe;
extern int rxmode;
extern int rx_ready;
extern int bind_safety;
int rx_bind_enable = 0;

// internal dsm variables

#ifdef RX_DSMX_2048
#define CHANNEL_COUNT 10
#define BIND_PULSES 9
// 11 bit frames
static uint8_t spek_chan_shift = 3;
static uint8_t spek_chan_mask = 0x07;
#endif

#ifdef RX_DSM2_1024
#define CHANNEL_COUNT 7
#define BIND_PULSES 3
// 10 bit frames
static uint8_t spek_chan_shift = 2;
static uint8_t spek_chan_mask = 0x03;
#endif

extern uint32_t channels[CHANNEL_COUNT];
int rcFrameComplete = 0;
extern int framestarted;
int rx_frame_pending;
int rx_frame_pending_last;
uint32_t flagged_time;
volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
float dsm2_scalefactor = (0.29354210f / DSM_SCALE_PERCENT);
float dsmx_scalefactor = (0.14662756f / DSM_SCALE_PERCENT);

uint8_t spekFramePosition = 0;
u32 spekTimeLast = 0;
u32 spekTime = 0;
u32 spekTimeInterval = 0;

extern uint16_t chan[4];
unsigned int dsmtime;
unsigned int lastdsmtime;

/*************************************************************************
**������Ϣ ��dsm_bind(void)
**�������� ����dsm���а�
**������� ����
**������� ����
**************************************************************************/
void dsm_bind(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RX line, set high
    GPIO_SetBits(GPIOA, GPIO_Pin_10);
    // Bind window is around 20-140ms after powerup
    delay_ms(60);

    for (uint8_t i = 0; i < BIND_PULSES; i++)   // 9 pulses for internal dsmx 11ms, 3 pulses for internal dsm2 22ms
    {
        // RX line, drive low for 120us
        GPIO_ResetBits(GPIOA, GPIO_Pin_10);
        delay_us(120);

        // RX line, drive high for 120us
        GPIO_SetBits(GPIOA, GPIO_Pin_10);
        delay_us(120);
    }

}


/*************************************************************************
**������Ϣ ��dsm_init(void)
**�������� ��dsm ��ʼ��
**������� ����
**������� ����
**************************************************************************/
void dsm_init(void)
{
    //��ʼ������1 �����ж�
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //ʹ��UART1��GPIOAʱ��


    //UART ��ʼ������
    UART_InitStructure.UART_BaudRate = 115200;//���ڲ�����
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    UART_InitStructure.UART_StopBits = UART_StopBits_1;//һ��ֹͣλ
    UART_InitStructure.UART_Parity = UART_Parity_No;//����żУ��λ
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//��Ӳ������������
    UART_InitStructure.UART_Mode = UART_Mode_Rx;    //�շ�ģʽ

    UART_Init(UART1, &UART_InitStructure); //��ʼ������1
    UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);//�������ڽ����ж�
    UART_Cmd(UART1, ENABLE);                    //ʹ�ܴ���1


    //UART1_RX    GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10

    //UART1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���

    framestarted = 0;
}


/*************************************************************************
**������Ϣ ��spektrumFrameStatus(void)
**�������� �����dsm״̬
**������� ����
**������� ����
**************************************************************************/
void spektrumFrameStatus(void)
{
    if (rcFrameComplete == 0)
    {
        rx_frame_pending = 1;
    }
    else
    {
        rcFrameComplete = 0;

        for (int b = 3; b < SPEK_FRAME_SIZE; b += 2)
        {
            const uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
            if (spekChannel < CHANNEL_COUNT && spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT)
            {
                channels[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
                framestarted = 1;
                rx_frame_pending = 0;
                bind_safety++;

            }
        }
    }
}


/*************************************************************************
**������Ϣ ��dsm_check(void)
**�������� ������dsm������
**������� ����
**������� ����
**************************************************************************/
void dsm_check(void)
{
    unsigned long time = gettime();
    dsmtime = ((uint32_t)(time - lastdsmtime));

    lastdsmtime = time;

    if (framestarted < 0)
    {
        failsafe = 1;
        dsm_init();
        rxmode = !RXMODE_BIND;
        ledcommand = 2;
    }

    if (framestarted == 0)
    {
        failsafe = 1;
        if(ledcommand !=2)
            ledcommand = 3;
    }

    rx_frame_pending_last = rx_frame_pending;
    spektrumFrameStatus();

    if (rx_frame_pending != rx_frame_pending_last) flagged_time = gettime();
    if (gettime() - flagged_time > FAILSAFETIME) framestarted = 0;

    if ((bind_safety < 100) && (bind_safety > 0)) rxmode = RXMODE_BIND;

    // TAER channel order
#ifdef RX_DSMX_2048
    rx[0] = (channels[1] - 1024.0f) * dsmx_scalefactor;
    rx[1] = (channels[2] - 1024.0f) * dsmx_scalefactor;
    rx[2] = (channels[3] - 1024.0f) * dsmx_scalefactor;
    rx[3] = ((channels[0] - 1024.0f) * dsmx_scalefactor * 0.5f) + 0.5f;

    if (rx[3] > 1) rx[3] = 1;
    if (rx[3] < 0) rx[3] = 0;
#endif

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
#ifdef RX_DSM2_1024
    rx[0] = (channels[1] - 512.0f) * dsm2_scalefactor;
    rx[1] = (channels[2] - 512.0f) * dsm2_scalefactor;
    rx[2] = (channels[3] - 512.0f) * dsm2_scalefactor;
    rx[3] = ((channels[0] - 512.0f) * dsm2_scalefactor * 0.5f) + 0.5f;

    if (rx[3] > 1) rx[3] = 1;
    if (rx[3] < 0) rx[3] = 0;
#endif



    //ң����ͨ��ֵ
#ifdef RX_DSMX_2048

    chan[0] = channels[4];
    chan[1] = channels[5];
    chan[2] = channels[6];
    chan[3] = channels[7];

    aux[CHAN_5] = (channels[4] > 1100) ? 1 : 0;
    aux[CHAN_6] = (channels[5] > 1100) ? 1 : 0;
    aux[CHAN_7] = (channels[6] > 1100) ? 1 : 0;
    aux[CHAN_8] = (channels[7] > 1100) ? 1 : 0;
    aux[CHAN_9] = (channels[8] > 1100) ? 1 : 0;
    aux[CHAN_10] = (channels[9] > 1100) ? 1 : 0;
#endif

#ifdef RX_DSM2_1024
    aux[CHAN_5] = (channels[4] > 550) ? 1 : 0;
    aux[CHAN_6] = (channels[5] > 550) ? 1 : 0;
    aux[CHAN_7] = (channels[6] > 550) ? 1 : 0;
#endif

    if (bind_safety > 100)
    {
        rx_ready = 1;
        failsafe = 0;
        rxmode = !RXMODE_BIND;
        bind_safety = 101;
        if(ledcommand != 5)
            ledcommand = 4;
    }
}


#endif



