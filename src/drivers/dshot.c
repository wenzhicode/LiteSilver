/**
******************************************************************************
* @file    dshot.c
* @author
* @version V0.0.1
* @date    27/05/2020
* @brief   驱动文件，dshot硬件相关函数.
******************************************************************************
*/


#include "dshot.h"
#include "config.h"
#include "defines.h"
#include "time.h"


volatile uint16_t dshot_portA[1] = { 0 };       // sum of all motor pins at portA
volatile uint16_t dshot_portB[1] = { 0 };       // sum of all motor pins at portB

volatile uint16_t motor_data_portA[ 16 ] = { 0 };   // DMA buffer: reset output when bit data=0 at TOH timing
volatile uint16_t motor_data_portB[ 16 ] = { 0 };   //

volatile int dshot_dma_phase = 0;                                   // 1:portA  2:portB  0:idle
volatile uint16_t dshot_packet[4];                              // 16bits dshot data for 4 motors

int pwmdir = 0;
static unsigned long pwm_failsafe_time = 1;
extern unsigned char showcase;
extern float rx[];
extern char aux[16];
extern int failsafe;
extern int onground;
extern unsigned char osd_mode;
extern unsigned char dshot_select;
extern unsigned char turtlemode;
extern float lipo_cell_count;
extern unsigned char motor_min;
extern uint8_t crash;

uint16_t dshot_bit_time=0;
uint16_t dshot_t0h_time=0;
uint16_t dshot_t1h_time=0;
uint16_t dshot_max=0;


/**************************************************************************
**函数信息 ：dshot_init(void)
**功能描述 ：dshot初始化
**输入参数 ：无
**输出参数 ：无

**************************************************************************/
void dshot_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_0 ;
    GPIO_Init(DSHOT_PORT_0, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_1 ;
    GPIO_Init(DSHOT_PORT_1, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_2 ;
    GPIO_Init(DSHOT_PORT_2, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_3 ;
    GPIO_Init(DSHOT_PORT_3, &GPIO_InitStructure);

    /*

    原来的函数中含有如下两行，不知道是什么作用。还需要进行测试
    对比手册，GPIOA PA3是UART2 RX


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
    GPIO_Init( GPIOA, &GPIO_InitStructure );

    */

    if (DSHOT_PORT_0 == GPIOA)  *dshot_portA |= DSHOT_PIN_0;
    else                                                *dshot_portB |= DSHOT_PIN_0;
    if (DSHOT_PORT_1 == GPIOA)  *dshot_portA |= DSHOT_PIN_1;
    else                                                *dshot_portB |= DSHOT_PIN_1;
    if (DSHOT_PORT_2 == GPIOA)  *dshot_portA |= DSHOT_PIN_2;
    else                                                *dshot_portB |= DSHOT_PIN_2;
    if (DSHOT_PORT_3 == GPIOA)  *dshot_portA |= DSHOT_PIN_3;
    else                                                *dshot_portB |= DSHOT_PIN_3;

    // DShot timer/DMA init
    // TIM1_UP  DMA_CH5: set all output to HIGH     at TIM1 update
    // TIM1_CH1 DMA_CH2: reset output if data=0     at T0H timing
    // TIM1_CH4 DMA_CH4: reset all output           at T1H timing

    if(dshot_select ==0)
    {
        dshot_bit_time=((SYS_CLOCK_FREQ_HZ/1000/150)-1);
        dshot_t0h_time=(DSHOT_BIT_TIME*0.30 + 0.05 );
        dshot_t1h_time=(DSHOT_BIT_TIME*0.60 + 0.05 );

    }
    else if(dshot_select==1)
    {
        dshot_bit_time=((SYS_CLOCK_FREQ_HZ/1000/300)-1);
        dshot_t0h_time=(DSHOT_BIT_TIME*0.30 + 0.05 );
        dshot_t1h_time=(DSHOT_BIT_TIME*0.60 + 0.05 );
    }
    else {
        dshot_bit_time=((SYS_CLOCK_FREQ_HZ/1000/600)-1);
        dshot_t0h_time=(DSHOT_BIT_TIME*0.30 + 0.05 );
        dshot_t1h_time=(DSHOT_BIT_TIME*0.60 + 0.05 );
    }

    if(lipo_cell_count == 1)
    {
        dshot_max = 1801;
    }
    else if(lipo_cell_count == 2) {
        dshot_max = 1501;
    }

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    // TIM1 Periph clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period =                      dshot_bit_time;
    TIM_TimeBaseStructure.TIM_Prescaler =               0;
    TIM_TimeBaseStructure.TIM_ClockDivision =       0;
    TIM_TimeBaseStructure.TIM_CounterMode =             TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /* Timing Mode configuration: Channel 1 */
    TIM_OCInitStructure.TIM_OCMode =                            TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState =               TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse =                             dshot_t0h_time;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* Timing Mode configuration: Channel 4 */
    TIM_OCInitStructure.TIM_OCMode =                            TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState =               TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse =                             dshot_t1h_time;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);


    DMA_StructInit(&DMA_InitStructure);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* DMA1 Channe5 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->BSRR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dshot_portA;
    DMA_InitStructure.DMA_DIR =                                     DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize =                      16;
    DMA_InitStructure.DMA_PeripheralInc =               DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc =                       DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize =      DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize =              DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode =                                    DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority =                            DMA_Priority_High;
    DMA_InitStructure.DMA_M2M =                                     DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    /* DMA1 Channel2 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->BRR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)motor_data_portA;
    DMA_InitStructure.DMA_DIR =                                     DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize =                      16;
    DMA_InitStructure.DMA_PeripheralInc =               DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc =                       DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize =      DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize =              DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode =                                    DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority =                            DMA_Priority_High;
    DMA_InitStructure.DMA_M2M =                                     DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    /* DMA1 Channel4 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->BRR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)dshot_portA;
    DMA_InitStructure.DMA_DIR =                                     DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize =                      16;
    DMA_InitStructure.DMA_PeripheralInc =               DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc =                       DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize =      DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize =              DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode =                                    DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority =                            DMA_Priority_High;
    DMA_InitStructure.DMA_M2M =                                     DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);


    /* configure DMA1 Channel4 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel =    DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (uint8_t)DMA_Priority_High;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = (uint8_t)DMA_Priority_High;
    NVIC_Init(&NVIC_InitStructure);
    /* enable DMA1 Channel4 transfer complete interrupt */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    pwm_failsafe_time = gettime() - 100000;
    pwmdir = 0;
}

/**************************************************************************
**函数信息 ：dshot_dma_portA(void)
**功能描述 ：dshot PA通道传输
**输入参数 ：无
**输出参数 ：无

**************************************************************************/
void dshot_dma_portA()
{
    DMA1_Channel5->CPAR = (uint32_t)&GPIOA->BSRR;
    DMA1_Channel5->CMAR = (uint32_t)dshot_portA;
    DMA1_Channel2->CPAR = (uint32_t)&GPIOA->BRR;
    DMA1_Channel2->CMAR = (uint32_t)motor_data_portA;
    DMA1_Channel4->CPAR = (uint32_t)&GPIOA->BRR;
    DMA1_Channel4->CMAR = (uint32_t)dshot_portA;

    DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_GL4 | DMA1_FLAG_GL5);

    DMA1_Channel5->CNDTR = 16;
    DMA1_Channel2->CNDTR = 16;
    DMA1_Channel4->CNDTR = 16;

    TIM1->SR = 0;

    DMA_Cmd(DMA1_Channel5, ENABLE);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);


    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);
    TIM_SetCounter(TIM1, DSHOT_BIT_TIME);
    TIM_Cmd(TIM1, ENABLE);
}

/**************************************************************************
**函数信息 ：dshot_dma_portB(void)
**功能描述 ：dshot PB通道传输
**输入参数 ：无
**输出参数 ：无

**************************************************************************/
void dshot_dma_portB()
{
    DMA1_Channel5->CPAR = (uint32_t)&GPIOB->BSRR;
    DMA1_Channel5->CMAR = (uint32_t)dshot_portB;
    DMA1_Channel2->CPAR = (uint32_t)&GPIOB->BRR;
    DMA1_Channel2->CMAR = (uint32_t)motor_data_portB;
    DMA1_Channel4->CPAR = (uint32_t)&GPIOB->BRR;
    DMA1_Channel4->CMAR = (uint32_t)dshot_portB;

    DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_GL4 | DMA1_FLAG_GL5);

    DMA1_Channel5->CNDTR = 16;
    DMA1_Channel2->CNDTR = 16;
    DMA1_Channel4->CNDTR = 16;

    TIM1->SR = 0;
    DMA_Cmd(DMA1_Channel5, ENABLE);
    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);


    TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);
    TIM_SetCounter(TIM1, DSHOT_BIT_TIME);
    TIM_Cmd(TIM1, ENABLE);
}


/**************************************************************************
**函数信息 ：make_packet( uint8_t number, uint16_t value, bool telemetry )
**功能描述 ：将发送给电机的值进行打包
**输入参数 ：number 电机序号
             value  电机转速值
             telemetry 是否开启遥测
**输出参数 ：无

**************************************************************************/
void make_packet(uint8_t number, uint16_t value, bool telemetry)
{
    uint16_t packet = (value << 1) | (telemetry ? 1 : 0);     // Here goes telemetry bit
    // compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = packet;

    for (uint8_t i = 0; i < 3; ++i)
    {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
    }

    csum &= 0xf;
    // append checksum
    dshot_packet[ number ] = (packet << 4) | csum;
}

/**************************************************************************
**函数信息 ：dshot_dma_start(void)
**功能描述 ：dshot 开始传输
**输入参数 ：无
**输出参数 ：无

**************************************************************************/
void dshot_dma_start()
{
    uint32_t    time = gettime();
    while (dshot_dma_phase != 0 && (gettime() - time) < LOOPTIME) { }    // wait maximum a LOOPTIME for dshot dma to complete
    if (dshot_dma_phase != 0) return;                                                                // skip this dshot command

//#if defined(RGB_LED_DMA) && (RGB_LED_NUMBER>0)
//    /// terminate current RGB transfer
//    extern int  rgb_dma_phase;

//    time = gettime();
//    while (rgb_dma_phase == 1 && (gettime() - time) < LOOPTIME) { }      // wait maximum a LOOPTIME for RGB dma to complete

//    if (rgb_dma_phase == 1)                                                                                      // terminate current RGB dma transfer, proceed dshot
//    {
//        rgb_dma_phase = 0;
//        DMA_Cmd(DMA1_Channel5, DISABLE);
//        DMA_Cmd(DMA1_Channel2, DISABLE);
//        DMA_Cmd(DMA1_Channel4, DISABLE);

//        TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, DISABLE);
//        TIM_Cmd(TIM1, DISABLE);
//        extern void failloop();
//        failloop(9);
//    }
//#endif

    // generate dshot dma packet
    for (uint8_t i = 0; i < 16; i++)
    {
        motor_data_portA[ i ] = 0;
        motor_data_portB[ i ] = 0;

        if (!(dshot_packet[0] & 0x8000))
        {
            if (DSHOT_PORT_0 == GPIOA)  motor_data_portA[ i ] |= DSHOT_PIN_0;
            else                                                motor_data_portB[ i ] |= DSHOT_PIN_0;
        }
        if (!(dshot_packet[1] & 0x8000))
        {
            if (DSHOT_PORT_1 == GPIOA)  motor_data_portA[ i ] |= DSHOT_PIN_1;
            else                                                motor_data_portB[ i ] |= DSHOT_PIN_1;
        }
        if (!(dshot_packet[2] & 0x8000))
        {
            if (DSHOT_PORT_2 == GPIOA)  motor_data_portA[ i ] |= DSHOT_PIN_2;
            else                                                motor_data_portB[ i ] |= DSHOT_PIN_2;
        }
        if (!(dshot_packet[3] & 0x8000))
        {
            if (DSHOT_PORT_3 == GPIOA)  motor_data_portA[ i ] |= DSHOT_PIN_3;
            else                                                motor_data_portB[ i ] |= DSHOT_PIN_3;
        }

        dshot_packet[0] <<= 1;
        dshot_packet[1] <<= 1;
        dshot_packet[2] <<= 1;
        dshot_packet[3] <<= 1;
    }

    dshot_dma_phase = DSHOT_DMA_PHASE;

    TIM1->ARR   = DSHOT_BIT_TIME;
    TIM1->CCR1  = DSHOT_T0H_TIME;
    TIM1->CCR4  = DSHOT_T1H_TIME;

    DMA1_Channel2->CCR |= DMA_MemoryDataSize_HalfWord | DMA_PeripheralDataSize_HalfWord; // switch from byte to halfword

    dshot_dma_portA();
}


/**************************************************************************
**函数信息 ：pwm_set( uint8_t number, float pwm )
**功能描述 ：将解算出来的混控输出对应到dshot
**输入参数 ：number 电机序号
             pwm  混控输出
**输出参数 ：无

**************************************************************************/
void pwm_set(uint8_t number, float pwm)
{
    uint16_t value = 0;

    extern uint8_t motor_test;
    extern uint16_t dT[4];
    if(!motor_test)
    {
        // if ( number > 3 ) failloop(5);
        if (number > 3) return;

        // maps 0.0 .. 0.999 to 48 + IDLE_OFFSET * 2 .. 2047
        value = 48 + motor_min * 20 + (uint16_t)(pwm * (dshot_max - motor_min * 20));


        if (onground || crash || showcase)
        {
            value = 0; // stop the motors
        }
        if (!aux[ARMING] && !showcase && turtlemode)
        {
            if ((rx[0] > 0.3f))
            {
                if (number > 1)
                {
                    value = 0 + rx[0] * 1000  + 1000;
                }
                else
                {
                    value = 0;
                }
            }
            if ((rx[0] < -0.3f))
            {
                if (number < 2)
                {
                    value = 0 + rx[0] * (-1000)  + 1000;
                }
                else
                {
                    value = 0;
                }
            }
            if ((rx[1] > 0.3f))
            {
                if (number == 1 || number == 3)
                {
                    value = 0 + rx[1] * 1000  + 1000;
                }
                else
                {
                    value = 0;
                }
            }
            if ((rx[1] < -0.3f))
            {
                if (number == 0 || number == 2)
                {
                    value = 0 + rx[1] * (-1000)  + 1000;
                }
                else
                {
                    value = 0;
                }
            }
        }

        if (failsafe)
        {
            if (! pwm_failsafe_time)
            {
                pwm_failsafe_time = gettime();
            }
            else
            {
                // 1s after failsafe we turn off the signal for safety
                // this means the escs won't rearm correctly after 2 secs of signal lost
                // usually the quad should be gone by then
                if (gettime() - pwm_failsafe_time > 1000000)
                {
                    value = 0;
                    /*
                    gpioreset( DSHOT_PORT_0, DSHOT_PIN_0 );
                    gpioreset( DSHOT_PORT_1, DSHOT_PIN_1 );
                    gpioreset( DSHOT_PORT_2, DSHOT_PIN_2 );
                    gpioreset( DSHOT_PORT_3, DSHOT_PIN_3 );
                    */
                    //////
                    return;
                }
            }
        }
        else
        {
            pwm_failsafe_time = 0;
        }

        make_packet(number, value, false);

        
        if (number == 3 )
        {
            dshot_dma_start();
        }
    }
    else{
        
        make_packet(number,dT[number],false);

        if (number == 3 )
        {
            dshot_dma_start();
        }
    }
}

/**************************************************************************
**函数信息 ：DMA1_Channel4_IRQHandler(void)
**功能描述 ：DMA中断处理
**输入参数 ：无
**输出参数 ：无

**************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{

    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_Cmd(DMA1_Channel2, DISABLE);


    TIM_DMACmd(TIM1, TIM_DMA_Update, DISABLE);
    TIM_DMACmd(TIM1, TIM_DMA_CC4, DISABLE);
    TIM_DMACmd(TIM1, TIM_DMA_CC1, DISABLE);

    DMA_ClearITPendingBit(DMA1_IT_TC4);

    TIM_Cmd(TIM1, DISABLE);



    switch (dshot_dma_phase)
    {
    case 2:
        dshot_dma_phase = 1;
        dshot_dma_portB();
        return;
    case 1:
        dshot_dma_phase = 0;
//#if defined(RGB_LED_DMA) && (RGB_LED_NUMBER>0)
//        extern int rgb_dma_phase;
//        extern void rgb_dma_trigger();

//        if (rgb_dma_phase == 2)
//        {
//            rgb_dma_phase = 1;
//            rgb_dma_trigger();
//        }
//#endif
        return;
    default :
        dshot_dma_phase = 0;
        break;
    }
//#if defined(RGB_LED_DMA) && (RGB_LED_NUMBER>0)
//    extern int rgb_dma_phase;
//    rgb_dma_phase = 0;
//#endif
}



void motor_dir(unsigned char  number, unsigned char value)
{
    make_packet(number, value, true);
    if (number == 3)
    {
        dshot_dma_start();
    }
}



