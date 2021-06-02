/**
******************************************************************************
* @file    adc.c
* @author
* @version V0.0.1
* @date    1/06/2020
* @brief   驱动文件，adc硬件相关函数.
******************************************************************************

*/


#include "adc.h"


volatile uint16_t adcarray;
volatile u32 adcvalue;



typedef struct
{
    __IO uint16_t word1;
    __IO uint16_t word2;

} adcrefcal;


uint16_t adcref_read(adcrefcal *adcref_address)
{
    return adcref_address ->word1;
}
float vref_cal;

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/**************************************************************************
**函数信息 ：adc_init(void)
**功能描述 ：adc初始化
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void adc_init(void)
{
    ADC_InitTypeDef  ADC_InitStructure;

    GPIO_Configuration();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitStructure.ADC_PRESCARE = ADC_PCLK2_PRESCARE_16;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Continuous_Scan;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, 9, 0, 0);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, 10);

    ADC_Cmd(ADC1, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    ADC_DeInit(ADC2);

    ADC_InitStructure.ADC_PRESCARE = ADC_PCLK2_PRESCARE_16;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Continuous_Scan;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Cmd(ADC2, ENABLE);

    ADC_RegularChannelConfig(ADC2, 9, 0, 0);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 0, 0);

    ADC_TempSensorVrefintCmd(ENABLE);

    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (ADC1->ADDATA);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adcvalue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);

    vref_cal = (3.3f / (float)3.3) * (float)(adcref_read((adcrefcal *) 0x1FFFF7E0));
}


/**************************************************************************
**函数信息 ：adc_read(int channel)
**功能描述 ：读取adc 通道值
**输入参数 ：channel 通道
**输出参数 ：通道值
**************************************************************************/
float adc_read(int channel)
{
    switch (channel)
    {
    case 0:
        return (float) adcvalue * ((float)(ADC_SCALEFACTOR)) ;;
        break;

    case 1:
        while (!(ADC2->ADSTA & 0x01));
        adcarray =  ADC2->ADDATA & 0xfff;

        return vref_cal / (float) adcarray;
        break;
    }
    return 0;
}

