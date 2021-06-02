/**
******************************************************************************
* @file    misc_gpio.c
* @author
* @version V0.0.1
* @date    9/06/2020
* @brief   驱动文件，gpio硬件相关函数.
******************************************************************************
*/

#include "misc_gpio.h"


void misc_gpioInit(void)
{
    //初始化备份寄存器的时钟
//    RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN , ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOA.9

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOA.9

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOA.9

    GPIO_ResetBits(GPIOB, GPIO_Pin_2);

    GPIO_SetBits(GPIOB, GPIO_Pin_7);

    GPIO_SetBits(GPIOD, GPIO_Pin_1);

    GPIO_InitTypeDef ExtiState;
    ExtiState.GPIO_Pin=GPIO_Pin_0;
    ExtiState.GPIO_Mode=GPIO_Mode_IPU;
    ExtiState.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD,&ExtiState);

    GPIO_InitTypeDef Read_IO_KEY;
    Read_IO_KEY.GPIO_Pin=GPIO_Pin_8;
    Read_IO_KEY.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_Init(GPIOB,&Read_IO_KEY);

}


