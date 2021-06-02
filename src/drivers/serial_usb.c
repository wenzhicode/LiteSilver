/**
******************************************************************************
* @file    serial_usb.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   驱动文件，usb硬件相关函数.
******************************************************************************
*/


#include "serial_usb.h"
#include "sys.h"
#include "usb.h"
#include "usbprop.h"


/**************************************************************************
**函数信息 ：usb_init(void)
**功能描述 ：usb初始化
**输入参数 ：无
**输出参数 ：无

系统时钟96MHz,此处2分频给USB使用

**************************************************************************/
void usb_GpioInit(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure1;

    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure1);

    MY_NVIC_Init(0, 0, USB_HP_CAN1_TX_IRQn, 2);          //配置USB中断

}








