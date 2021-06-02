/**
******************************************************************************
* @file    serial_usb.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   �����ļ���usbӲ����غ���.
******************************************************************************
*/


#include "serial_usb.h"
#include "sys.h"
#include "usb.h"
#include "usbprop.h"


/**************************************************************************
**������Ϣ ��usb_init(void)
**�������� ��usb��ʼ��
**������� ����
**������� ����

ϵͳʱ��96MHz,�˴�2��Ƶ��USBʹ��

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

    MY_NVIC_Init(0, 0, USB_HP_CAN1_TX_IRQn, 2);          //����USB�ж�

}








