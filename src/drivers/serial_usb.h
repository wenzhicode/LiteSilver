/**
******************************************************************************
* @file    serial_usb.h
* @author  wz
* @version V0.0.1
* @date    25/05/2020
* @brief   usbͷ�ļ���usb��غ�������.
******************************************************************************
*/


#include "hardware.h"




void usb_GpioInit(void);

void usbVcpWrite(unsigned char *Info);

unsigned int usbVcpAvailable(void);

uint8_t usbVcpRead(void);


