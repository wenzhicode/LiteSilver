/**
******************************************************************************
* @file    usbprop.h
* @author  AE Team
* @version V1.1.0
* @date    28/08/2019
* @brief   This file contains all the usbprop inc file for the library.
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, MindMotion SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2019 MindMotion</center></h2>
*/
#ifndef __usb_prop_H
#define __usb_prop_H

#include "stdint.h"

typedef struct
{
    unsigned int bitrate;
    unsigned char format;
    unsigned char paritytype;
    unsigned char datatype;
} LINE_CODING;

unsigned int UsbVcomRec(unsigned char *rxInfo, int len);
char UsbVcomSend(const unsigned char *Info, unsigned int Len);
uint32_t CDC_Receive_BytesAvailable(void);


#endif


/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/*-------------------------(C) COPYRIGHT 2019 MindMotion ----------------------*/
