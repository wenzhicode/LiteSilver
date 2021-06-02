/**
******************************************************************************
* @file    blackbox_encoding.c
* @author
* @version V0.0.1
* @date    4/06/2020
* @brief   blackbox文件，blackbox相关函数.
******************************************************************************
*/


#include "blackbox_encoding.h"




uint8_t openLogBuff[20] = {1, 2, 3};
uint16_t bufNum = 0;



void blackboxWrite(uint8_t value)
{
    openLogBuff[bufNum++] = value;
}


void blackboxWriteUnsignedVB(uint32_t value)
{
    //While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127)
    {
        blackboxWrite((uint8_t)(value | 0x80));  // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    blackboxWrite(value);
}

uint32_t zigzagEncode(int32_t value)
{
    return (uint32_t)((value << 1) ^ (value >> 31));
}


/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
void blackboxWriteSignedVB(int32_t value)
{
    //ZigZag encode to make the value always positive
    blackboxWriteUnsignedVB(zigzagEncode(value));
}



void blackboxWriteSigned16VBArray(int16_t *array, int count)
{
    for (int i = 0; i < count; i++)
    {
        blackboxWriteSignedVB(array[i]);
    }
}




