/**
******************************************************************************
* @file    blackbox_encoding.h
* @author
* @version V0.0.1
* @date    4/06/2020
* @brief   ͷ�ļ���blackbox��غ�������.
******************************************************************************
*/


#include "hardware.h"







void blackboxWrite(uint8_t value);


void blackboxWriteUnsignedVB(uint32_t value);


void blackboxWriteSigned16VBArray(int16_t *array, int count);


void blackboxWriteSignedVB(int32_t value);




