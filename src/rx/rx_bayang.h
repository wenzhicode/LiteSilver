/**
******************************************************************************
* @file    rx_bayang.h
* @author
* @version V0.0.1
* @date    10/06/2020
* @brief   ͷ�ļ���bayang������غ�������.
******************************************************************************
*/



#include "hardware.h"


void checkrx(void);

void rx_init(void);

char checkpacket(void);

void writeregs(uint8_t data[], uint8_t size);


