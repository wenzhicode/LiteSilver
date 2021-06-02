/**
******************************************************************************
* @file    item.h
* @author
* @version V0.0.1
* @date    1/07/2020
* @brief   ͷ�ļ���osd item��غ�������.
******************************************************************************
*/


#pragma once

#include "hardware.h"


#pragma anon_unions

typedef struct menu
{
    union
    {
        unsigned char uvalue;
        float fvalue;
    };
    char index;
    char item;
    struct menu *next;
    struct menu *prior;
} menu_node, *menu_list;

menu_list createMenu(char len, char item);





