/**
******************************************************************************
* @file    flash_io.c
* @author
* @version V0.0.1
* @date    30/06/2020
* @brief   �����ļ���flash io��غ���.
******************************************************************************
*/




#include "flash_io.h"


extern void failloop(int);




/**************************************************************************
**������Ϣ ��writeword( unsigned long address, unsigned long value)
**�������� ��дһ��word
**������� ��address ��ַ  value ֵ
**������� ����
**************************************************************************/
void writeword(unsigned long address, unsigned long value)
{
    int test = FLASH_ProgramWord(FLASH_ADDR + (address << 2), value);
    if (test != FLASH_COMPLETE)
    {
        FLASH_Lock();
        failloop(5);
    }
}


/**************************************************************************
**������Ϣ ��fmc_read(unsigned long address)
**�������� ����һ��word
**������� ��address ��ַ
**������� ��value ֵ
**************************************************************************/
unsigned long fmc_read(unsigned long address)
{
    address = address * 4 + FLASH_ADDR;
    unsigned int *addressptr = (unsigned int *)address;
    return (*addressptr);
}


/**************************************************************************
**������Ϣ ��fmc_erase( void )
**�������� ������һ page
**������� ����
**������� ������ �Ƿ�ɹ�
**************************************************************************/
int fmc_erase(void)
{
    int test = FLASH_ErasePage(FLASH_ADDR);
    if (test != FLASH_COMPLETE) FLASH_Lock();
    else return 0;
    return 1;
}


/**************************************************************************
**������Ϣ ��fmc_write_float��fmc_read_float
**�������� ����д ������
**������� ��
**������� ��
**************************************************************************/
void fmc_write_float(unsigned long address, float float_to_write)
{
    writeword(address, *(unsigned long *) &float_to_write);
}

float fmc_read_float(unsigned long address)
{
    unsigned long result = fmc_read(address);
    return *(float *)&result;
}



