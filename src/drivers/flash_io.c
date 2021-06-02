/**
******************************************************************************
* @file    flash_io.c
* @author
* @version V0.0.1
* @date    30/06/2020
* @brief   驱动文件，flash io相关函数.
******************************************************************************
*/




#include "flash_io.h"


extern void failloop(int);




/**************************************************************************
**函数信息 ：writeword( unsigned long address, unsigned long value)
**功能描述 ：写一个word
**输入参数 ：address 地址  value 值
**输出参数 ：无
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
**函数信息 ：fmc_read(unsigned long address)
**功能描述 ：读一个word
**输入参数 ：address 地址
**输出参数 ：value 值
**************************************************************************/
unsigned long fmc_read(unsigned long address)
{
    address = address * 4 + FLASH_ADDR;
    unsigned int *addressptr = (unsigned int *)address;
    return (*addressptr);
}


/**************************************************************************
**函数信息 ：fmc_erase( void )
**功能描述 ：擦除一 page
**输入参数 ：无
**输出参数 ：擦除 是否成功
**************************************************************************/
int fmc_erase(void)
{
    int test = FLASH_ErasePage(FLASH_ADDR);
    if (test != FLASH_COMPLETE) FLASH_Lock();
    else return 0;
    return 1;
}


/**************************************************************************
**函数信息 ：fmc_write_float、fmc_read_float
**功能描述 ：读写 浮点数
**输入参数 ：
**输出参数 ：
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



