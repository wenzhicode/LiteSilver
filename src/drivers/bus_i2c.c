/**
******************************************************************************
* @file    bus_i2c.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   驱动文件，i2c硬件相关函数.
******************************************************************************

开发板用的是48个pin的，i2c1可以重映射到pb8 pb9引脚上。
选用QFN32pin的时候，i2c1在pb6 pb7,飞控板初始化记得改这里。

*/



#include "bus_i2c.h"


/********************************************************************************************************
**函数信息 ：I2CInitMasterMode(I2C_TypeDef *I2Cx, unsigned int uiI2C_speed) //unit is Khz
**功能描述 ：初始化I2C
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1
**输出参数 ：无
********************************************************************************************************/
void i2c_init(I2C_TypeDef *I2Cx, unsigned int uiI2C_speed)
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //复用I2C1 IO口
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);//I2C1重映射
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9;  //I2C1重映射IO口


    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;               //保持总线空闲即CLK&DATA为高
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_Mode = I2C_Mode_MASTER;
    I2C_InitStructure.I2C_OwnAddress = 0;
    I2C_InitStructure.I2C_Speed = I2C_Speed_FAST;
    I2C_InitStructure.I2C_ClockSpeed = uiI2C_speed;

    I2C_Init(I2Cx, &I2C_InitStructure);

    //I2C超频，600khz
    I2C1->IC_FS_SCL_LCNT = 0X10;
    I2C1->IC_FS_SCL_HCNT = 0X10;


    I2C_Cmd(I2Cx, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;             //需要外加上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/********************************************************************************************************
**函数信息 ：I2CTXEmptyCheck(I2C_TypeDef *I2Cx)
**功能描述 ：检查发送中断标志位
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2
**输出参数 ：无
********************************************************************************************************/
void I2CTXEmptyCheck(I2C_TypeDef *I2Cx)
{
    while (1)
    {
        if (I2C_GetFlagStatus(I2Cx, I2C_STATUS_FLAG_TFE))
        {
            break;
        }
    }
}
/********************************************************************************************************
**函数信息 ：I2CTXEmptyCheck(I2C_TypeDef *I2Cx)
**功能描述 ：检查接收中断标志位
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2
**输出参数 ：无
********************************************************************************************************/
void I2CRXFullCheck(I2C_TypeDef *I2Cx)
{
    while (1)
    {
        if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_RX_FULL))
        {
            break;
        }
    }
}
/********************************************************************************************************
**函数信息 ：I2CTXByte(I2C_TypeDef *I2Cx,unsigned short cmd,unsigned char temp)
**功能描述 ：I2C发送数据
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2；cmd；temp
**输出参数 ：无
********************************************************************************************************/
void I2CTXByte(I2C_TypeDef *I2Cx, unsigned char temp)
{
    I2C_SendData(I2Cx, temp);
    while (I2C_GetFlagStatus(I2Cx, I2C_STATUS_FLAG_TFE) == 0);

}
/********************************************************************************************************
**函数信息 ：I2CRXByte(I2C_TypeDef *I2Cx)
**功能描述 ：I2C接收数据
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2
**输出参数 ：temp
********************************************************************************************************/
unsigned char I2CRXByte(I2C_TypeDef *I2Cx)
{
    unsigned short temp;

    I2CRXFullCheck(I2Cx);

    temp = I2C_ReceiveData(I2Cx);
    return (unsigned char)temp;
}

/********************************************************************************************************
**函数信息 ：I2CRXGroup()
**功能描述 ：I2C接收数据
**输入参数 ：I2C_TypeDef *I2Cx(I2C1,I2C2)
u16 rx_count(数据个数)
u8 *data_buf(接收数据指针)
**输出参数 ：temp
********************************************************************************************************/
void I2CRXGroup(I2C_TypeDef *I2Cx, u16 rx_count, u8 *data_buf)
{
    int i, data_cnt = 0;
    char flag = 0;
    for (i = 0; i < rx_count; i++)
    {
        I2C_ReadCmd(I2Cx);
        while (1)
        {
            if ((I2C_GetFlagStatus(I2Cx, I2C_STATUS_FLAG_TFNF) == 1) && (flag == 0)) //RX FIFO未满就写入发送指令
            {
                I2C_ReadCmd(I2Cx);
                data_cnt++;
                if (data_cnt == rx_count)
                    flag = 1;
            }
            if (I2C_GetFlagStatus(I2Cx, I2C_STATUS_FLAG_RFNE))
            {
                data_buf[i] = I2C_ReceiveData(I2Cx);
                break;
            }
        }
    }
}

/********************************************************************************************************
**函数信息 ：   I2CMasterWrite(I2C_TypeDef *I2Cx, unsigned short mem_byte_addr, unsigned short tx_count, unsigned char *tx_data )
**功能描述 ：   I2C发送数据
**输入参数 ：
I2C_TypeDef *I2Cx(I2C1,I2C2)
unsigned short mem_byte_addr(E2PROM地址)
unsigned short tx_count(数据长度)
unsigned char *tx_data(数据buf指针)
**输出参数 ：无*************************************************************************************/
void I2CMasterWrite(I2C_TypeDef *I2Cx, unsigned short mem_byte_addr, unsigned short tx_count, unsigned char *tx_data)
{
    unsigned short i;
    unsigned char *p;
    p = tx_data;

    I2CTXByte(I2Cx, mem_byte_addr);
    for (i = 0; i < tx_count; i++)
    {
        I2CTXByte(I2Cx, *p); //发送数据
        p++;
    }

    I2C_GenerateSTOP(I2Cx, ENABLE);
    while ((I2C_GetITStatus(I2Cx, I2C_IT_STOP_DET)) == 0);
}

/********************************************************************************************************
**函数信息 ：   I2CMasterRead()
**功能描述 ：   I2C接收数据
**输入参数 ：
I2C_TypeDef *I2Cx(I2C1,I2C2)
unsigned short mem_byte_addr(E2PROM地址)
unsigned char rx_count(数据长度)
unsigned char *rx_data(数据buf指针)
**输出参数 ：无
********************************************************************************************************/
void I2CMasterRead(I2C_TypeDef *I2Cx, unsigned short mem_byte_addr, unsigned short rx_count, unsigned char *rx_data)
{
    I2CMasterWrite(I2Cx, mem_byte_addr, 0, rx_data);//写设备地址
    I2CRXGroup(I2Cx, rx_count, rx_data);

    I2C_GenerateSTOP(I2Cx, ENABLE);
    while ((I2C_GetITStatus(I2Cx, I2C_IT_STOP_DET)) == 0);
}

/********************************************************************************************************
**函数信息 ：I2CSetDeviceAddr(I2C_TypeDef *I2Cx,unsigned char deviceaddr)
**功能描述 ：选择从机地址
**输入参数 ：I2C_TypeDef *I2Cx，选择I2C1,I2C2;deviceaddr:从机地址
**输出参数 ：无
********************************************************************************************************/
void I2CSetDeviceAddr(I2C_TypeDef *I2Cx, unsigned char deviceaddr)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    if (I2Cx == I2C1)
    {
        GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9;  //I2C1重映射IO口
    }
    if (I2Cx == I2C2)
    {
        GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10 | GPIO_Pin_11;
    }

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_Cmd(I2Cx, DISABLE);
    I2C_Send7bitAddress(I2Cx, deviceaddr << 1, I2C_Direction_Transmitter);
    I2C_Cmd(I2Cx, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}






