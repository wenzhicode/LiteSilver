/**
******************************************************************************
* @file    bus_i2c.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   �����ļ���i2cӲ����غ���.
******************************************************************************

�������õ���48��pin�ģ�i2c1������ӳ�䵽pb8 pb9�����ϡ�
ѡ��QFN32pin��ʱ��i2c1��pb6 pb7,�ɿذ��ʼ���ǵø����

*/



#include "bus_i2c.h"


/********************************************************************************************************
**������Ϣ ��I2CInitMasterMode(I2C_TypeDef *I2Cx, unsigned int uiI2C_speed) //unit is Khz
**�������� ����ʼ��I2C
**������� ��I2C_TypeDef *I2Cx��ѡ��I2C1
**������� ����
********************************************************************************************************/
void i2c_init(I2C_TypeDef *I2Cx, unsigned int uiI2C_speed)
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //����I2C1 IO��
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);//I2C1��ӳ��
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9;  //I2C1��ӳ��IO��


    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;               //�������߿��м�CLK&DATAΪ��
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_Mode = I2C_Mode_MASTER;
    I2C_InitStructure.I2C_OwnAddress = 0;
    I2C_InitStructure.I2C_Speed = I2C_Speed_FAST;
    I2C_InitStructure.I2C_ClockSpeed = uiI2C_speed;

    I2C_Init(I2Cx, &I2C_InitStructure);

    //I2C��Ƶ��600khz
    I2C1->IC_FS_SCL_LCNT = 0X10;
    I2C1->IC_FS_SCL_HCNT = 0X10;


    I2C_Cmd(I2Cx, ENABLE);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;             //��Ҫ�������
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/********************************************************************************************************
**������Ϣ ��I2CTXEmptyCheck(I2C_TypeDef *I2Cx)
**�������� ����鷢���жϱ�־λ
**������� ��I2C_TypeDef *I2Cx��ѡ��I2C1,I2C2
**������� ����
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
**������Ϣ ��I2CTXEmptyCheck(I2C_TypeDef *I2Cx)
**�������� ���������жϱ�־λ
**������� ��I2C_TypeDef *I2Cx��ѡ��I2C1,I2C2
**������� ����
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
**������Ϣ ��I2CTXByte(I2C_TypeDef *I2Cx,unsigned short cmd,unsigned char temp)
**�������� ��I2C��������
**������� ��I2C_TypeDef *I2Cx��ѡ��I2C1,I2C2��cmd��temp
**������� ����
********************************************************************************************************/
void I2CTXByte(I2C_TypeDef *I2Cx, unsigned char temp)
{
    I2C_SendData(I2Cx, temp);
    while (I2C_GetFlagStatus(I2Cx, I2C_STATUS_FLAG_TFE) == 0);

}
/********************************************************************************************************
**������Ϣ ��I2CRXByte(I2C_TypeDef *I2Cx)
**�������� ��I2C��������
**������� ��I2C_TypeDef *I2Cx��ѡ��I2C1,I2C2
**������� ��temp
********************************************************************************************************/
unsigned char I2CRXByte(I2C_TypeDef *I2Cx)
{
    unsigned short temp;

    I2CRXFullCheck(I2Cx);

    temp = I2C_ReceiveData(I2Cx);
    return (unsigned char)temp;
}

/********************************************************************************************************
**������Ϣ ��I2CRXGroup()
**�������� ��I2C��������
**������� ��I2C_TypeDef *I2Cx(I2C1,I2C2)
u16 rx_count(���ݸ���)
u8 *data_buf(��������ָ��)
**������� ��temp
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
            if ((I2C_GetFlagStatus(I2Cx, I2C_STATUS_FLAG_TFNF) == 1) && (flag == 0)) //RX FIFOδ����д�뷢��ָ��
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
**������Ϣ ��   I2CMasterWrite(I2C_TypeDef *I2Cx, unsigned short mem_byte_addr, unsigned short tx_count, unsigned char *tx_data )
**�������� ��   I2C��������
**������� ��
I2C_TypeDef *I2Cx(I2C1,I2C2)
unsigned short mem_byte_addr(E2PROM��ַ)
unsigned short tx_count(���ݳ���)
unsigned char *tx_data(����bufָ��)
**������� ����*************************************************************************************/
void I2CMasterWrite(I2C_TypeDef *I2Cx, unsigned short mem_byte_addr, unsigned short tx_count, unsigned char *tx_data)
{
    unsigned short i;
    unsigned char *p;
    p = tx_data;

    I2CTXByte(I2Cx, mem_byte_addr);
    for (i = 0; i < tx_count; i++)
    {
        I2CTXByte(I2Cx, *p); //��������
        p++;
    }

    I2C_GenerateSTOP(I2Cx, ENABLE);
    while ((I2C_GetITStatus(I2Cx, I2C_IT_STOP_DET)) == 0);
}

/********************************************************************************************************
**������Ϣ ��   I2CMasterRead()
**�������� ��   I2C��������
**������� ��
I2C_TypeDef *I2Cx(I2C1,I2C2)
unsigned short mem_byte_addr(E2PROM��ַ)
unsigned char rx_count(���ݳ���)
unsigned char *rx_data(����bufָ��)
**������� ����
********************************************************************************************************/
void I2CMasterRead(I2C_TypeDef *I2Cx, unsigned short mem_byte_addr, unsigned short rx_count, unsigned char *rx_data)
{
    I2CMasterWrite(I2Cx, mem_byte_addr, 0, rx_data);//д�豸��ַ
    I2CRXGroup(I2Cx, rx_count, rx_data);

    I2C_GenerateSTOP(I2Cx, ENABLE);
    while ((I2C_GetITStatus(I2Cx, I2C_IT_STOP_DET)) == 0);
}

/********************************************************************************************************
**������Ϣ ��I2CSetDeviceAddr(I2C_TypeDef *I2Cx,unsigned char deviceaddr)
**�������� ��ѡ��ӻ���ַ
**������� ��I2C_TypeDef *I2Cx��ѡ��I2C1,I2C2;deviceaddr:�ӻ���ַ
**������� ����
********************************************************************************************************/
void I2CSetDeviceAddr(I2C_TypeDef *I2Cx, unsigned char deviceaddr)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    if (I2Cx == I2C1)
    {
        GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9;  //I2C1��ӳ��IO��
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






