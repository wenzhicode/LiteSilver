/**
******************************************************************************
* @file    serial_uart.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   �����ļ�������Ӳ����غ���.
******************************************************************************
*/

#include "MM32F103.h"
#include "SYSTEM_MM32F103.h"
#include "hardware.h"
#include "serial_uart.h"
#include "stdio.h"
#include "stdarg.h"


char UartTxBuf[128];

extern uint8_t openLogBuff[20];
extern uint16_t bufNum;

#define SERIAL_BAUDRATE 100000

u8 UART_RX_BUF[10] = {0};
u8 curUartRxLenth = 0;


DMA_InitTypeDef DMA_InitStructure;

#if 0
/*************************************************************************
**������Ϣ ��uart1_initwBaudRate(u32 bound)
**�������� ������1��ʼ
**������� ��bound ������
**������� ����
**************************************************************************/
void uart1_initwBaudRate(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //ʹ��UART1��GPIOAʱ��


    //UART ��ʼ������
    UART_InitStructure.UART_BaudRate = bound;//���ڲ�����
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    UART_InitStructure.UART_StopBits = UART_StopBits_1;//һ��ֹͣλ
    UART_InitStructure.UART_Parity = UART_Parity_No;//����żУ��λ
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//��Ӳ������������
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx; //�շ�ģʽ

    UART_Init(UART1, &UART_InitStructure); //��ʼ������1
    UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);//�������ڽ����ж�
    UART_Cmd(UART1, ENABLE);                    //ʹ�ܴ���1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //UART1_RX    GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10

    //UART1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���

    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART1->TDR;  //DMA�������ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)openLogBuff;  //DMA�ڴ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
    //DMA_InitStructure.DMA_BufferSize = 16;  //DMAͨ����DMA����Ĵ�С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��UART1_Tx_DMA_Channel����ʶ�ļĴ���

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /* Enable the UARTy_DMA1_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable UARTy_DMA1_Channel Transfer complete interrupt */
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    UART_DMACmd(UART1, UART_DMAReq_EN, ENABLE);
    /* UARTy_DMA1_Channel enable */
    DMA_Cmd(DMA1_Channel4, DISABLE);

}


/*************************************************************************
**������Ϣ ��DMA1_Channel7_IRQHandler(void)
**�������� ��DMAͨ��7��������ж�
**������� ����
**������� ����
**************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC4) == SET)
    {
        DMA_ClearFlag(DMA1_FLAG_TC4);
    }
}

/**************************************************************************
**������Ϣ ��UART2_DMA_Send(void)
**�������� ����ʼһ��DMA����
**������� ����
**������� ����
***************************************************************************/

void UART1_DMA_Send(uint8_t len)
{
    DMA_Cmd(DMA1_Channel4, DISABLE);

    DMA1_Channel4->CPAR = (u32)&UART1->TDR;
    DMA1_Channel4->CMAR = (u32)UartTxBuf;
    DMA1_Channel4->CNDTR = len;

    DMA_Cmd(DMA1_Channel4, ENABLE);

}


void Usart1DmaPrintf(const char *format, ...)
{
    uint16_t len;
    va_list args;
    va_start(args, format);
    len = vsnprintf((char *)UartTxBuf, sizeof(UartTxBuf) + 1, (char *)format, args);
    va_end(args);
    UART1_DMA_Send(len);
}

#endif
/*************************************************************************
**������Ϣ ��uart2_initwBaudRate(u32 bound)
**�������� ������2��ʼ
**������� ��bound ������
**������� ����
**************************************************************************/
void uart2_initwBaudRate(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //ʹ��UART2��GPIOAʱ��
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //ʹ��DMA����

    //UART ��ʼ������
    UART_InitStructure.UART_BaudRate = bound;//���ڲ�����
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    UART_InitStructure.UART_StopBits = UART_StopBits_1;//һ��ֹͣλ
    UART_InitStructure.UART_Parity = UART_Parity_No;//����żУ��λ
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//��Ӳ������������
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx; //�շ�ģʽ

    UART_Init(UART2, &UART_InitStructure); //��ʼ������2
    UART_ITConfig(UART2, UART_IT_RXIEN, ENABLE);//�������ڽ����ж�
    UART_Cmd(UART2, ENABLE);                    //ʹ�ܴ���2

    //UART2_TX   GPIOA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2

    //UART2_RX    GPIOA.3��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3

    //UART1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���

    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART2->TDR;  //DMA�������ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)openLogBuff;  //DMA�ڴ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
    //DMA_InitStructure.DMA_BufferSize = 16;  //DMAͨ����DMA����Ĵ�С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��UART1_Tx_DMA_Channel����ʶ�ļĴ���

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /* Enable the UARTy_DMA1_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable UARTy_DMA1_Channel Transfer complete interrupt */
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);

    UART_DMACmd(UART2, UART_DMAReq_EN, ENABLE);
    /* UARTy_DMA1_Channel enable */
    DMA_Cmd(DMA1_Channel7, DISABLE);

}

/*************************************************************************
**������Ϣ ��UART2_IRQHandler(void)
**�������� ������2�ж�
**������� ����
**������� ����
**************************************************************************/
void UART2_IRQHandler(void)
{
    if (UART_GetITStatus(UART2, UART_IT_RXIEN)  != RESET)
    {
        UART_ClearITPendingBit(UART2, UART_IT_RXIEN);
        UART_RX_BUF[curUartRxLenth] = UART2->RDR;
    }
}

/*************************************************************************
**������Ϣ ��DMA1_Channel7_IRQHandler(void)
**�������� ��DMAͨ��7��������ж�
**������� ����
**������� ����
**************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC7) == SET)
    {
        DMA_ClearFlag(DMA1_FLAG_TC7);
    }
}

/**************************************************************************
**������Ϣ ��UART2_DMA_Send(void)
**�������� ����ʼһ��DMA����
**������� ����
**������� ����
***************************************************************************/

void UART2_DMA_Send(void)
{
    DMA_Cmd(DMA1_Channel7, DISABLE);

    DMA1_Channel7->CPAR = (u32)&UART2->TDR;
    DMA1_Channel7->CMAR = (u32)openLogBuff;
    DMA1_Channel7->CNDTR = 13;

    DMA_Cmd(DMA1_Channel7, ENABLE);
}


#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;

};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
//__use_no_semihosting was requested, but _ttywrch was
void _ttywrch(int ch)
{
    ch = ch;
}
void _sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while ((UART1->CSR & UART_IT_TXIEN) == 0); //ѭ������,ֱ���������
    UART1->TDR = (ch & (uint16_t)0x00FF);
    return ch;
}

