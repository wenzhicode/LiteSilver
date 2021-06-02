/**
******************************************************************************
* @file    serial_uart.c
* @author
* @version V0.0.1
* @date    25/05/2020
* @brief   驱动文件，串口硬件相关函数.
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
**函数信息 ：uart1_initwBaudRate(u32 bound)
**功能描述 ：串口1初始
**输入参数 ：bound 波特率
**输出参数 ：无
**************************************************************************/
void uart1_initwBaudRate(u32 bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //使能UART1，GPIOA时钟


    //UART 初始化设置
    UART_InitStructure.UART_BaudRate = bound;//串口波特率
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;//字长为8位数据格式
    UART_InitStructure.UART_StopBits = UART_StopBits_1;//一个停止位
    UART_InitStructure.UART_Parity = UART_Parity_No;//无奇偶校验位
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//无硬件数据流控制
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx; //收发模式

    UART_Init(UART1, &UART_InitStructure); //初始化串口1
    UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);//开启串口接收中断
    UART_Cmd(UART1, ENABLE);                    //使能串口1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //UART1_RX    GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10

    //UART1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器

    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART1->TDR;  //DMA外设基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)openLogBuff;  //DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
    //DMA_InitStructure.DMA_BufferSize = 16;  //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道UART1_Tx_DMA_Channel所标识的寄存器

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
**函数信息 ：DMA1_Channel7_IRQHandler(void)
**功能描述 ：DMA通道7传输完成中断
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC4) == SET)
    {
        DMA_ClearFlag(DMA1_FLAG_TC4);
    }
}

/**************************************************************************
**函数信息 ：UART2_DMA_Send(void)
**功能描述 ：开始一次DMA传输
**输入参数 ：无
**输出参数 ：无
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
**函数信息 ：uart2_initwBaudRate(u32 bound)
**功能描述 ：串口2初始
**输入参数 ：bound 波特率
**输出参数 ：无
**************************************************************************/
void uart2_initwBaudRate(u32 bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    UART_InitTypeDef UART_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //使能UART2，GPIOA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //使能DMA传输

    //UART 初始化设置
    UART_InitStructure.UART_BaudRate = bound;//串口波特率
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;//字长为8位数据格式
    UART_InitStructure.UART_StopBits = UART_StopBits_1;//一个停止位
    UART_InitStructure.UART_Parity = UART_Parity_No;//无奇偶校验位
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;//无硬件数据流控制
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx; //收发模式

    UART_Init(UART2, &UART_InitStructure); //初始化串口2
    UART_ITConfig(UART2, UART_IT_RXIEN, ENABLE);//开启串口接收中断
    UART_Cmd(UART2, ENABLE);                    //使能串口2

    //UART2_TX   GPIOA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2

    //UART2_RX    GPIOA.3初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3

    //UART1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0 ; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器

    DMA_DeInit(DMA1_Channel7);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART2->TDR;  //DMA外设基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)openLogBuff;  //DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
    //DMA_InitStructure.DMA_BufferSize = 16;  //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道UART1_Tx_DMA_Channel所标识的寄存器

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
**函数信息 ：UART2_IRQHandler(void)
**功能描述 ：串口2中断
**输入参数 ：无
**输出参数 ：无
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
**函数信息 ：DMA1_Channel7_IRQHandler(void)
**功能描述 ：DMA通道7传输完成中断
**输入参数 ：无
**输出参数 ：无
**************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC7) == SET)
    {
        DMA_ClearFlag(DMA1_FLAG_TC7);
    }
}

/**************************************************************************
**函数信息 ：UART2_DMA_Send(void)
**功能描述 ：开始一次DMA传输
**输入参数 ：无
**输出参数 ：无
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
//标准库需要的支持函数
struct __FILE
{
    int handle;

};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
//__use_no_semihosting was requested, but _ttywrch was
void _ttywrch(int ch)
{
    ch = ch;
}
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while ((UART1->CSR & UART_IT_TXIEN) == 0); //循环发送,直到发送完毕
    UART1->TDR = (ch & (uint16_t)0x00FF);
    return ch;
}

