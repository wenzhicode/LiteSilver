/**
******************************************************************************
* @file    serial_uart.h
* @author  wz
* @version V0.0.1
* @date    25/05/2020
* @brief   串口头文件，串口相关函数声明.
******************************************************************************
*/

#include "MM32F103.h"

#ifndef __SERIAL_UART_H
#define __SERIAL_UART_H

typedef struct
{
    uint8_t                  *pTxBuffPtr;      /*!< Pointer to UART Tx transfer Buffer */
    uint16_t                 TxXferSize;       /*!< UART Tx Transfer size              */
    __IO uint16_t            TxXferCount;      /*!< UART Tx Transfer Counter           */
    uint8_t                  *pRxBuffPtr;      /*!< Pointer to UART Rx transfer Buffer */
    uint16_t                 RxXferSize;       /*!< UART Rx Transfer size              */
    __IO uint16_t            RxXferCount;      /*!< UART Rx Transfer Counter           */
} uartPort_t;


void uart2_initwBaudRate(u32 bound);
void uart1_initwBaudRate(u32 bound);

void UART2_DMA_Send(void);

void Usart1DmaPrintf(const char *format, ...);

void UART1_DMA_Send(uint8_t len);

#endif

