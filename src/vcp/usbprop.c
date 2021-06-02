/**
******************************************************************************
* @file    usbprop.c
* @author  AE Team
* @version V1.1.0
* @date    28/08/2019
* @brief   This file provides all the usbprop firmware functions.
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, MindMotion SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2019 MindMotion</center></h2>
*/
#include "usbprop.h"
#include "usb.h"
#include "HAL_device.h"
#include "time.h"

unsigned char *pucClassDrcData;


//���⴮�ڲ���
LINE_CODING linecoding =
{
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* no. of bits 8*/
};

__IO uint32_t APP_Tx_ptr_out = 0;
__IO uint32_t APP_Tx_ptr_in = 0;

__IO  uint32_t APP_Rx_ptr_in  = 0;
__IO uint32_t APP_Rx_ptr_out = 0;
__IO uint32_t APP_Rx_length  = 0;

#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024

uint8_t buffer_out[APP_TX_DATA_SIZE];
uint8_t buffer_in[APP_RX_DATA_SIZE];

/********************************************************************************************************
**������Ϣ ��USBCDC_senddata(unsigned char *data,unsigned short length)
**�������� ������ת��
**������� ��unsigned char *data(ת��������ָ��),unsigned short length(ת�����ݳ���)
**������� ��
**��    ע ��
********************************************************************************************************/
void USBCDC_senddata(unsigned char *data, unsigned short length)
{
    if (linecoding.datatype == 7)
    {
        while (length)
        {
            *data &= 0x7f;
            length--;
            data++;
        }
    }
    else if (linecoding.datatype == 8)                          //������
    {
        while (length)
        {

        }
    }
}

/********************************************************************************************************
**������Ϣ ��Class_Get_Line_Coding(void)
**�������� ��CDC Abstract Control Model(���⴮��) ������GET_LINE_CODING
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/
void Class_Get_Line_Coding(void)                                // bRequest = 0x21
{
    int count;
    switch (usb_running_ctrl_state)
    {
    case    USB_CTRL_SETUP :
        if (req_dir == USB_REQ_DIR_IN)
        {
            pucClassDrcData = (u8 *)&linecoding;
            usb_running_ctrl_state = USB_CTRL_IN;
        }
        else
        {
            usb_running_ctrl_state = USB_IDLE;
        }
        break ;

    case    USB_CTRL_IN :

        while (USB->rEP0_CTRL & 0x80);
        for (count = 0; count < 7; count++)
        {
            USB->rEP0_FIFO = *pucClassDrcData;
            pucClassDrcData ++;
        }
        USB->rEP0_CTRL = 0x87;

        usb_running_ctrl_state = USB_IDLE;
        break ;

    default :
        //      usb_running_ctrl_state = USB_IDLE ;
        break ;
    }
}
unsigned char baud_read [64];
/********************************************************************************************************
**������Ϣ ��Class_Get_Line_Coding(void)
**�������� ��CDC Abstract Control Model(���⴮��) ������SET_LINE_CODING
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/
void Class_Set_Line_Coding(void)  // bRequest = 0x20
{
    int count, i;

    switch (usb_running_ctrl_state)
    {
    case    USB_CTRL_SETUP :
        if (req_dir == USB_REQ_DIR_OUT)
        {
            usb_running_ctrl_state = USB_CTRL_OUT;
        }
        else
        {
            usb_running_ctrl_state = USB_IDLE;
        }
        break ;

    case    USB_CTRL_OUT :
        count = USB->rEP0_AVIL;
        for (i = 0; i < count ; i++)
        {
            baud_read[i] = USB->rEP0_FIFO;
        }
        linecoding.bitrate = ((u32)baud_read[0] << 0) | ((u32)baud_read[1] << 8) | ((u32)baud_read[2] << 16) | ((u32)baud_read[3] << 24);

        usb_running_ctrl_state = USB_CTRL_STATUS;
        break;

    default :
        //          usb_running_ctrl_state = USB_IDLE ;
        break ;
    }
}
/********************************************************************************************************
**������Ϣ ��Class_Get_Line_Coding(void)
**�������� ��CDC Abstract Control Model(���⴮��) ������SET_LINE_CODING
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/

void Class_Set_Control_Line_State(void)                     // bRequest = 0x22
{
    int count, i;
    unsigned char state_temp = 0;
    state_temp = state_temp;
    switch (usb_running_ctrl_state)
    {
    case    USB_CTRL_SETUP :                            //���ƴ���״̬�׶�
        if (req_dir == USB_REQ_DIR_OUT)
        {
            usb_running_ctrl_state = USB_CTRL_OUT;
        }
        else
        {
            usb_running_ctrl_state = USB_IDLE;
        }
        break ;
    case    USB_CTRL_OUT :
        count = USB->rEP0_AVIL;
        for (i = 0; i < count ; i++)
        {
            state_temp = USB->rEP0_FIFO;                //�˴�Ϊ������(���ƴ������ݽ׶�)
        }
        usb_running_ctrl_state = USB_CTRL_STATUS;
        break;

    default :
    {
        //          usb_running_ctrl_state = USB_IDLE ;
        break ;
    }
    }
}

unsigned char ep1_tx_flag = 0;
/********************************************************************************************************
**������Ϣ ��EP1_USB_IN_Data(void)
**�������� ��USB�ӷ��ͻ�����������ͨ��IN�˵㷢��
**������� ��
**������� ��
**��    ע ��USB���ݷ���ʱ�����ݳ������˵�sizeʱ����Ҫ�������
********************************************************************************************************/
void EP1_USB_IN_Data(void)                  // bRequest = 0x22
{
    int count, i;

    if(ep1_tx_flag==0)
        return;

    if (APP_Rx_ptr_in != APP_Rx_ptr_out)
    {
        count = APP_RX_DATA_SIZE - ((APP_Rx_ptr_out - APP_Rx_ptr_in) + (-((int)(APP_Rx_ptr_out <= APP_Rx_ptr_in)) & APP_RX_DATA_SIZE));
        while (USB->rEP1_CTRL & 0x80);
        for (i = 0; i < count; i++)
        {
            USB->rEP1_FIFO = *(buffer_in + APP_Rx_ptr_out);
            APP_Rx_ptr_out = (APP_Rx_ptr_out + 1) % APP_RX_DATA_SIZE;
        }

        ep1_tx_flag = 0;

        USB->rEP1_CTRL = 0x80 | count;
    }

}
/********************************************************************************************************
**������Ϣ ��UsbVcomSend(unsigned char*Info,unsigned int *infoLenth)
**�������� ����Ҫ���͵�����д��USB���ͻ�����
**������� ��unsigned char*Info(��������ָ��),unsigned int *infoLenth(�������ݳ���ָ��)
**������� ��return status(0��ʾ�������,1��ʾ���ڷ���);
**��    ע ��
********************************************************************************************************/
uint32_t CDC_Send_FreeBytes(void)
{
    /*
        return the bytes free in the circular buffer

        functionally equivalent to:
        (APP_Rx_ptr_out > APP_Rx_ptr_in ? APP_Rx_ptr_out - APP_Rx_ptr_in : APP_RX_DATA_SIZE - APP_Rx_ptr_in + APP_Rx_ptr_in)
        but without the impact of the condition check.
    */
    return ((APP_Rx_ptr_out - APP_Rx_ptr_in) + (-((int)(APP_Rx_ptr_out <= APP_Rx_ptr_in)) & APP_RX_DATA_SIZE)) - 1;
}


char UsbVcomSend(const unsigned char *Info, unsigned int Len)
{
    uint32_t i;

    if((ep1_tx_flag ==0) && Len !=0 )
    {
        for (i = 0; i < Len; i++)
        {
            buffer_in[APP_Rx_ptr_in] = Info[i];
            APP_Rx_ptr_in = (APP_Rx_ptr_in + 1) % APP_RX_DATA_SIZE;

            while (CDC_Send_FreeBytes() == 0)
            {
                delay(1);
            }
        }
        ep1_tx_flag = 1;
    }

    return 0;
}
unsigned char ep3_rx_flag = 0;              //�˵�3���ձ�־
/********************************************************************************************************
**������Ϣ ��EP3_USB_OUT_Data(void)
**�������� ��USB_OUT�˵���յ���Ч����,���ձ�־λ��1
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/
void EP3_USB_OUT_Data(void)                 // bRequest = 0x22
{
    u16  tempLen = 0;
    uint32_t i;

    tempLen = USB->rEP3_AVIL & 0x7f;//������64�ֽڻ�ִ�ж˵㺯��

    for (i = tempLen; i != 0; i--)
    {
        *(buffer_out + APP_Tx_ptr_in) = USB->rEP3_FIFO;
        APP_Tx_ptr_in = (APP_Tx_ptr_in + 1) % APP_TX_DATA_SIZE;
    }

    usb_running_ctrl_state = USB_IDLE;
}

uint32_t CDC_Receive_BytesAvailable(void)
{
    /* return the bytes available in the receive circular buffer */
    return APP_Tx_ptr_out > APP_Tx_ptr_in ? APP_TX_DATA_SIZE - APP_Tx_ptr_out + APP_Tx_ptr_in : APP_Tx_ptr_in - APP_Tx_ptr_out;
}

/********************************************************************************************************
**������Ϣ ��UsbVcomRec(unsigned char*rxInfo)
**�������� ����USB device�� OUT�˵��������
**������� ��unsigned char*rxInfo(������Ϣbuf)
**������� ��
**��    ע ��
********************************************************************************************************/
unsigned int UsbVcomRec(unsigned char *rxInfo, int len)
{
    uint32_t count = 0;

    while (APP_Tx_ptr_out != APP_Tx_ptr_in && count < len)
    {
        rxInfo[count] = buffer_out[APP_Tx_ptr_out];
        APP_Tx_ptr_out = (APP_Tx_ptr_out + 1) % APP_TX_DATA_SIZE;
        count++;
    }
    return count;
}

