/**
******************************************************************************
* @file    cc2500.c
* @author
* @version V0.0.1
* @date    22/06/2020
* @brief   �����ļ���cc2500Ӳ����غ���.
******************************************************************************
*/



#include "cc2500.h"
#include "bus_spi.h"
#include "time.h"
#include "rx_spi.h"
#include "defines.h"

uint8_t calData[255][3];
uint32_t timeTunedMs;
extern int8_t bindOffset;
uint8_t listLength;
uint8_t bindIdx;
extern rx_spi_protocol_e spiProtocol;

rxCc2500SpiConfig_t rxCc2500SpiConfigMutable =
{
    .autoBind = true,
    .bindTxId = {0, 0, 0},
    .bindOffset = 0,
    .bindHopData = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    },
    .rxNum = 0,
    .a1Source = 0,
    .chipDetectEnabled = 0,
    .txEnIoTag = 0,
    .lnaEnIoTag = 0,
    .antSelIoTag = 0,
};


const cc2500RegisterConfigElement_t cc2500FrskyBaseConfig[] =
{
    { CC2500_02_IOCFG0,   0x01 },
    { CC2500_18_MCSM0,    0x18 },
    { CC2500_07_PKTCTRL1, 0x05 },
    { CC2500_3E_PATABLE,  0xFF },
    { CC2500_0C_FSCTRL0,  0x00 },
    { CC2500_0D_FREQ2,    0x5C },
    { CC2500_13_MDMCFG1,  0x23 },
    { CC2500_14_MDMCFG0,  0x7A },
    { CC2500_19_FOCCFG,   0x16 },
    { CC2500_1A_BSCFG,    0x6C },
    { CC2500_1B_AGCCTRL2, 0x03 },
    { CC2500_1C_AGCCTRL1, 0x40 },
    { CC2500_1D_AGCCTRL0, 0x91 },
    { CC2500_21_FREND1,   0x56 },
    { CC2500_22_FREND0,   0x10 },
    { CC2500_23_FSCAL3,   0xA9 },
    { CC2500_24_FSCAL2,   0x0A },
    { CC2500_25_FSCAL1,   0x00 },
    { CC2500_26_FSCAL0,   0x11 },
    { CC2500_29_FSTEST,   0x59 },
    { CC2500_2C_TEST2,    0x88 },
    { CC2500_2D_TEST1,    0x31 },
    { CC2500_2E_TEST0,    0x0B },
    { CC2500_03_FIFOTHR,  0x07 },
    { CC2500_09_ADDR,     0x00 }
};

const cc2500RegisterConfigElement_t cc2500FrskyDConfig[] =
{
    { CC2500_17_MCSM1,    0x0C },
    { CC2500_0E_FREQ1,    0x76 },
    { CC2500_0F_FREQ0,    0x27 },
    { CC2500_06_PKTLEN,   0x19 },
    { CC2500_08_PKTCTRL0, 0x05 },
    { CC2500_0B_FSCTRL1,  0x08 },
    { CC2500_10_MDMCFG4,  0xAA },
    { CC2500_11_MDMCFG3,  0x39 },
    { CC2500_12_MDMCFG2,  0x11 },
    { CC2500_15_DEVIATN,  0x42 }
};

const cc2500RegisterConfigElement_t cc2500FrskyXConfig[] =
{
    { CC2500_17_MCSM1,    0x0C },
    { CC2500_0E_FREQ1,    0x76 },
    { CC2500_0F_FREQ0,    0x27 },
    { CC2500_06_PKTLEN,   0x1E },
    { CC2500_08_PKTCTRL0, 0x01 },
    { CC2500_0B_FSCTRL1,  0x0A },
    { CC2500_10_MDMCFG4,  0x7B },
    { CC2500_11_MDMCFG3,  0x61 },
    { CC2500_12_MDMCFG2,  0x13 },
    { CC2500_15_DEVIATN,  0x51 }
};

const cc2500RegisterConfigElement_t cc2500FrskyXLbtConfig[] =
{
    { CC2500_17_MCSM1,    0x0E },
    { CC2500_0E_FREQ1,    0x80 },
    { CC2500_0F_FREQ0,    0x00 },
    { CC2500_06_PKTLEN,   0x23 },
    { CC2500_08_PKTCTRL0, 0x01 },
    { CC2500_0B_FSCTRL1,  0x08 },
    { CC2500_10_MDMCFG4,  0x7B },
    { CC2500_11_MDMCFG3,  0xF8 },
    { CC2500_12_MDMCFG2,  0x03 },
    { CC2500_15_DEVIATN,  0x53 }
};


u8 cc2500_readReg(u8 reg)
{
    u8 data;
//    mpu6000_CSH;
//    spl06_CSH;
    SPIM_CSLow();//CS������ΪSPIģʽ
    SPI_RW(reg | 0x80); //���ͼĴ�����ַ+������
    data = SPI_RW(0xff);
    SPIM_CSHigh();
    return data;
}

void cc2500_writeReg(u8 REG, u8 DATA)
{
//    mpu6000_CSH;
//    spl06_CSH;
    SPIM_CSLow();
    SPI_RW(REG & 0x7f); //���ͼĴ�����ַ+д����
    SPI_RW(DATA);
    SPIM_CSHigh();
}

void cc2500_write(u8 DATA)
{
//    mpu6000_CSH;
//    spl06_CSH;
    SPIM_CSLow();
    SPI_RW(DATA);
    SPIM_CSHigh();
}

void cc2500_readRegs(u8 reg,  u8 *data, u8 length)
{
    u8 count = 0;
//    mpu6000_CSH;
//    spl06_CSH;
    SPIM_CSLow();
    SPI_RW(reg | 0x80);
    for (count = 0; count < length; count++)
    {
        data[count] = SPI_RW(0xff);
    }
    SPIM_CSHigh();
}


void cc2500ReadFifo(uint8_t *dpbuffer, uint8_t len)
{
    cc2500_readRegs(CC2500_3F_RXFIFO | CC2500_READ_BURST, dpbuffer, len);
}


/***********************************************************************************
**������Ϣ ��cc2500Strobe(uint8_t address)
**�������� : cc2500ѡͨĳ���Ĵ���
**������� ��address �Ĵ�����ַ
**������� ����
************************************************************************************/
void cc2500Strobe(uint8_t address)
{
//    mpu6000_CSH;
//    spl06_CSH;
    SPIM_CSLow();
    cc2500_write(address);
    SPIM_CSHigh();
}

/***********************************************************************************
**������Ϣ ��uint8_t cc2500Reset(void)
**�������� : cc2500��λ
**������� ����
**������� ���Ƿ�λ�ɹ�
************************************************************************************/
uint8_t cc2500Reset(void)
{
    cc2500Strobe(CC2500_SRES);                        //����оƬ
    delay_ms(1);    //1000us
    return (cc2500_readReg(CC2500_0E_FREQ1) == 0xC4); //0x0E��FREQ1-Ƶ�ʿ��ƴʻ㣬�м��ֽڣ���λֵΪ0xc4
}


void rxSpiWriteMulti(const uint8_t data[], uint8_t length)
{
    for (uint8_t i = 0; i < length ; i++)
    {
        cc2500_write(data[i]);
        delay_us(1);
    }
}

void rxSpiWriteCommandMulti(uint8_t command, const uint8_t *data, uint8_t length)
{
    SPIM_CSLow();
    cc2500_write(command);
    rxSpiWriteMulti(data, length);
    SPIM_CSHigh();
}

void cc2500WriteFifo(uint8_t *dpbuffer, uint8_t len)
{
    cc2500Strobe(CC2500_SFTX); // 0x3B SFTX
    rxSpiWriteCommandMulti(CC2500_3F_TXFIFO | CC2500_WRITE_BURST,
                           dpbuffer, len);
    cc2500Strobe(CC2500_STX); // 0x35
}


void cc2500SetPower(uint8_t power)
{
    const uint8_t patable[8] =
    {
        0xC5,    // -12dbm
        0x97,    // -10dbm
        0x6E,    // -8dbm
        0x7E,    // -6dbm
        0xA9,    // -4dbm
        0xBB,    // -2dbm
        0xFE,    // 0dbm
        0xFF,    // 1.5dbm
    };
    if (power > 7)
        power = 7;
    cc2500_writeReg(CC2500_3E_PATABLE, patable[power]);
}

/***********************************************************************************
**������Ϣ ��cc2500_init(void)
**�������� : cc2500��ʼ��
**������� ����
**������� ����

D8/D16/LBT�ĳ�ʼ���в��ּĴ������ò�ͬ

************************************************************************************/
void cc2500ApplyRegisterConfig(const cc2500RegisterConfigElement_t *configArrayPtr, int configSize)
{
    const int entryCount = configSize / sizeof(cc2500RegisterConfigElement_t);
    for (int i = 0; i < entryCount; i++)
    {
        cc2500_writeReg(configArrayPtr->registerID, configArrayPtr->registerValue);
        configArrayPtr++;
    }
}


void cc2500_init(void)
{
    cc2500Reset();

    cc2500ApplyRegisterConfig(cc2500FrskyBaseConfig, sizeof(cc2500FrskyBaseConfig));

    switch (spiProtocol) {
    case RX_SPI_FRSKY_D:
        cc2500ApplyRegisterConfig(cc2500FrskyDConfig, sizeof(cc2500FrskyDConfig));

        break;
    case RX_SPI_FRSKY_X:
        cc2500ApplyRegisterConfig(cc2500FrskyXConfig, sizeof(cc2500FrskyXConfig));

        break;
    case RX_SPI_FRSKY_X_LBT:
        cc2500ApplyRegisterConfig(cc2500FrskyXLbtConfig, sizeof(cc2500FrskyXLbtConfig));

        break;
    default:

        break;
    }

    for (unsigned c = 0; c < 0xFF; c++)
    {
        //calibrate all channels
        cc2500Strobe(CC2500_SIDLE);
        cc2500_writeReg(CC2500_0A_CHANNR, c);
        cc2500Strobe(CC2500_SCAL);
        delay_us(900); //
        calData[c][0] = cc2500_readReg(CC2500_23_FSCAL3);
        calData[c][1] = cc2500_readReg(CC2500_24_FSCAL2);
        calData[c][2] = cc2500_readReg(CC2500_25_FSCAL1);
    }
}


/***********************************************************************************
**������Ϣ ��initTuneRx(void)
**�������� : cc2500��ʼ��������
**������� ����
**������� ����
************************************************************************************/
void initTuneRx(void)
{
    cc2500_writeReg(CC2500_19_FOCCFG, 0x14);                 //0x19��FOCCFG-Ƶ�ʱ��˲�������
    timeTunedMs = millis();
    bindOffset = -126;
    cc2500_writeReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset); //0x0C��FSCTRL0-Ƶ�ʺϳ�������
    cc2500_writeReg(CC2500_07_PKTCTRL1, 0x0C);               //0x07��PKTCTRL1-���ݰ��Զ�����
    cc2500_writeReg(CC2500_18_MCSM0, 0x08);                  //0x18��MCSM0-��ͨ�ſ���״̬������

    cc2500Strobe(CC2500_SIDLE);                         //�뿪 RX/TX,�ض�Ƶ�ʺϳ������뿪��Ų�����ģʽ������
    cc2500_writeReg(CC2500_23_FSCAL3, calData[0][0]);   //0x23��FSCAL3-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_24_FSCAL2, calData[0][1]);   //0x24��FSCAL2-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_25_FSCAL1, calData[0][2]);   //0x25��FSCAL1-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_0A_CHANNR, 0);               //0x0A��CHANNR-�ŵ���
    cc2500Strobe(CC2500_SFRX);                          //��ϴ RX FIFO ����
    cc2500Strobe(CC2500_SRX);                           //���� RX������һ״̬Ϊ������ MCSM0.FS_AUTOCAL=1 ����������У׼��
}



/***********************************************************************************
**������Ϣ ��tuneRx(uint8_t *packet)
**�������� : cc2500���ý�����
**������� ��packet ���ݰ�
**������� ���Ƿ����óɹ�


�������ӽ���


************************************************************************************/
//bool tuneRx(uint8_t *packet)
//{
//    if(bindOffset >= 126)
//    {
//        bindOffset = -126;
//    }
//    if((millis() - timeTunedMs) > 50)
//    {
//        timeTunedMs = millis();
//        bindOffset += 5;
//        cc2500_writeReg(CC2500_0C_FSCTRL0,(uint8_t)bindOffset);                       //0x0C��FSCTRL0-Ƶ�ʺϳ�������
//    }
//    if(rxSpiGetExtiState())
//    {
//        uint8_t ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F; //RXBYTES��RX FIFO �е�����ͱ�����
//        if(ccLen)
//        {
//            cc2500ReadFifo(packet, ccLen);
//            if(packet[2] == 0x01)
//            {
//                uint8_t Lqi = packet[ccLen - 1] & 0x7F;
//                if(Lqi > 50)
//                {
//                    rxCc2500SpiConfigMutable.bindOffset = bindOffset;
//                    return true;
//                }
//            }
//        }
//    }
//    return false;
//}

extern uint8_t packetLength;
static bool isValidBindPacket(uint8_t *packet)
{
    if (spiProtocol == RX_SPI_FRSKY_D) {
        if (!(packet[packetLength - 1] & 0x80)) {
            return false;
        }
    }
    if ((packet[0] == packetLength - 3) && (packet[1] == 0x03) && (packet[2] == 0x01)) {
        return true;
    }

    return false;
}

bool tuneRx(uint8_t *packet, int8_t inc)
{
    if ((millis() - timeTunedMs) > 50 || bindOffset == 126 || bindOffset == -126)
    {
        timeTunedMs = millis();
        bindOffset += inc;
        cc2500_writeReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
        cc2500Strobe(CC2500_SRX);
    }
    if (rxSpiGetExtiState())
    {
        uint8_t ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen >= packetLength)
        {
            cc2500ReadFifo(packet, packetLength);
            if (isValidBindPacket(packet))
            {
                return true;
            }
        }
    }

    return false;
}
uint16_t bindState;
/***********************************************************************************
**������Ϣ ��initGetBind(void)
**�������� : cc2500��ʼ����
**������� ����
**������� ����


�������ӽ���


************************************************************************************/
void initGetBind(void)
{
    cc2500Strobe(CC2500_SIDLE);                        //�뿪 RX/TX,�ض�Ƶ�ʺϳ������뿪��Ų�����ģʽ������
    cc2500_writeReg(CC2500_23_FSCAL3, calData[0][0]);  //0x23��FSCAL3-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_24_FSCAL2, calData[0][1]);  //0x24��FSCAL2-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_25_FSCAL1, calData[0][2]);  //0x25��FSCAL1-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_0A_CHANNR, 0);              //0x0A��CHANNR-�ŵ���
    cc2500Strobe(CC2500_SFRX);                         //��ϴ RX FIFO ����
    delay_us(20); // waiting flush FIFO

    cc2500Strobe(CC2500_SRX);                          //���� RX������һ״̬Ϊ������ MCSM0.FS_AUTOCAL=1 ����������У׼��
    listLength = 0;
    bindState = 0;
}


/***********************************************************************************
**������Ϣ ��initGetBind(void)
**�������� : cc2500��ʼ����
**������� ����
**������� ����


�������ӽ���


************************************************************************************/
void initialiseData(bool inBindState)
{
    cc2500_writeReg(CC2500_0C_FSCTRL0, rxCc2500SpiConfigMutable.bindOffset);                    //0x0C��FSCTRL0-Ƶ�ʺϳ�������
    cc2500_writeReg(CC2500_18_MCSM0, 0x08);                                                     //0x18��MCSM0-��ͨ�ſ���״̬������
    cc2500_writeReg(CC2500_09_ADDR, inBindState ? 0x03 : rxCc2500SpiConfigMutable.bindTxId[0]); //0x09��ADDR-�豸��ַ
    cc2500_writeReg(CC2500_07_PKTCTRL1, 0x0D);                                                  //0x07��PKTCTRL1-���ݰ��Զ�����
    cc2500_writeReg(CC2500_19_FOCCFG, 0x16);                                                    //0x19��FOCCFG-Ƶ�ʱ��˲�������
    if (!inBindState)
    {
        cc2500_writeReg(CC2500_03_FIFOTHR,  0x0E);
    }
    delay(10);
}


/***********************************************************************************
**������Ϣ ��getBind1(uint8_t *packet)
**�������� : cc2500���а�
**������� ��packet �����ݰ�
**������� ���Ƿ�󶨳ɹ�


�������ӽ���


************************************************************************************/
bool getBind1(uint8_t *packet)
{
    // len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    // Start by getting bind packet 0 and the txid
    if (rxSpiGetExtiState())
    {
        uint8_t ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;      //RXBYTES��RX FIFO �е�����ͱ�����
        if (ccLen)
        {
            cc2500ReadFifo(packet, ccLen);
            if (packet[ccLen - 1] & 0x80)
            {
                if (packet[2] == 0x01)
                {
                    if (packet[5] == 0x00)
                    {
                        rxCc2500SpiConfigMutable.bindTxId[0] = packet[3];
                        rxCc2500SpiConfigMutable.bindTxId[1] = packet[4];
                        for (uint8_t n = 0; n < 5; n++)
                        {
                            rxCc2500SpiConfigMutable.bindHopData[packet[5] + n] = packet[6 + n];
                        }
                        rxCc2500SpiConfigMutable.rxNum = packet[12];
                        return true;
                    }
                }
            }
        }
    }
    return false;
}


/***********************************************************************************
**������Ϣ ��getBind2(uint8_t *packet)
**�������� : cc2500���а�
**������� ��packet �����ݰ�
**������� ���Ƿ�󶨳ɹ�


�������ӽ���


************************************************************************************/
bool getBind2(uint8_t *packet)
{
    if (bindIdx <= 120)
    {
        if (rxSpiGetExtiState())
        {
            uint8_t ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;    //RXBYTES��RX FIFO �е�����ͱ�����
            if (ccLen)
            {
                cc2500ReadFifo(packet, ccLen);
                if (packet[ccLen - 1] & 0x80)
                {
                    if (packet[2] == 0x01)
                    {
                        if ((packet[3] == rxCc2500SpiConfigMutable.bindTxId[0]) &&
                                (packet[4] == rxCc2500SpiConfigMutable.bindTxId[1]))
                        {
                            if (packet[5] == bindIdx)
                            {
                                for (uint8_t n = 0; n < 5; n++)
                                {
                                    if (packet[6 + n] == packet[ccLen - 3] || (packet[6 + n] == 0))
                                    {
                                        if (bindIdx >= 0x2D)
                                        {
                                            listLength = packet[5] + n;

                                            return true;
                                        }
                                    }

                                    rxCc2500SpiConfigMutable.bindHopData[packet[5] + n] = packet[6 + n];
                                }

                                bindIdx = bindIdx + 5;

                                return false;
                            }
                        }
                    }
                }
            }
        }

        return false;
    }
    else
    {
        return true;
    }
}

bool getBind(uint8_t *packet)
{
    // len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    if (rxSpiGetExtiState())
    {
        uint8_t ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen >= packetLength)
        {
            cc2500ReadFifo(packet, packetLength);
            if (isValidBindPacket(packet))
            {
                if (packet[5] == 0x00)
                {
                    rxCc2500SpiConfigMutable.bindTxId[0] = packet[3];
                    rxCc2500SpiConfigMutable.bindTxId[1] = packet[4];
                    rxCc2500SpiConfigMutable.bindTxId[2] = packet[11];
                    rxCc2500SpiConfigMutable.rxNum = packet[12];
                }
                for (uint8_t n = 0; n < 5; n++)
                {
                    rxCc2500SpiConfigMutable.bindHopData[packet[5] + n] = (packet[5] + n) >= 47 ? 0 : packet[6 + n];
                }
                bindState |= 1 << (packet[5] / 5);
                if (bindState == 0x3FF)
                {
                    listLength = 47;

                    return true;
                }
            }
        }
    }

    return false;
}


/***********************************************************************************
**������Ϣ ��nextChannel(uint8_t skip)
**�������� : cc2500ѡ����һ��ͨ��
**������� ��skip ���ֵ
**������� ����


�������ӽ���


************************************************************************************/
void nextChannel(uint8_t skip)
{
    static uint8_t channr = 0;

    channr += skip;
    while (channr >= listLength)
    {
        channr -= listLength;
    }
    cc2500Strobe(CC2500_SIDLE);                                                                  //�뿪 RX/TX,�ض�Ƶ�ʺϳ������뿪��Ų�����ģʽ������
    cc2500_writeReg(CC2500_23_FSCAL3, calData[rxCc2500SpiConfigMutable.bindHopData[channr]][0]); //0x23��FSCAL3-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_24_FSCAL2, calData[rxCc2500SpiConfigMutable.bindHopData[channr]][1]); //0x24��FSCAL2-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_25_FSCAL1, calData[rxCc2500SpiConfigMutable.bindHopData[channr]][2]); //0x25��FSCAL1-Ƶ�ʺϳ���У׼
    cc2500_writeReg(CC2500_0A_CHANNR, rxCc2500SpiConfigMutable.bindHopData[channr]);             //0x0A��CHANNR-�ŵ���

    if (spiProtocol == RX_SPI_FRSKY_D) {
        cc2500Strobe(CC2500_SFRX);
    }
}














