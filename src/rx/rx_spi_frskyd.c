/**
******************************************************************************
* @file    rx_spi_frskyd.c
* @author
* @version V0.0.1
* @date    24/06/2020
* @brief   RX文件，frsky D8相关函数.
******************************************************************************
*/


#include "rx_spi_frskyD.h"
#include "rx_spi.h"
#include "stdbool.h"
#include "time.h"
#include "cc2500.h"


extern uint8_t listLength;
extern int32_t timeoutUs;
extern uint32_t missingPackets;


/***********************************************************************************
**函数信息 ：frSkyDSetRcData(uint16_t *rcData, const uint8_t *packet)
**功能描述 : frsky D8协议数据解析
**输入参数 ：无
**输出参数 ：无
************************************************************************************/
static void decodeChannelPair(uint16_t *channels, const uint8_t *packet, const uint8_t highNibbleOffset)
{
    channels[0] = FRSKY_D_CHANNEL_SCALING * (uint16_t)((packet[highNibbleOffset] & 0xf) << 8 | packet[0]);
    channels[1] = FRSKY_D_CHANNEL_SCALING * (uint16_t)((packet[highNibbleOffset] & 0xf0) << 4 | packet[1]);
}

void frSkyDSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    static uint16_t dataErrorCount = 0;

    uint16_t channels[RC_CHANNEL_COUNT_FRSKY_D];
    bool dataError = false;

    decodeChannelPair(channels, packet + 6, 4);
    decodeChannelPair(channels + 2, packet + 8, 3);
    decodeChannelPair(channels + 4, packet + 12, 4);
    decodeChannelPair(channels + 6, packet + 14, 3);

    for (int i = 0; i < RC_CHANNEL_COUNT_FRSKY_D; i++)
    {
        if ((channels[i] < 800) || (channels[i] > 2200))
        {
            dataError = true;

            break;
        }
    }

    if (!dataError)
    {
        for (int i = 0; i < RC_CHANNEL_COUNT_FRSKY_D; i++)
        {
            rcData[i] = channels[i];
        }
    }
    else
    {
    }
}


/***********************************************************************************
**函数信息 ：frSkyDHandlePacket(uint8_t * const packet, uint8_t * const protocolState)
**功能描述 : frsky D8接收过程
**输入参数 ：无
**输出参数 ：无
************************************************************************************/
rx_spi_received_e frSkyDHandlePacket(uint8_t *const packet, uint8_t *const protocolState)
{
    static u32 lastPacketReceivedTime = 0;
    static u32 telemetryTimeUs;

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    const u32 currentPacketReceivedTime = gettime();

    switch (*protocolState)
    {
    case STATE_STARTING:
        listLength = 47;
        initialiseData(false);
        *protocolState = STATE_UPDATE;
        nextChannel(1);
        cc2500Strobe(CC2500_SRX);

        break;

    case STATE_UPDATE:
        lastPacketReceivedTime = currentPacketReceivedTime;
        *protocolState = STATE_DATA;

//        if (rxSpiCheckBindRequested())
//        {
//            lastPacketReceivedTime = 0;
//            timeoutUs = 50;
//            missingPackets = 0;

//            *protocolState = STATE_INIT;

//            break;
//        }

    // here FS code could be
    case STATE_DATA:
        if (rxSpiGetExtiState())
        {
            uint8_t ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            bool packetOk = false;
            if (ccLen >= 20)
            {
                cc2500ReadFifo(packet, 20);
                if (packet[19] & 0x80)
                {
                    packetOk = true;
                    missingPackets = 0;
                    timeoutUs = 1;
                    if (packet[0] == 0x11)
                    {
                        if ((packet[1] == rxCc2500SpiConfigMutable.bindTxId[0]) &&
                                (packet[2] == rxCc2500SpiConfigMutable.bindTxId[1]))
                        {
                            nextChannel(1);
                            cc2500setRssiDbm(packet[18]);
                            if ((packet[3] % 4) == 2)
                            {
                                telemetryTimeUs = gettime();
                                //buildTelemetryFrame(packet);
                                *protocolState = STATE_TELEMETRY;
                            }
                            else
                            {
                                cc2500Strobe(CC2500_SRX);
                                *protocolState = STATE_UPDATE;
                            }
                            ret = RX_SPI_RECEIVED_DATA;
                            lastPacketReceivedTime = currentPacketReceivedTime;
                        }
                    }
                }
            }
            if (!packetOk)
            {
                cc2500Strobe(CC2500_SFRX);
            }
        }

        if (cmpTimeUs(currentPacketReceivedTime, lastPacketReceivedTime) > (timeoutUs * SYNC_DELAY_MAX))
        {
            if (timeoutUs == 1)
            {
                if (missingPackets > MAX_MISSING_PKT)
                {
                    timeoutUs = 5;
                }

                missingPackets++;
                nextChannel(1);
            }
            else
            {
                nextChannel(13);
            }

            cc2500Strobe(CC2500_SRX);
            *protocolState = STATE_UPDATE;
        }
        break;
    case STATE_TELEMETRY:
        if (cmpTimeUs(gettime(), telemetryTimeUs) >= 1380)
        {
            cc2500Strobe(CC2500_SIDLE);
            cc2500SetPower(6);
            cc2500Strobe(CC2500_SFRX);

            cc2500Strobe(CC2500_SIDLE);
            // cc2500WriteFifo(frame, frame[0] + 1);
            *protocolState = STATE_DATA;
            lastPacketReceivedTime = currentPacketReceivedTime;
        }

        break;
    }

    return ret;
}




