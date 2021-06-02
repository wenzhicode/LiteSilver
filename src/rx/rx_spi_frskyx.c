/**
******************************************************************************
* @file    rx_spi_frskyx.c
* @author
* @version V0.0.1
* @date    22/06/2020
* @brief   RX文件，frsky D16相关函数.
******************************************************************************
*/


#include "rx_spi_frskyx.h"
#include "rx_spi.h"
#include "stdbool.h"
#include "time.h"
#include "cc2500.h"
#include "defines.h"


uint32_t packetTimerUs;
bool frameReceived;
int32_t receiveDelayUs;


extern uint8_t protocolState;
extern uint32_t start_time;
extern uint8_t packetLength;
extern uint16_t telemetryDelayUs;
extern uint32_t missingPackets;
extern int32_t timeoutUs;
extern float rx[4];
extern char aux[16];


extern uint8_t listLength;

static uint8_t frame[20];

static uint16_t calculateCrc(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0;
    for (unsigned i = 0; i < len; i++)
    {
        crc = (crc << 8) ^ (crcTable[((uint8_t)(crc >> 8) ^ *data++) & 0xFF]);
    }
    return crc;
}


static bool isValidPacket(const uint8_t *packet)
{
    uint16_t lcrc = calculateCrc(&packet[3], (packetLength - 7));
    if ((lcrc >> 8) == packet[packetLength - 4] && (lcrc & 0x00FF) == packet[packetLength - 3] &&
            (packet[0] == packetLength - 3) &&
            (packet[1] == rxCc2500SpiConfigMutable.bindTxId[0]) &&
            (packet[2] == rxCc2500SpiConfigMutable.bindTxId[1]) &&
            (packet[3] == rxCc2500SpiConfigMutable.bindTxId[2]) &&
            (rxCc2500SpiConfigMutable.rxNum == 0 || packet[6] == 0 || packet[6] == rxCc2500SpiConfigMutable.rxNum))
    {
        return true;
    }
    return false;
}


/***********************************************************************************
**函数信息 ：frSkyXSetRcData(uint16_t *rcData, const uint8_t *packet)
**功能描述 : frsky D16协议数据解析
**输入参数 ：无
**输出参数 ：无
************************************************************************************/
void frSkyXSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    uint16_t c[8];
    // ignore failsafe packet
    if (packet[7] != 0)
    {
        return;
    }
    c[0] = (uint16_t)((packet[10] << 8) & 0xF00) | packet[9];
    c[1] = (uint16_t)((packet[11] << 4) & 0xFF0) | (packet[10] >> 4);
    c[2] = (uint16_t)((packet[13] << 8) & 0xF00) | packet[12];
    c[3] = (uint16_t)((packet[14] << 4) & 0xFF0) | (packet[13] >> 4);
    c[4] = (uint16_t)((packet[16] << 8) & 0xF00) | packet[15];
    c[5] = (uint16_t)((packet[17] << 4) & 0xFF0) | (packet[16] >> 4);
    c[6] = (uint16_t)((packet[19] << 8) & 0xF00) | packet[18];
    c[7] = (uint16_t)((packet[20] << 4) & 0xFF0) | (packet[19] >> 4);

    for (unsigned i = 0; i < 8; i++)
    {
        const bool channelIsShifted = c[i] & 0x800;
        const uint16_t channelValue = c[i] & 0x7FF;
        rcData[channelIsShifted ? i + 8 : i] = ((channelValue - 64) * 2 + 860 * 3) / 3;
        //rcData[channelIsShifted ? i + 8 : i] = channelValue;
    }
}

static void buildTelemetryFrame(uint8_t *packet)
{
    static uint8_t localPacketId;

    static bool evenRun = false;

    frame[0] = 0x0E;//length
    frame[1] = rxCc2500SpiConfigMutable.bindTxId[0];
    frame[2] = rxCc2500SpiConfigMutable.bindTxId[1];
    frame[3] = rxCc2500SpiConfigMutable.bindTxId[2];

    frame[4] = 0;
    frame[5] = 0;
    frame[6] = 0;
    frame[7] = 0;
    frame[8] = 0;
    frame[9] = 0;
    frame[10] = 0;
    frame[11] = 0;
    frame[12] = 0;

    uint16_t lcrc = calculateCrc(&frame[3], 10);
    frame[13] = lcrc >> 8;
    frame[14] = lcrc;
}

/***********************************************************************************
**函数信息 ：frSkyXHandlePacket(uint8_t * const packet, uint8_t * const protocolState)
**功能描述 : frsky D16接收过程
**输入参数 ：无
**输出参数 ：无
************************************************************************************/
rx_spi_received_e frSkyXHandlePacket(uint8_t *const packet, uint8_t *const protocolState)
{
    static bool skipChannels = true;
    static uint8_t channelsToSkip = 1;
    static uint32_t packetErrors = 0;
    static bool telemetryReceived = false;
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
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
        packetTimerUs = gettime();
        *protocolState = STATE_DATA;
        frameReceived = false;
        receiveDelayUs = 5300;
        break;

    case STATE_DATA:
        if (rxSpiGetExtiState() && (frameReceived == false))
        {
            uint8_t cclen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
//            if(cclen >= packetLength)
//            {
//                cc2500ReadFifo(packet,packetLength);
//                if(isValidPacket(packet))
//                {
//                    missingPackets = 0;
//                    timeoutUs = 1;
//                    receiveDelayUs = 0;
//                    if(skipChannels)
//                    {
//                        channelsToSkip = packet[5] << 2;
//                        if(packet[4] >= listLength)
//                        {
//                            if(packet[4] < (64 + listLength))
//                            {
//                                channelsToSkip += 1;
//                            }
//                            else if(packet[4] < (128 + listLength))
//                            {
//                                channelsToSkip += 2;
//                            }
//                            else if(packet[4] < (192 + listLength))
//                            {
//                                channelsToSkip += 3;
//                            }
//                        }
//                        telemetryReceived = true;
//                        skipChannels = false;
//                    }
//                    packetTimerUs = gettime();
//                    frameReceived = true;
//                }
//                if(!frameReceived)
//                {
//                    packetErrors++;
//                    cc2500Strobe(CC2500_SFRX);
//                }
//            }

            if (cclen >= packetLength)
            {
                cc2500ReadFifo(packet, packetLength);
                if (isValidPacket(packet))
                {
                    missingPackets = 0;
                    timeoutUs = 1;
                    receiveDelayUs = 0;
                    if (skipChannels)
                    {
                        channelsToSkip = (packet[5] << 2) | (packet[4] >> 6);
                        telemetryReceived = true; // now telemetry can be sent
                        skipChannels = false;
                    }
                    cc2500setRssiDbm(packet[packetLength - 2]);

                    packetTimerUs = gettime();
                    frameReceived = true; // no need to process frame again.
                }
                if (!frameReceived)
                {
                    packetErrors++;
                    cc2500Strobe(CC2500_SFRX);
                }
            }
        }
        if (telemetryReceived)
        {
            if (cmpTimeUs(gettime(), packetTimerUs) > receiveDelayUs)
            {
                *protocolState = STATE_TELEMETRY;
//                buildTelemetryFrame(packet);
            }
        }
        if (cmpTimeUs(gettime(), packetTimerUs) > timeoutUs * SYNC_DELAY_MAX)
        {
            nextChannel(1);
            cc2500Strobe(CC2500_SRX);

            *protocolState = STATE_UPDATE;
        }
        if (frameReceived)
        {
            ret |= RX_SPI_RECEIVED_DATA;
        }
        break;
    case STATE_TELEMETRY:
        if (cmpTimeUs(gettime(), packetTimerUs) >= receiveDelayUs + telemetryDelayUs)    //53mm   +    400us
        {
            cc2500Strobe(CC2500_SIDLE);
            cc2500SetPower(6);
            cc2500Strobe(CC2500_SFRX);
            delay_us(30);
            cc2500Strobe(CC2500_SIDLE);
//            cc2500WriteFifo(frame,frame[0] + 1);
            *protocolState = STATE_RESUME;
        }
        break;
    case STATE_RESUME:
        if (cmpTimeUs(gettime(), packetTimerUs) > receiveDelayUs + 3700)
        {
            packetTimerUs = gettime();
            receiveDelayUs = 5300;
            frameReceived = false; // again set for receive
            nextChannel(channelsToSkip);
            cc2500Strobe(CC2500_SRX);
            if (missingPackets > MAX_MISSING_PKT)
            {
                timeoutUs = 5;
                skipChannels = true;
                telemetryReceived = false;
                *protocolState = STATE_UPDATE;
                break;
            }
            missingPackets++;
            *protocolState = STATE_DATA;
        }
        break;
    }//end switch
    return ret;
}





