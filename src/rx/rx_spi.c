
/**
******************************************************************************
* @file    rx_spi.c
* @author
* @version V0.0.1
* @date    22/06/2020
* @brief   rx_spi文件，rx_spi相关函数.
******************************************************************************
*/



#include "rx_spi.h"
#include "cc2500.h"
#include "time.h"
#include "flash.h"
#include "defines.h"
#include "maths.h"

rx_spi_protocol_e spiProtocol;

uint8_t protocolState;
uint32_t start_time;
uint8_t packetLength;
uint16_t telemetryDelayUs;
uint32_t missingPackets;
int32_t timeoutUs;

extern uint8_t packet[35];
extern uint16_t rcData[18];
extern int rxmode;

handlePacketFn *handlePacket;
processFrameFn *processFrame;
setRcDataFn *setRcData;

extern rx_spi_received_e frSkySpiDataReceived(uint8_t *packet);
extern void frSkyXSetRcData(uint16_t *rcData, const uint8_t *packet);
extern void frSkyDSetRcData(uint16_t *rcData, const uint8_t *packet);
extern rx_spi_received_e frSkyDHandlePacket(uint8_t *const packet, uint8_t *const protocolState);
extern rx_spi_received_e frSkyXHandlePacket(uint8_t *const packet, uint8_t *const protocolState);


/*


注意此处  有引脚初始化


*/
bool rxSpiGetExtiState(void)
{
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);


    if ((GPIOD->IDR & GPIO_Pin_0) != (uint32_t)Bit_RESET)
    {
        return true;
    }
    else
    {
        return false;
    }
}


bool rxSpiCheckBindRequested(void)
{
    if ((GPIOB->IDR & GPIO_Pin_8) != (uint32_t)Bit_RESET)
    {
        return true;
    }
    else
    {
        return false;
    }

}
int8_t bindOffset, bindOffset_min, bindOffset_max;

rx_spi_received_e frSkySpiDataReceived(uint8_t *packet)
{
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
    switch (protocolState)
    {
    case STATE_INIT:
        if ((millis() - start_time) > 10)
        {
            cc2500_init();
            protocolState = STATE_BIND;
        }
        break;
//    case STATE_BIND:
//        if(rxCc2500SpiConfigMutable.autoBind)
//        {
//            initTuneRx();
//            protocolState = STATE_BIND_TUNING;
//        }
//        else
//        {
//            protocolState = STATE_STARTING;
//        }
//        break;
    case STATE_BIND:
        if (rxCc2500SpiConfigMutable.autoBind)
        {
            initTuneRx();

            protocolState = STATE_BIND_TUNING_LOW;
        }
        else
        {
            protocolState = STATE_STARTING;
        }

        break;
//    case STATE_BIND_TUNING:
//        if(tuneRx(packet))
//        {
//            initGetBind();
//            initialiseData(true);
//            protocolState = STATE_BIND_BINDING1;
//        }
//        break;
//    case STATE_BIND_BINDING1:
//        if(getBind1(packet))
//        {
//            protocolState = STATE_BIND_BINDING2;
//        }
//        break;
//    case STATE_BIND_BINDING2:
//        if(getBind2(packet))
//        {
//            cc2500Strobe(CC2500_SIDLE);
//            rxCc2500SpiConfigMutable.autoBind = false;
//            protocolState = STATE_BIND_COMPLETE;
//        }
//        break;
    case STATE_BIND_TUNING_LOW:
        if (tuneRx(packet, 2))
        {
            bindOffset_min = bindOffset;
            bindOffset = 126;

            protocolState = STATE_BIND_TUNING_HIGH;
        }

        break;
    case STATE_BIND_TUNING_HIGH:
        if (tuneRx(packet, -2))
        {
            bindOffset_max = bindOffset;
            bindOffset = ((int16_t)bindOffset_max + (int16_t)bindOffset_min) / 2;
            rxCc2500SpiConfigMutable.bindOffset = bindOffset;
            initGetBind();
            initialiseData(true);

            if (bindOffset_min < bindOffset_max)
                protocolState = STATE_BIND_BINDING;
            else
                protocolState = STATE_BIND;
        }

        break;
    case STATE_BIND_BINDING:
        if (getBind(packet))
        {
            cc2500Strobe(CC2500_SIDLE);

            rxCc2500SpiConfigMutable.autoBind = false;
            protocolState = STATE_BIND_COMPLETE;
        }

        break;
    case STATE_BIND_COMPLETE:
        if (!rxCc2500SpiConfigMutable.autoBind)
        {
            flash_save();
        }
        else
        {
            uint8_t ctr = 80;
            while (ctr--)
            {
                delay_ms(50);
            }
        }
        rxmode = !RXMODE_BIND;
        ret = RX_SPI_RECEIVED_BIND;
        protocolState = STATE_STARTING;
        break;
    default:
        ret = handlePacket(packet, &protocolState);
        break;
    }
    return ret;
}


void frSkyDInit(const rx_spi_protocol_e Protocol)
{
    packetLength = 20;

}


void frSkyXInit(const rx_spi_protocol_e Protocol)
{
    switch (Protocol)
    {
    case RX_SPI_FRSKY_X:
        packetLength = 32;
        telemetryDelayUs = 400;
        break;
    case RX_SPI_FRSKY_X_LBT:
        packetLength = 35;
        telemetryDelayUs = 1400;
        break;
    default:
        break;
    }
#if defined(USE_TELEMETRY_SMARTPORT)
    if (featureIsEnabled(FEATURE_TELEMETRY))
    {
        telemetryEnabled = initSmartPortTelemetryExternal(frSkyXTelemetryWriteFrame);
    }
#endif
}


bool frSkySpiInit(rx_spi_protocol_e Protocol)
{
    spiProtocol = Protocol;
    switch (Protocol)
    {
    case RX_SPI_FRSKY_D:
        handlePacket = frSkyDHandlePacket;
        setRcData = frSkyDSetRcData;
        frSkyDInit(Protocol);

        break;
    case RX_SPI_FRSKY_X:
    case RX_SPI_FRSKY_X_LBT:
        handlePacket = frSkyXHandlePacket;
        setRcData = frSkyXSetRcData;
        frSkyXInit(Protocol);

        break;
    default:

        break;
    }

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
    if (rssiSource == RSSI_SOURCE_NONE)
    {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }
#endif

    missingPackets = 0;
    timeoutUs = 50;

    start_time = millis();
    protocolState = STATE_INIT;

    return true;
}

uint16_t rssi = 0;
int16_t rssiDbm;
#define RSSI_SAMPLE_COUNT 16
uint16_t updateRssiSamples(uint16_t value)
{
    static uint16_t samples[RSSI_SAMPLE_COUNT];
    static uint8_t sampleIndex = 0;
    static unsigned sum = 0;

    sum += value - samples[sampleIndex];
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % RSSI_SAMPLE_COUNT;
    return sum / RSSI_SAMPLE_COUNT;
}

void setRssi(uint16_t rssiValue)
{
    rssi = updateRssiSamples(rssiValue);
}

void cc2500setRssiDbm(uint8_t value)
{
    if (value >= 128)
    {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) - 82;
    }
    else
    {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) + 65;
    }

    setRssi(rssiDbm << 3);
}
#define RSSI_MAX_VALUE 1023
uint8_t rssip = 0;

uint8_t getRssiPercent(void)
{
    rssip = scaleRange(rssi, 0, RSSI_MAX_VALUE, 0, 100);
    return rssip;
}

uint8_t rxSpiNewPacketAvailable;

uint8_t rxSpiFrameStatus(void)
{
    uint8_t status = RX_FRAME_PENDING;

    rx_spi_received_e result = frSkySpiDataReceived(packet);

    if (result & RX_SPI_RECEIVED_DATA)
    {
        rxSpiNewPacketAvailable = true;
        status = RX_FRAME_COMPLETE;
    }

    return status;
}



