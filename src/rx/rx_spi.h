/**
******************************************************************************
* @file    rx_spi.h
* @author
* @version V0.0.1
* @date    22/06/2020
* @brief   头文件，rx_spi相关函数声明.
******************************************************************************
*/


#pragma once


#include "hardware.h"
#include "stdbool.h"

#define SYNC_DELAY_MAX 9000
#define MAX_MISSING_PKT 5

enum
{
    STATE_INIT = 0,
    STATE_BIND,
    STATE_BIND_TUNING_LOW,
    STATE_BIND_TUNING_HIGH,
    STATE_BIND_BINDING,
    STATE_BIND_COMPLETE,
    STATE_STARTING,
    STATE_UPDATE,
    STATE_DATA,
    STATE_TELEMETRY,
    STATE_RESUME,
};


typedef enum
{
    RX_SPI_FRSKY_D,
    RX_SPI_FRSKY_X,
    RX_SPI_SFHSS,
    RX_SPI_FRSKY_X_LBT,
    RX_SPI_PROTOCOL_COUNT
} rx_spi_protocol_e;

typedef enum
{
    RX_SPI_RECEIVED_NONE = 0,
    RX_SPI_RECEIVED_BIND = (1 << 0),
    RX_SPI_RECEIVED_DATA = (1 << 1),
    RX_SPI_ROCESSING_REQUIRED = (1 << 2),
} rx_spi_received_e;

typedef enum
{
    RX_PROVIDER_NONE = 0,
    RX_PROVIDER_PARALLEL_PWM,
    RX_PROVIDER_PPM,
    RX_PROVIDER_SERIAL,
    RX_PROVIDER_MSP,
    RX_PROVIDER_SPI,
} rxProvider_t;

struct rxRuntimeState_s;
typedef uint16_t (*rcReadRawDataFnPtr)(const struct rxRuntimeState_s *rxRuntimeState, uint8_t chan);
typedef uint8_t (*rcFrameStatusFnPtr)(struct rxRuntimeState_s *rxRuntimeState);
typedef bool (*rcProcessFrameFnPtr)(const struct rxRuntimeState_s *rxRuntimeState);
typedef u32(*rcGetFrameTimeUsFnPtr)(void);

typedef rx_spi_received_e handlePacketFn(uint8_t *const packet, uint8_t *const protocolState);
typedef rx_spi_received_e processFrameFn(uint8_t *const packet);
typedef void setRcDataFn(uint16_t *rcData, const uint8_t *payload);


extern handlePacketFn *handlePacket;
extern processFrameFn *processFrame;
extern setRcDataFn *setRcData;


typedef enum
{
    RX_FRAME_PENDING = 0,
    RX_FRAME_COMPLETE = (1 << 0),
    RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
    RX_FRAME_DROPPED = (1 << 3)
} rxFrameState_e;


typedef struct rxRuntimeState_s
{
    rxProvider_t        rxProvider;
    uint8_t             channelCount;
    uint16_t            rxRefreshRate;
    rcReadRawDataFnPtr  rcReadRawFn;
    rcFrameStatusFnPtr  rcFrameStatusFn;
    rcProcessFrameFnPtr rcProcessFrameFn;
    rcGetFrameTimeUsFnPtr rcFrameTimeUsFn;
    uint16_t            *channelData;
    void                *frameData;
} rxRuntimeState_t;

bool rxSpiGetExtiState(void);
bool rxSpiCheckBindRequested(void);


void cc2500setRssiDbm(uint8_t value);

void setRssi(uint16_t rssiValue);
uint16_t updateRssiSamples(uint16_t value);
uint8_t rxSpiFrameStatus(void);
uint8_t getRssiPercent(void);





