/**
******************************************************************************
* @file    cc2500.h
* @author
* @version V0.0.1
* @date    22/06/2020
* @brief   头文件，cc2500硬件相关函数声明.
******************************************************************************
*/



#include "hardware.h"
#include "stdbool.h"



#define CC2500_SRES 0x30
#define CC2500_SIDLE 0x36
#define CC2500_SCAL 0x33
#define CC2500_SAFC 0x37
#define CC2500_SWOR 0x38
#define CC2500_SPWD 0x39
#define CC2500_SFRX 0x3A
#define CC2500_SFTX 0x3B
#define CC2500_SRX  0x34
#define CC2500_STX  0x35

#define CC2500_WRITE_SINGLE 0x00
#define CC2500_WRITE_BURST 0x40
#define CC2500_READ_SINGLE 0x80
#define CC2500_READ_BURST 0xC0




enum
{
    CC2500_00_IOCFG2 = 0x00,   // GDO2 output pin configuration
    CC2500_01_IOCFG1 = 0x01,   // GDO1 output pin configuration
    CC2500_02_IOCFG0 = 0x02,   // GDO0 output pin configuration
    CC2500_03_FIFOTHR = 0x03,  // RX FIFO and TX FIFO thresholds
    CC2500_04_SYNC1 = 0x04,    // Sync word, high byte
    CC2500_05_SYNC0 = 0x05,    // Sync word, low byte
    CC2500_06_PKTLEN = 0x06,   // Packet length
    CC2500_07_PKTCTRL1 = 0x07, // Packet automation control
    CC2500_08_PKTCTRL0 = 0x08, // Packet automation control
    CC2500_09_ADDR = 0x09,     // Device address
    CC2500_0A_CHANNR = 0x0A,   // Channel number
    CC2500_0B_FSCTRL1 = 0x0B,  // Frequency synthesizer control
    CC2500_0C_FSCTRL0 = 0x0C,  // Frequency synthesizer control
    CC2500_0D_FREQ2 = 0x0D,    // Frequency control word, high byte
    CC2500_0E_FREQ1 = 0x0E,    // Frequency control word, middle byte
    CC2500_0F_FREQ0 = 0x0F,    // Frequency control word, low byte
    CC2500_10_MDMCFG4 = 0x10,  // Modem configuration
    CC2500_11_MDMCFG3 = 0x11,  // Modem configuration
    CC2500_12_MDMCFG2 = 0x12,  // Modem configuration
    CC2500_13_MDMCFG1 = 0x13,  // Modem configuration
    CC2500_14_MDMCFG0 = 0x14,  // Modem configuration
    CC2500_15_DEVIATN = 0x15,  // Modem deviation setting
    CC2500_16_MCSM2 = 0x16,    // Main Radio Cntrl State Machine config
    CC2500_17_MCSM1 = 0x17,    // Main Radio Cntrl State Machine config
    CC2500_18_MCSM0 = 0x18,    // Main Radio Cntrl State Machine config
    CC2500_19_FOCCFG = 0x19,   // Frequency Offset Compensation config
    CC2500_1A_BSCFG = 0x1A,    // Bit Synchronization configuration
    CC2500_1B_AGCCTRL2 = 0x1B, // AGC control
    CC2500_1C_AGCCTRL1 = 0x1C, // AGC control
    CC2500_1D_AGCCTRL0 = 0x1D, // AGC control
    CC2500_1E_WOREVT1 = 0x1E,  // High byte Event 0 timeout
    CC2500_1F_WOREVT0 = 0x1F,  // Low byte Event 0 timeout
    CC2500_20_WORCTRL = 0x20,  // Wake On Radio control
    CC2500_21_FREND1 = 0x21,   // Front end RX configuration
    CC2500_22_FREND0 = 0x22,   // Front end TX configuration
    CC2500_23_FSCAL3 = 0x23,   // Frequency synthesizer calibration
    CC2500_24_FSCAL2 = 0x24,   // Frequency synthesizer calibration
    CC2500_25_FSCAL1 = 0x25,   // Frequency synthesizer calibration
    CC2500_26_FSCAL0 = 0x26,   // Frequency synthesizer calibration
    CC2500_27_RCCTRL1 = 0x27,  // RC oscillator configuration
    CC2500_28_RCCTRL0 = 0x28,  // RC oscillator configuration
    CC2500_29_FSTEST = 0x29,   // Frequency synthesizer cal control
    CC2500_2A_PTEST = 0x2A,    // Production test
    CC2500_2B_AGCTEST = 0x2B,  // AGC test
    CC2500_2C_TEST2 = 0x2C,    // Various test settings
    CC2500_2D_TEST1 = 0x2D,    // Various test settings
    CC2500_2E_TEST0 = 0x2E,    // Various test settings

    // Status registers
    CC2500_30_PARTNUM = 0x30,    // Part number
    CC2500_31_VERSION = 0x31,    // Current version number
    CC2500_32_FREQEST = 0x32,    // Frequency offset estimate
    CC2500_33_LQI = 0x33,        // Demodulator estimate for link quality
    CC2500_34_RSSI = 0x34,       // Received signal strength indication
    CC2500_35_MARCSTATE = 0x35,  // Control state machine state
    CC2500_36_WORTIME1 = 0x36,   // High byte of WOR timer
    CC2500_37_WORTIME0 = 0x37,   // Low byte of WOR timer
    CC2500_38_PKTSTATUS = 0x38,  // Current GDOx status and packet status
    CC2500_39_VCO_VC_DAC = 0x39, // Current setting from PLL cal module
    CC2500_3A_TXBYTES = 0x3A,    // Underflow and # of bytes in TXFIFO
    CC2500_3B_RXBYTES = 0x3B,    // Overflow and # of bytes in RXFIFO

    // Multi byte memory locations
    CC2500_3E_PATABLE = 0x3E,
    CC2500_3F_TXFIFO = 0x3F,
    CC2500_3F_RXFIFO = 0x3F
};

typedef struct cc2500RegisterConfigElement_s
{
    uint8_t registerID;
    uint8_t registerValue;
} cc2500RegisterConfigElement_t;

typedef struct rxCc2500SpiConfig_s
{
    uint8_t autoBind;
    uint8_t bindTxId[3];
    int8_t  bindOffset;
    uint8_t bindHopData[50];
    uint8_t rxNum;
    uint8_t a1Source;
    uint8_t chipDetectEnabled;
    uint8_t txEnIoTag;
    uint8_t lnaEnIoTag;
    uint8_t antSelIoTag;
} rxCc2500SpiConfig_t;

void cc2500WriteFifo(uint8_t *dpbuffer, uint8_t len);

extern rxCc2500SpiConfig_t rxCc2500SpiConfigMutable;

void cc2500_writeReg(u8 REG, u8 DATA);

void cc2500_write(u8 DATA);

u8 cc2500_readReg(u8 reg);

bool getBind2(uint8_t *packet);

bool getBind1(uint8_t *packet);

void initialiseData(bool inBindState);

void initGetBind(void);

bool tuneRx(uint8_t *packet, int8_t inc);

bool rxSpiGetExtiState(void);

void initTuneRx(void);

void cc2500_init(void);

void nextChannel(uint8_t skip);

void cc2500ReadFifo(uint8_t *dpbuffer, uint8_t len);

void cc2500Strobe(uint8_t address);

void cc2500SetPower(uint8_t power);

bool getBind(uint8_t *packet);

