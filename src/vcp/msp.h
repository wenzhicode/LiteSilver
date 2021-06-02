/**
******************************************************************************
* @file    msp.h
* @author
* @version V0.0.1
* @date    8/06/2020
* @brief   头文件，msp相关函数声明.
******************************************************************************
*/


#include "hardware.h"
#include "stdbool.h"
#include "streambuf.h"
#include "string.h"

#define MAX_MANUFACTURER_ID_LENGTH 4
#define MAX_BOARD_NAME_LENGTH 20
#define SIGNATURE_LENGTH 32


#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_FIRMWARE_IDENTIFIER      "BTFL"
#define FC_VERSION_MAJOR            4  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            2  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed

#define MSP_PORT_INBUF_SIZE 192
#define MSP_PORT_OUTBUF_SIZE 256

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAYEND(x) (&(x)[ARRAYLEN(x)])

#define JUMBO_FRAME_SIZE_LIMIT 255


typedef int mspDescriptor_t;

typedef struct __attribute__((packed))
{
    uint16_t size;
}
mspHeaderJUMBO_t;

#define MSP_VERSION_MAGIC_INITIALIZER { 'M', 'M', 'X' }

// return positive for ACK, negative on error, zero for no reply
typedef enum
{
    MSP_RESULT_ACK = 1,
    MSP_RESULT_ERROR = -1,
    MSP_RESULT_NO_REPLY = 0,
    MSP_RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
} mspResult_e;


typedef enum
{
    MSP_DIRECTION_REPLY = 0,
    MSP_DIRECTION_REQUEST = 1
} mspDirection_e;

typedef struct mspPacket_s
{
    sbuf_t buf;
    int16_t cmd;
    uint8_t flags;
    int16_t result;
    uint8_t direction;
} mspPacket_t;

typedef enum
{
    // ARM flag
    BOXARM = 0,
    // FLIGHT_MODE
    BOXANGLE,
    BOXHORIZON,
    BOXMAG,
    BOXHEADFREE,
    BOXPASSTHRU,
    BOXFAILSAFE,
    BOXGPSRESCUE,
    BOXID_FLIGHTMODE_LAST = BOXGPSRESCUE,

// When new flight modes are added, the parameter group version for 'modeActivationConditions' in src/main/fc/rc_modes.c has to be incremented to ensure that the RC modes configuration is reset.

    // RCMODE flags
    BOXANTIGRAVITY,
    BOXHEADADJ,
    BOXCAMSTAB,
    BOXBEEPERON,
    BOXLEDLOW,
    BOXCALIB,
    BOXOSD,
    BOXTELEMETRY,
    BOXSERVO1,
    BOXSERVO2,
    BOXSERVO3,
    BOXBLACKBOX,
    BOXAIRMODE,
    BOX3D,
    BOXFPVANGLEMIX,
    BOXBLACKBOXERASE,
    BOXCAMERA1,
    BOXCAMERA2,
    BOXCAMERA3,
    BOXFLIPOVERAFTERCRASH,
    BOXPREARM,
    BOXBEEPGPSCOUNT,
    BOXVTXPITMODE,
    BOXPARALYZE,
    BOXUSER1,
    BOXUSER2,
    BOXUSER3,
    BOXUSER4,
    BOXPIDAUDIO,
    BOXACROTRAINER,
    BOXVTXCONTROLDISABLE,
    BOXLAUNCHCONTROL,
    CHECKBOX_ITEM_COUNT
} boxId_e;

typedef struct boxBitmask_s
{
    uint32_t bits[(CHECKBOX_ITEM_COUNT + 31) / 32];
} boxBitmask_t;


typedef enum
{
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_X,

    MSP_HEADER_V1,
    MSP_PAYLOAD_V1,
    MSP_CHECKSUM_V1,

    MSP_HEADER_V2_OVER_V1,
    MSP_PAYLOAD_V2_OVER_V1,
    MSP_CHECKSUM_V2_OVER_V1,

    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,

    MSP_COMMAND_RECEIVED
} mspState_e;

typedef enum
{
    MSP_PACKET_COMMAND,
    MSP_PACKET_REPLY
} mspPacketType_e;

typedef enum
{
    MSP_V1          = 0,
    MSP_V2_OVER_V1  = 1,
    MSP_V2_NATIVE   = 2,
    MSP_VERSION_COUNT
} mspVersion_e;


typedef struct mspPort_s
{
    struct serialPort_s *port; // null when port unused.
    uint32_t lastActivityMs;
    mspState_e c_state;
    mspPacketType_e packetType;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
    uint16_t cmdMSP;
    uint8_t cmdFlags;
    mspVersion_e mspVersion;
    uint_fast16_t offset;
    uint_fast16_t dataSize;
    uint8_t checksum1;
    uint8_t checksum2;
    bool sharedWithTelemetry;
    int descriptor;
} mspPort_t;


typedef struct __attribute__((packed))
{
    uint8_t size;
    uint8_t cmd;
}
mspHeaderV1_t;






bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c);



void mspSerialProcess(void);




