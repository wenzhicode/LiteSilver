/**
******************************************************************************
* @file    msp.c
* @author
* @version V0.0.1
* @date    8/06/2020
* @brief   msp文件，msp相关函数.
******************************************************************************

电调上位机（blheli suit等）需要先用msp 协议与飞控通信，确认是否进行电调更新等操作

上位机与电调连接后，不再使用msp协议，传输文件时使用xmodem协议。

建立连接的过程参考《/docs/电调.md》，飞控主要的工作是为电调程序和上位机之间建立连接，
电调和上位机建立连接后，就不关飞控的事了。建立连接也就是《电调.md》中的那几个通信过程


*/


#include "msp.h"
#include "stdint.h"
#include "time.h"
#include "msp_protocol.h"
#include "string.h"
#include "usbprop.h"
#include "serial_4way.h"

const char *const targetName = "Lite";
const char *const shortGitRevision = "20208";
const char *const buildDate = __DATE__;
const char *const buildTime = __TIME__;

mspPort_t MspPort;
typedef void (*mspPostProcessFnPtr)(void);
typedef mspResult_e(*mspProcessCommandFnPtr)(mspDescriptor_t srcDesc, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn);
typedef void (*mspProcessReplyFnPtr)(mspPacket_t *cmd);

/********************************************************************************
**函数信息 ：mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
**功能描述 ：解析msp协议
**输入参数 ：mspPort msp消息结构体，c 接收到的字符
**输出参数 ：是否解析到完整的一帧数据
*********************************************************************************/
bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    switch (mspPort->c_state)
    {
    default:
    case MSP_IDLE:      // Waiting for '$' character
        if (c == '$')
        {
            mspPort->c_state = MSP_HEADER_START;
        }
        else
        {
            return false;
        }
        break;

    case MSP_HEADER_START:  // Waiting for 'M' (MSPv1 / MSPv2_over_v1) or 'X' (MSPv2 native)
        mspPort->offset = 0;
        mspPort->checksum1 = 0;
        switch (c)
        {
        case 'M':
            mspPort->c_state = MSP_HEADER_M;
            mspPort->mspVersion = MSP_V1;
            break;
        default:
            mspPort->c_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_M:      // Waiting for '<' / '>'
        mspPort->c_state = MSP_HEADER_V1;
        switch (c)
        {
        case '<':
            mspPort->packetType = MSP_PACKET_COMMAND;
            break;
        case '>':
            mspPort->packetType = MSP_PACKET_REPLY;
            break;
        default:
            mspPort->c_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_V1:     // Now receive v1 header (size/cmd), this is already checksummable
        mspPort->inBuf[mspPort->offset++] = c;
        mspPort->checksum1 ^= c;
        if (mspPort->offset == sizeof(mspHeaderV1_t))
        {
            mspHeaderV1_t *hdr = (mspHeaderV1_t *)&mspPort->inBuf[0];
            // Check incoming buffer size limit
            if (hdr->size > MSP_PORT_INBUF_SIZE)
            {
                mspPort->c_state = MSP_IDLE;
            }
            else
            {
                mspPort->dataSize = hdr->size;
                mspPort->cmdMSP = hdr->cmd;
                mspPort->cmdFlags = 0;
                mspPort->offset = 0;                // re-use buffer
                mspPort->c_state = mspPort->dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
            }
        }
        break;

    case MSP_PAYLOAD_V1:
        mspPort->inBuf[mspPort->offset++] = c;
        mspPort->checksum1 ^= c;
        if (mspPort->offset == mspPort->dataSize)
        {
            mspPort->c_state = MSP_CHECKSUM_V1;
        }
        break;

    case MSP_CHECKSUM_V1:
        if (mspPort->checksum1 == c)
        {
            mspPort->c_state = MSP_COMMAND_RECEIVED;
        }
        else
        {
            mspPort->c_state = MSP_IDLE;
        }
        break;
    }

    return true;
}

static uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len)
{
    while (len-- > 0)
    {
        checksum ^= *data++;
    }
    return checksum;
}

void serialWriteBuf(const uint8_t *data, int count)
{
    delay_us(200);
    UsbVcomSend(data, count);
}


static int mspSerialSendFrame(mspPort_t *msp, const uint8_t *hdr, int hdrLen, const uint8_t *data, int dataLen, const uint8_t *crc, int crcLen)
{
    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    const int totalFrameLength = hdrLen + dataLen + crcLen;

    // Transmit frame
    serialWriteBuf(hdr, hdrLen);
    serialWriteBuf(data, dataLen);
    serialWriteBuf(crc, crcLen);

    return totalFrameLength;
}


static int mspSerialEncode(mspPort_t *msp, mspPacket_t *packet, mspVersion_e mspVersion)
{
    static const uint8_t mspMagic[MSP_VERSION_COUNT] = MSP_VERSION_MAGIC_INITIALIZER;
    const int dataLen = sbufBytesRemaining(&packet->buf);
    uint8_t hdrBuf[16] = { '$', mspMagic[mspVersion], packet->result == MSP_RESULT_ERROR ? '!' : '>'};
    uint8_t crcBuf[2];
    uint8_t checksum;
    int hdrLen = 3;
    int crcLen = 0;

#define V1_CHECKSUM_STARTPOS 3
    if (mspVersion == MSP_V1)
    {
        mspHeaderV1_t *hdrV1 = (mspHeaderV1_t *)&hdrBuf[hdrLen];
        hdrLen += sizeof(mspHeaderV1_t);
        hdrV1->cmd = packet->cmd;

        // Add JUMBO-frame header if necessary
        if (dataLen >= JUMBO_FRAME_SIZE_LIMIT)
        {
            mspHeaderJUMBO_t *hdrJUMBO = (mspHeaderJUMBO_t *)&hdrBuf[hdrLen];
            hdrLen += sizeof(mspHeaderJUMBO_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = dataLen;
        }
        else
        {
            hdrV1->size = dataLen;
        }

        // Pre-calculate CRC
        checksum = mspSerialChecksumBuf(0, hdrBuf + V1_CHECKSUM_STARTPOS, hdrLen - V1_CHECKSUM_STARTPOS);
        checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);
        crcBuf[crcLen++] = checksum;
    }
    // Send the frame
    return mspSerialSendFrame(msp, hdrBuf, hdrLen, sbufPtr(&packet->buf), dataLen, crcBuf, crcLen);
}


static bool mspCommonProcessOutCommand(int16_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    switch (cmdMSP)
    {
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;
    case MSP_FC_VARIANT:
        sbufWriteData(dst, FC_FIRMWARE_IDENTIFIER, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;
    case MSP_BOARD_INFO:
    {
        sbufWriteData(dst, "BTFL", BOARD_IDENTIFIER_LENGTH);

        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.

        sbufWriteU8(dst, 0);  // 0 == FC


        // Target capabilities (uint8)
#define TARGET_HAS_VCP 0
#define TARGET_HAS_SOFTSERIAL 1
#define TARGET_IS_UNIFIED 2
#define TARGET_HAS_FLASH_BOOTLOADER 3
#define TARGET_SUPPORTS_CUSTOM_DEFAULTS 4
#define TARGET_HAS_CUSTOM_DEFAULTS 5
#define TARGET_SUPPORTS_RX_BIND 6

        uint8_t targetCapabilities = 0;

        sbufWriteU8(dst, targetCapabilities);

        // Target name with explicit length
        sbufWriteU8(dst, strlen(targetName));
        sbufWriteData(dst, targetName, strlen(targetName));

        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);


        uint8_t emptySignature[SIGNATURE_LENGTH];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));


        sbufWriteU8(dst, 1);

        // Added in API version 1.42
        sbufWriteU8(dst, 0);

        // Added in API version 1.43
        sbufWriteU16(dst, 1); // informational so the configurator can display the correct gyro/pid frequencies in the drop-down

        // Configuration warnings / problems (uint32_t)
#define PROBLEM_ACC_NEEDS_CALIBRATION 0
#define PROBLEM_MOTOR_PROTOCOL_DISABLED 1

        uint32_t configurationProblems = 0;

        sbufWriteU32(dst, configurationProblems);

        break;
    }

    case MSP_BUILD_INFO:
        sbufWriteData(dst, buildDate, 11);
        sbufWriteData(dst, buildTime, 8);
        sbufWriteData(dst, shortGitRevision, 7);
        break;

    case MSP_FEATURE_CONFIG:
        sbufWriteU32(dst, 1);
        break;

    default:
        return false;
    }
    return true;
}

static inline int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
static bool mspProcessOutCommand(int16_t cmdMSP, sbuf_t *dst)
{
    bool unsupportedCommand = false;
    switch (cmdMSP)
    {
    case MSP_STATUS_EX:
    case MSP_STATUS:
    {
        boxBitmask_t flightModeFlags;
        const int flagBits = 0;
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 0);
        sbufWriteU16(dst, 1);
        sbufWriteData(dst, &flightModeFlags, 4);
        sbufWriteU8(dst, 1);

        sbufWriteU16(dst, 30);
        sbufWriteU16(dst, 0);

        int byteCount = (flagBits - 32 + 7) / 8;
        byteCount = constrain(byteCount, 0, 15);
        sbufWriteU8(dst, byteCount);
        sbufWriteData(dst, ((uint8_t *)&flightModeFlags) + 4, byteCount);
        sbufWriteU8(dst, 10);
        sbufWriteU32(dst, 0);
        sbufWriteU8(dst, 0);
    }
    break;

    case MSP_MOTOR_3D_CONFIG:
        sbufWriteU16(dst, 1406);
        sbufWriteU16(dst, 1514);
        sbufWriteU16(dst, 1460);
        break;

    case MSP_MOTOR_CONFIG:
        sbufWriteU16(dst, 1150);
        sbufWriteU16(dst, 1850);
        sbufWriteU16(dst, 1000);

        // API 1.42
        sbufWriteU8(dst, 4);
        sbufWriteU8(dst, 14);

        sbufWriteU8(dst, 0);

        sbufWriteU8(dst, 0);
        break;

    case MSP_MOTOR:
        for (unsigned i = 0; i < 8; i++)
        {
            sbufWriteU16(dst, 0);
        }
        break;


    default:
        unsupportedCommand = true;
    }
    return !unsupportedCommand;
}

static void mspFcSetPassthroughCommand(sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    sbufWriteU8(dst, 4);

    esc4wayInit();
    *mspPostProcessFn = esc4wayProcess;

}


mspResult_e mspFcProcessCommand(mspDescriptor_t srcDesc, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const int16_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspCommonProcessOutCommand(cmdMSP, dst, mspPostProcessFn))
    {
        ret = MSP_RESULT_ACK;
    }
    else if (mspProcessOutCommand(cmdMSP, dst))
    {
        ret = MSP_RESULT_ACK;
    }
    else if (cmdMSP == MSP_SET_PASSTHROUGH)
    {
        mspFcSetPassthroughCommand(dst, src, mspPostProcessFn);
        ret = MSP_RESULT_ACK;
    }
    reply->result = ret;
    return ret;
}


mspPostProcessFnPtr mspSerialProcessReceivedCommand(mspPort_t *msp, mspProcessCommandFnPtr mspProcessCommandFn)
{

    static uint8_t outBuf[MSP_PORT_OUTBUF_SIZE];

    mspPacket_t reply =
    {
        .buf = { .ptr = outBuf, .end = ARRAYEND(outBuf), },
        .cmd = -1,
        .flags = 0,
        .result = 0,
        .direction = MSP_DIRECTION_REPLY,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command =
    {
        .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, },
        .cmd = msp->cmdMSP,
        .flags = msp->cmdFlags,
        .result = 0,
        .direction = MSP_DIRECTION_REQUEST,
    };

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    const mspResult_e status = mspProcessCommandFn(msp->descriptor, &command, &reply, &mspPostProcessFn);

    if (status != MSP_RESULT_NO_REPLY)
    {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        mspSerialEncode(msp, &reply, msp->mspVersion);
    }

    return mspPostProcessFn;
}




/********************************************************************************
**函数信息 ：mspSerialProcess(void)
**功能描述 ：解析msp协议
**输入参数 ：无
**输出参数 ：无
*********************************************************************************/
void mspSerialProcess(void)
{
    mspPort_t *const mspPort = &MspPort;
    uint8_t c;

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    int num = 0;
    num = CDC_Receive_BytesAvailable();
    if (num)
    {
        while (CDC_Receive_BytesAvailable())
        {
            UsbVcomRec(&c, 1);

            mspSerialProcessReceivedData(mspPort, c);

            if (mspPort->c_state == MSP_COMMAND_RECEIVED)
            {
                if (mspPort->packetType == MSP_PACKET_COMMAND)
                {
                    mspPostProcessFn = mspSerialProcessReceivedCommand(mspPort, mspFcProcessCommand);
                }
                else if (mspPort->packetType == MSP_PACKET_REPLY)
                {
                    //mspSerialProcessReceivedReply(mspPort, mspProcessReplyFn);
                }
            }
            if(mspPostProcessFn)
            {
                mspPostProcessFn();
            }
        }
    }
}





















