#pragma once
#include "crsfprotocol.hpp"
#include "crc.hpp"
#include "rtoscommon.hpp"
#include <functional>
//#define CRSF_PORT_OPTIONS       (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
//#define CRSF_PORT_MODE          MODE_RXTX

#define CRSF_MAX_CHANNEL        16
#define CRSFV3_MAX_CHANNEL      24

#define CRSF_SUBSET_RC_STARTING_CHANNEL_BITS        5
#define CRSF_SUBSET_RC_STARTING_CHANNEL_MASK        0x1F
#define CRSF_SUBSET_RC_RES_CONFIGURATION_BITS       2
#define CRSF_SUBSET_RC_RES_CONFIGURATION_MASK       0x03
#define CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS  1

#define CRSF_RC_CHANNEL_SCALE_LEGACY                0.62477120195241f
#define CRSF_SUBSET_RC_RES_CONF_10B                 0
#define CRSF_SUBSET_RC_RES_BITS_10B                 10
#define CRSF_SUBSET_RC_RES_MASK_10B                 0x03FF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_10B            1.0f
#define CRSF_SUBSET_RC_RES_CONF_11B                 1
#define CRSF_SUBSET_RC_RES_BITS_11B                 11
#define CRSF_SUBSET_RC_RES_MASK_11B                 0x07FF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_11B            0.5f
#define CRSF_SUBSET_RC_RES_CONF_12B                 2
#define CRSF_SUBSET_RC_RES_BITS_12B                 12
#define CRSF_SUBSET_RC_RES_MASK_12B                 0x0FFF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_12B            0.25f
#define CRSF_SUBSET_RC_RES_CONF_13B                 3
#define CRSF_SUBSET_RC_RES_BITS_13B                 13
#define CRSF_SUBSET_RC_RES_MASK_13B                 0x1FFF
#define CRSF_SUBSET_RC_CHANNEL_SCALE_13B            0.125f

#define CRSF_RSSI_MIN (-130)
#define CRSF_RSSI_MAX 0
#define CRSF_SNR_MIN (-30)
#define CRSF_SNR_MAX 20

struct crsfFrameDef_t {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
};

union crsfFrame_t {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrameDef_t frame;
};

struct crsfPayloadRcChannelsPacked_t {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

class crsf
{
public:
    static crsf* getInstance(void)
    {
        static crsf instance;
        return &instance;
    }
    void writeTelemetryData(const void *data, uint32_t len);
    void sendTelemetryData(void);
    bool isTelemetryBufEmpty(void);
    void bind(void);
    mResult rxDataParse(uint8_t* data, int len);
    mResult packRcChannels(uint8_t frameType, uint8_t startChannel = 0, 
        uint8_t numChannels = CRSF_MAX_CHANNEL,
        CRSF_ADDRESS address = CRSF_ADDRESS_FLIGHT_CONTROLLER);
    mResult unpackRcChannels(void);
    float readRawRC(uint8_t chan);
    template <typename T>
    void registerUartSend(T* instance, mResult (T::*method)(const uint8_t*, uint32_t)) {
        uartSend = [instance, method](const uint8_t* d, uint32_t l) {
            return (instance->*method)(d, l); 
        };
    }
    uint32_t* getRxChannelData(void)
    {
        return crsfRxChannelData;
    }
    uint32_t* getTxChannelData(void)
    {
        return crsfTxChannelData;
    }
    float getChannelScale() const
    {
        return channelScale;
    }
    uint8_t getPacketLength(void)
    {
        return crsfFrame.frame.frameLength + 2;
    }
    crsfFrame_t* getFrame(void)
    {
        return &crsfFrame;
    }
    uint8_t getResolution(void) const
    {
        return resolution;
    }
    void setResolution(uint8_t res)
    {
        resolution = res;
    }
    uint8_t getResolutionBits(void) const
    {
        switch(resolution) {
        case CRSF_SUBSET_RC_RES_CONF_10B: return 10;
        case CRSF_SUBSET_RC_RES_CONF_11B: return 11;
        case CRSF_SUBSET_RC_RES_CONF_12B: return 12;
        case CRSF_SUBSET_RC_RES_CONF_13B: return 13;
        default: return 11;
        }
    }
private:
    crsf(){};
    ~crsf(){};
    uint8_t crsfFrameCRC(void)
    {
        // CRC includes type and payload
        std::uint8_t crc;

        crc = SOFTCRC::Calculate(&crsfFrame.frame.type, 1, SOFTCRC::CRC_8());
        for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
            crc = SOFTCRC::Calculate(&crsfFrame.frame.payload[ii], 1, SOFTCRC::CRC_8(), crc);
        }

        return crc;
    }
    bool crsfFrameDone = false;
    float channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;
    uint8_t resolution = CRSF_SUBSET_RC_RES_CONF_11B;
    crsfFrame_t crsfFrame;
    uint32_t crsfRxChannelData[CRSF_MAX_CHANNEL];
    uint32_t crsfTxChannelData[CRSF_MAX_CHANNEL];
    uint8_t telemetryBuf[CRSF_FRAME_SIZE_MAX];
    uint8_t telemetryBufLen = 0;
    std::function<mResult(const uint8_t*, uint32_t)> uartSend;
};
