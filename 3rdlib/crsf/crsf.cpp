#include "crsf.hpp"


void crsf::writeTelemetryData(const void *data, uint32_t len)
{
    len = len < sizeof(telemetryBuf) ? len : sizeof(telemetryBuf);
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}

void crsf::sendTelemetryData(void)
{
    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        if (uartSend) {
            uartSend(telemetryBuf, telemetryBufLen);
        }
        telemetryBufLen = 0; // reset telemetry buffer
    }
}

bool crsf::isTelemetryBufEmpty(void)
{
    return telemetryBufLen == 0;
}
void crsf::bind(void)
{
    uint8_t bindFrame[] = {
        CRSF_SYNC_BYTE,
        0x07,  // frame length
        CRSF_FRAMETYPE_COMMAND,
        CRSF_ADDRESS_CRSF_RECEIVER,
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_COMMAND_SUBCMD_RX,
        CRSF_COMMAND_SUBCMD_RX_BIND,
        0x9E,  // Command CRC8
        0xE8,  // Packet CRC8
    };
    writeTelemetryData(bindFrame, sizeof(bindFrame));
    sendTelemetryData();
}
mResult crsf::rxDataParse(uint8_t* data, int len)
{
    if(len > CRSF_FRAME_SIZE_MAX)
    {
        len = CRSF_FRAME_SIZE_MAX;
    }
    memcpy(&crsfFrame, data, sizeof(crsfFrame_t));
    const uint8_t crc = crsfFrameCRC();
    const int fullFrameLength = crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH < CRSF_FRAME_SIZE_MAX ?
        crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH : CRSF_FRAME_SIZE_MAX;
    if (crc == crsfFrame.bytes[fullFrameLength - 1])
    {
        switch (crsfFrame.frame.type)
        {
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
                if (crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    crsfFrameDone = true;
                }
                break;
            
            default:
                break;
        }
    }
    else
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult crsf::packRcChannels(uint8_t frameType, uint8_t startChannel, 
    uint8_t numChannels,
    CRSF_ADDRESS address)
{
    crsfFrame.frame.deviceAddress = address;
    crsfFrame.frame.type = frameType;
    if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) 
    {
        this->resolution = CRSF_SUBSET_RC_RES_CONF_11B;
        // 打包传统16通道格式 (0x16)
        crsfPayloadRcChannelsPacked_t* rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfFrame.frame.payload;
        // 将通道数据拷贝到打包结构体
        rcChannels->chan0  = crsfTxChannelData[0];
        rcChannels->chan1  = crsfTxChannelData[1];
        rcChannels->chan2  = crsfTxChannelData[2];
        rcChannels->chan3  = crsfTxChannelData[3];
        rcChannels->chan4  = crsfTxChannelData[4];
        rcChannels->chan5  = crsfTxChannelData[5];
        rcChannels->chan6  = crsfTxChannelData[6];
        rcChannels->chan7  = crsfTxChannelData[7];
        rcChannels->chan8  = crsfTxChannelData[8];
        rcChannels->chan9  = crsfTxChannelData[9];
        rcChannels->chan10  = crsfTxChannelData[10];
        rcChannels->chan11  = crsfTxChannelData[11];
        rcChannels->chan12  = crsfTxChannelData[12];
        rcChannels->chan13  = crsfTxChannelData[13];
        rcChannels->chan14  = crsfTxChannelData[14];
        rcChannels->chan15  = crsfTxChannelData[15];
        crsfFrame.frame.frameLength = sizeof(crsfPayloadRcChannelsPacked_t) + CRSF_FRAME_LENGTH_TYPE_CRC;
    } 
    else if (frameType == CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED)
    {
        // 打包子集通道格式 (0x17)
        uint8_t writeByteIndex = 0;
        uint8_t* payload = crsfFrame.frame.payload;

        // 构建配置字节
        uint8_t configByte = (startChannel & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK);
        configByte |= (resolution & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK) << CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;
        configByte |= 0x00 << (CRSF_SUBSET_RC_STARTING_CHANNEL_BITS + CRSF_SUBSET_RC_RES_CONFIGURATION_BITS); // 保留位
        payload[writeByteIndex++] = configByte;

        // 获取分辨率参数
        const uint8_t channelBits = [&]{
        switch(resolution) {
        case CRSF_SUBSET_RC_RES_CONF_10B: return 10;
        case CRSF_SUBSET_RC_RES_CONF_11B: return 11;
        case CRSF_SUBSET_RC_RES_CONF_12B: return 12;
        case CRSF_SUBSET_RC_RES_CONF_13B: return 13;
        default: return 11;
        }
        }();
        const uint16_t channelMask = (1 << channelBits) - 1;

        // 位操作缓冲区
        uint32_t writeValue = 0;
        uint8_t bitsStored = 0;

        // 打包通道数据
        for (uint8_t n = 0; n < numChannels; n++) {
            uint16_t chanValue = crsfTxChannelData[startChannel + n] & channelMask;
            writeValue |= (chanValue << bitsStored);
            bitsStored += channelBits;

            while (bitsStored >= 8) {
            payload[writeByteIndex++] = writeValue & 0xFF;
            writeValue >>= 8;
            bitsStored -= 8;
            }
        }
        // 处理剩余位
        if (bitsStored > 0)
        {
            payload[writeByteIndex++] = writeValue & 0xFF;
        }

        crsfFrame.frame.frameLength = writeByteIndex + CRSF_FRAME_LENGTH_TYPE_CRC;
    }
    // 计算CRC并填充到帧尾
    uint8_t crc = crsfFrameCRC();
    crsfFrame.frame.payload[crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC] = crc;
    memset(crsfTxChannelData, 0, sizeof(crsfTxChannelData));
    return M_RESULT_EOK;
}
mResult crsf::unpackRcChannels(void)
{
    if(crsfFrameDone)
    {
        crsfFrameDone = false;
        memset(crsfRxChannelData, 0, sizeof(crsfRxChannelData));
        if(crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            const crsfPayloadRcChannelsPacked_t* const rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfFrame.frame.payload;
            channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;
            crsfRxChannelData[0] = rcChannels->chan0;
            crsfRxChannelData[1] = rcChannels->chan1;
            crsfRxChannelData[2] = rcChannels->chan2;
            crsfRxChannelData[3] = rcChannels->chan3;
            crsfRxChannelData[4] = rcChannels->chan4;
            crsfRxChannelData[5] = rcChannels->chan5;
            crsfRxChannelData[6] = rcChannels->chan6;
            crsfRxChannelData[7] = rcChannels->chan7;
            crsfRxChannelData[8] = rcChannels->chan8;
            crsfRxChannelData[9] = rcChannels->chan9;
            crsfRxChannelData[10] = rcChannels->chan10;
            crsfRxChannelData[11] = rcChannels->chan11;
            crsfRxChannelData[12] = rcChannels->chan12;
            crsfRxChannelData[13] = rcChannels->chan13;
            crsfRxChannelData[14] = rcChannels->chan14;
            crsfRxChannelData[15] = rcChannels->chan15;
        }
        else
        {
            // use subset RC frame structure (0x17)
            uint8_t readByteIndex = 0;
            const uint8_t *payload = crsfFrame.frame.payload;

            // get the configuration byte
            uint8_t configByte = payload[readByteIndex++];

            // get the channel number of start channel
            uint8_t startChannel = configByte & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
            configByte >>= CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;

            // get the channel resolution settings
            uint8_t channelBits;
            uint16_t channelMask;
            uint8_t channelRes = configByte & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK;
            resolution = channelRes;
            configByte >>= CRSF_SUBSET_RC_RES_CONFIGURATION_BITS;
            switch (channelRes)
            {
            case CRSF_SUBSET_RC_RES_CONF_10B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_10B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_10B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_10B;
                break;
            default:
            case CRSF_SUBSET_RC_RES_CONF_11B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_11B;
                break;
            case CRSF_SUBSET_RC_RES_CONF_12B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_12B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_12B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_12B;
                break;
            case CRSF_SUBSET_RC_RES_CONF_13B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_13B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_13B;
                channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_13B;
                break;
            }
            // do nothing for the reserved configuration bit
            configByte >>= CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS;

            // calculate the number of channels packed
            uint8_t numOfChannels = ((crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC - 1) * 8) / channelBits;

            // unpack the channel data
            uint8_t bitsMerged = 0;
            uint32_t readValue = 0;
            for (uint8_t n = 0; n < numOfChannels; n++) {
                while (bitsMerged < channelBits) {
                    uint8_t readByte = payload[readByteIndex++];
                    readValue |= ((uint32_t) readByte) << bitsMerged;
                    bitsMerged += 8;
                }
                crsfRxChannelData[startChannel + n] = readValue & channelMask;
                readValue >>= channelBits;
                bitsMerged -= channelBits;
            }
        }
        return M_RESULT_EOK;
    }
    return M_RESULT_ERROR;
}
float crsf::readRawRC(uint8_t chan)
{
    if (channelScale == CRSF_RC_CHANNEL_SCALE_LEGACY) {
        /* conversion from RC value to PWM
        * for 0x16 RC frame
        *       RC     PWM
        * min  172 ->  988us
        * mid  992 -> 1500us
        * max 1811 -> 2012us
        * scale factor = (2012-988) / (1811-172) = 0.62477120195241
        * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
        */
        return (channelScale * (float)crsfRxChannelData[chan]) + 881;
    } else {
        /* conversion from RC value to PWM
        * for 0x17 Subset RC frame
        */
        return (channelScale * (float)crsfRxChannelData[chan]) + 988;
    }
}