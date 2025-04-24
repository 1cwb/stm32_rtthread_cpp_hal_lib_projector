#pragma once

#include "mdevice.hpp"

namespace mDev
{
enum ADC_EVENT_TYPE
{
    ADC_EVNET_TYPE_CONV_COMPLETE,
    ADC_EVNET_TYPE_CONV_HALF_COMPLETE,
    ADC_EVNET_TYPE_LEVEL_OUT_OF_WINDOW,
    ADC_EVNET_TYPE_CONV_ERROR,
};
class mAdc : public mDevice
{
public:
    using usartData = devCbData<uint8_t*,ADC_EVENT_TYPE>;
    mAdc() = delete;
    mAdc(const char* name):
    mDevice(name),_recvMode(recvMode::RECV_MODE_NOMAL),_rxBuff(nullptr),_transferComplete(false)
    {_rxBuff =(new alignas(32) uint8_t[RX_BUFF_LEN]);};
    virtual ~mAdc(){if(_rxBuff) {delete[] _rxBuff;}};
    virtual mResult start(recvMode mode, uint32_t* value, uint32_t len) = 0;
    virtual mResult stop() = 0;
    virtual mResult read(uint32_t* value) = 0;
    virtual uint8_t getChannelNum() = 0;
    //void setRecvMode(recvMode mode) {_recvMode = mode;}
    //recvMode getRecvMode() const {return _recvMode;}
    bool btransferComplete()  {return _transferComplete;}
    void setTransferComplete(bool bcomplete) {_transferComplete = bcomplete;}
    uint8_t* getRxBuff() {return _rxBuff;}
public:
    constexpr static int RX_BUFF_LEN = 64;
protected:
    recvMode _recvMode;
    uint8_t* _rxBuff;
    volatile bool _transferComplete;
};
}