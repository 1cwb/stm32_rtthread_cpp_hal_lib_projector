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
    mDevice(name){};
    virtual ~mAdc(){};
    virtual mResult start(recvMode mode, uint32_t* value, uint32_t len) = 0;
    virtual mResult stop() = 0;
    virtual mResult read(uint32_t* value) = 0;
    //void setRecvMode(recvMode mode) {_recvMode = mode;}
    //recvMode getRecvMode() const {return _recvMode;}
    bool btransferComplete()  {return _transferComplete;}
    void setTransferComplete(bool bcomplete) {_transferComplete = bcomplete;}
protected:
    recvMode _recvMode;
    volatile bool _transferComplete;
};
}