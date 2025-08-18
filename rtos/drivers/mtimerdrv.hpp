#pragma once
#include "mdevice.hpp"

namespace mDev
{
class mTimer : public mDevice
{
public:
    mTimer(const char* name) : mDevice(name) {}
    virtual ~mTimer() = default;
    virtual void updateFreq(uint32_t timefreq)=0;
    virtual uint32_t getFreq() = 0;
    virtual uint32_t getTimeOutUs() =0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual bool isInitialized() = 0;
};
}