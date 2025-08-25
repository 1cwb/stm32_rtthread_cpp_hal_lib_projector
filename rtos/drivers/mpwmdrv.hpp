#pragma once
#include "mdevice.hpp"

namespace mDev
{
class mPWM : public mDevice
{
public:
    mPWM(const char* name) : mDev::mDevice(name){}
    virtual ~mPWM() = default;
    virtual void updateFreq(uint32_t timefreq)=0;
    virtual uint32_t getFreq() = 0;
    virtual void start()=0;
    virtual void stop()=0;
    virtual void updatePulse(uint32_t pulse)=0;
    virtual uint32_t getMaxPulse()=0;
    virtual uint32_t getCurPulse()=0;
    virtual void setDutyCycle(float dutyCycle)=0;
    virtual float getDutyCycle()=0;
};
} // namespace mDev