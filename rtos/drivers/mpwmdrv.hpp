#pragma once
#include "mdevice.hpp"

namespace mDev
{
enum mCHANNEL
{
    mCHANNEL_INVALED = 0X00,
    CHANNEL_1 = 0X01,
    CHANNEL_2 = 0X02,
    CHANNEL_3 = 0X04,
    CHANNEL_4 = 0X08,
    CHANNEL_5 = 0X10,
    CHANNEL_6 = 0X20,
    CHANNEL_ALL = 0X40,
};
class mPWM : public mDevice
{
public:
    mPWM(const char* name) : mDev::mDevice(name){}
    virtual ~mPWM() = default;
    virtual void updateFreq(uint32_t timefreq)=0;
    virtual uint32_t getFreq() = 0;
    virtual void start(mDev::mCHANNEL channel = mDev::mCHANNEL::mCHANNEL_INVALED)=0;
    virtual void stop(mDev::mCHANNEL channel)=0;
    virtual void pwmUpdatePulse(uint32_t pulse, mDev::mCHANNEL channel)=0;
    virtual uint32_t pwmGetMaxPulse()=0;
    virtual uint32_t pwmGetCurPulse(mDev::mCHANNEL channel)=0;
    virtual void pwmSetDutyCycle(float dutyCycle, mDev::mCHANNEL channel)=0;
    virtual float pwmGetDutyCycle(mDev::mCHANNEL channel)=0;
};
} // namespace mDev