#pragma once
#include "mdevice.hpp"

namespace mDev
{
enum CHANNLE
{
    CHANNLE_INVALED = 0X00,
    CHANNEL_1 = 0X01,
    CHANNEL_2 = 0X02,
    CHANNEL_3 = 0X04,
    CHANNEL_4 = 0X08,
    CHANNEL_5 = 0X10,
    CHANNEL_6 = 0X20,
    CHANNEL_ALL = 0X40,
};
enum TIMEMODE
{
    TIMEMODE_BASE_TIM,
    TIMEMODE_PWM,
    TIMEMODE_IC,
    TIMEMODE_ENCODER
};
enum TIMESTARTMODE
{
    TIMESTARTMODE_NOMAL,
    TIMESTARTMODE_IT,
    TIMESTARTMODE_DMA
};
class mTimer : public mDevice
{
public:
    mTimer(const char* name):mDevice(name){}
    ~mTimer() = default;
    virtual void updateFreq(uint32_t timefreq){}
    virtual uint32_t getFreq() const {return _timeFreq;}
    virtual uint32_t getTimeOutUs() {return 0;}
    virtual void start(mDev::CHANNLE channel = mDev::CHANNLE::CHANNLE_INVALED, uint32_t* data1 = nullptr, uint32_t* data2 = nullptr, size_t len = 0){}
    virtual void stop(mDev::CHANNLE channel){}
    virtual void pwmUpdatePulse(uint32_t pulse, CHANNLE channel){}
    virtual uint32_t pwmGetMaxPulse(){return 0;}
    virtual uint32_t pwmGetCurPulse(mDev::CHANNLE channel){return 0;}
    virtual void pwmSetDutyCycle(float dutyCycle, CHANNLE channel){}
    virtual float pwmGetDutyCycle(mDev::CHANNLE channel){return 0;}
    bool isInited() {return _btimerInited;}
protected:
    uint32_t _timeFreq = 0;
    bool _btimerInited = false;
    TIMEMODE _mode = TIMEMODE_BASE_TIM;
    TIMESTARTMODE _startMode = TIMESTARTMODE_NOMAL;
};
}