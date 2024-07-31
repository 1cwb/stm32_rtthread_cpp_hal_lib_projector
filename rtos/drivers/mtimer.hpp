#pragma once
#include "mdevice.hpp"

namespace mDev
{
class mTimer : public mDevice
{
public:
    mTimer(const char* name, uint32_t timefreq):mDevice(name), _timeFreq(timefreq){}
    ~mTimer() = default;
    virtual void updateFreq(uint32_t timefreq){_timeFreq = timefreq;}
    virtual uint32_t getFreq() const {return _timeFreq;}
    virtual uint32_t getTimeOut() {return 0;}
protected:
    uint32_t _timeFreq;
};
}