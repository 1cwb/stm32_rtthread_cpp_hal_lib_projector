#pragma once
#include "mdevice.hpp"

namespace mDev
{
class mTimer : public mDevice
{
public:
    mTimer(const char* name):mDevice(name){}
    ~mTimer() = default;
    virtual void updateFreq(uint32_t timefreq){}
    virtual uint32_t getFreq() const {return _timeFreq;}
    virtual uint32_t getTimeOut() {return 0;}
    virtual void start(){}
    virtual void stop(){}
    bool isInited() {return _btimerInited;}
protected:
    uint32_t _timeFreq = 0;
    bool _btimerInited = false;
};
}