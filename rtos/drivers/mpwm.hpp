#pragma once
#include "mdevice.hpp"
#include "mtimer.hpp"
namespace mDev
{
class mPwm : public mDevice
{
public:
    mPwm(const char* name, mTimer* timer) : mDevice(name),_timer(timer){}
    virtual ~mPwm() = default;
    virtual void start(){}
    virtual void stop(){}
    virtual void updatePulse(uint32_t pulse){}
    virtual uint32_t getMaxPulse(){return 0;}
    virtual uint32_t getCurPulse(){return 0;}
    virtual void setDutyCycle(float dutyCycle){}
    virtual float getDutyCycle(){return 0;} 
    void updateFreq(uint32_t freq)
    {
        if(_timer)
        {
            _timer->updateFreq(freq);
        }
    }
    uint32_t getFreq()
    {
        if(_timer)
        {
            return _timer->getFreq();
        }
        return 0;
    }
    mTimer* getTimer() 
    {
        return _timer;
    }
private:
    mTimer* _timer = nullptr;
};

}