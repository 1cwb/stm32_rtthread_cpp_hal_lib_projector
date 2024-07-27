#pragma once
#include "mdevice.hpp"
namespace mDev
{
class mLed : public mDevice
{
public: 
    explicit mLed(const char* name, const mDev::initCallbackExt& cb) : mDevice(name, cb){}
    ~mLed() = default;
    virtual mResult init(){return mDevice::init();}
    virtual mResult deInit(){return mDevice::deInit();}
    virtual void on(){}
    virtual void off(){}
    virtual void toggle(){}
private:
};
}