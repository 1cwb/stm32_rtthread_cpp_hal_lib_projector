#pragma once
#include "mdevice.hpp"
namespace mDev
{
class mLed : public mDevice
{
public: 
    explicit mLed(const char* name) : mDevice(name){}
    ~mLed() = default;
    virtual void on(){}
    virtual void off(){}
    virtual void toggle(){}
private:
};
}