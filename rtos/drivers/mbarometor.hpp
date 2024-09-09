#pragma once
#include "mdevice.hpp"

namespace mDev
{
class mBarometor : public mDevice
{
public:
    explicit mBarometor(const char* name) : mDevice(name){}
    virtual ~mBarometor() = default;
    virtual float getTemp(){return 0.0f;};
    virtual bool updateData(){return true;}
    virtual float getPressure() {return 0.0f;}
private:

};
}