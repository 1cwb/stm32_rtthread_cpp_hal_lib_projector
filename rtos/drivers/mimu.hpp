#pragma once
#include "mdevice.hpp"

namespace mDev
{
class mImu : public mDevice
{
public:
    explicit mImu(const char* name) : mDevice(name){}
    virtual ~mImu() = default;
    virtual float getTemp(){return 0.0f;};
    virtual float getAccelX(){return 0.0f;};
    virtual float getAccelY(){return 0.0f;};
    virtual float getAccelZ(){return 0.0f;};
    virtual float getGyroX(){return 0.0f;};
    virtual float getGyroY(){return 0.0f;};
    virtual float getGyroZ(){return 0.0f;};
    virtual bool updateData(){return true;}
    virtual float getYaw() {return 0.0f;}
    virtual float getPitch() {return 0.0f;}
    virtual float getRoll() {return 0.0f;}
private:

};
}