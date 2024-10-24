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
    virtual int16_t getAccelX(){return 0;};
    virtual int16_t getAccelY(){return 0;};
    virtual int16_t getAccelZ(){return 0;};
    virtual int16_t getGyroX(){return 0;};
    virtual int16_t getGyroY(){return 0;};
    virtual int16_t getGyroZ(){return 0;};
    virtual float getAccelXms2(){return 0.0f;};
    virtual float getAccelYms2(){return 0.0f;};
    virtual float getAccelZms2(){return 0.0f;};
    virtual float getGyroXrad(){return 0.0f;};
    virtual float getGyroYrad(){return 0.0f;};
    virtual float getGyroZrad(){return 0.0f;};
    virtual bool updateData(){return true;}
    virtual float getYaw() {return 0.0f;}
    virtual float getPitch() {return 0.0f;}
    virtual float getRoll() {return 0.0f;}
private:

};
}