#pragma once
#include "mdevice.hpp"

namespace mDev
{
class mMagnetmetor : public mDevice
{
public:
    explicit mMagnetmetor(const char* name) : mDevice(name){}
    virtual ~mMagnetmetor() = default;
    virtual float getTemp(){return 0.0f;};
    virtual int getMageX(){return 0;};
    virtual int getMageY(){return 0;};
    virtual int getMageZ(){return 0;};
    virtual bool updateData(){return true;}
    virtual int getAzimuth() {return 0;}
private:

};
}