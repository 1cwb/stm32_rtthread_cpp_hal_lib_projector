#pragma once
#include "mdevice.hpp"

namespace mDev
{
enum class sensorAlign {
    CW0_DEG,
    CW90_DEG,
    CW180_DEG,
    CW270_DEG,
    CW0_DEG_FLIP,
    CW90_DEG_FLIP,
    CW180_DEG_FLIP,
    CW270_DEG_FLIP
};

class mImu : public mDevice
{
public:
    explicit mImu(const char* name) : mDevice(name){}
    virtual ~mImu() = default;
    virtual float getTemp() = 0;
    virtual int16_t getAccelX() = 0;
    virtual int16_t getAccelY() = 0;
    virtual int16_t getAccelZ() = 0;
    virtual int16_t getGyroX() = 0;
    virtual int16_t getGyroY() = 0;
    virtual int16_t getGyroZ() = 0;
    virtual float getAccelXms2() = 0;
    virtual float getAccelYms2() = 0;
    virtual float getAccelZms2() = 0;
    virtual float getGyroXrad() = 0;
    virtual float getGyroYrad() = 0;
    virtual float getGyroZrad() = 0;
    virtual bool updateData() = 0;
    void rotation(float *v, sensorAlign align)
    {
        float tmp[3] = {v[0], v[1], v[2]};
        switch (align)
        {
            case sensorAlign::CW0_DEG:
                v[0] = tmp[0]; v[1] = tmp[1]; v[2] = tmp[2];
                break;
            case sensorAlign::CW90_DEG:
                v[0] = tmp[1]; v[1] = -tmp[0]; v[2] = tmp[2];
                break;
            case sensorAlign::CW180_DEG:
                v[0] = -tmp[0]; v[1] = -tmp[1]; v[2] = tmp[2];
                break;
            case sensorAlign::CW270_DEG:
                v[0] = -tmp[1]; v[1] = tmp[0]; v[2] = tmp[2];
                break;
            case sensorAlign::CW0_DEG_FLIP:
                v[0] = -tmp[0]; v[1] = tmp[1]; v[2] = -tmp[2];
                break;
            case sensorAlign::CW90_DEG_FLIP:
                v[0] = tmp[1]; v[1] = tmp[0]; v[2] = -tmp[2];
                break;
            case sensorAlign::CW180_DEG_FLIP:
                v[0] = tmp[0]; v[1] = -tmp[1]; v[2] = -tmp[2];
                break;
            case sensorAlign::CW270_DEG_FLIP:
                v[0] = -tmp[1]; v[1] = -tmp[0]; v[2] = -tmp[2];
                break;
            default:
                break;
        }
    }
private:

};
}