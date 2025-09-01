#pragma once
#include "mdevice.hpp"
namespace mDev
{
    class mGyroscope : public mDevice
    {
    public:
        explicit mGyroscope(const char* name) : mDevice(name){}
        virtual ~mGyroscope() = default;
        virtual float getX() = 0;
        virtual float getY() = 0;
        virtual float getZ() = 0;
        virtual float getXms2() = 0;
        virtual float getYms2() = 0;
        virtual float getZms2() = 0;
        virtual void update() = 0;
    };
} // namespace mDev