#pragma once
#include "mdevice.hpp"
namespace mDev
{
class mGpio : public mDevice
{
    explicit mGpio(const char* name) : mDevice(name) {}
    virtual ~mGpio() = default;
};
}