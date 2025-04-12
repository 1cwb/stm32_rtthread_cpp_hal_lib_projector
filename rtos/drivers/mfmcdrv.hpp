#pragma once

#include "mdevice.hpp"
namespace mDev {
    class mFMC : public mDevice {
    public:
        mFMC() = delete;
        explicit mFMC(const char* name) : mDevice(name) {}
        virtual ~mFMC() = default;
        
    }
}