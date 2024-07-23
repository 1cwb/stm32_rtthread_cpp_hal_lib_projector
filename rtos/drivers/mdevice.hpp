#pragma once
#include "mplatform.hpp"
namespace mDev
{
class mDevice
{
public:
    mDevice(const char* name):_devname(name)
    {
        mPlatform::getInstance()->registerDevice(_devname,this);
    }
    virtual ~mDevice() = default;
    mDevice(const mDevice&) = delete;
    mDevice(mDevice&&) = delete;
    mDevice& operator=(const mDevice&) = delete;
    mDevice& operator=(mDevice&&) = delete;
    const char* getDeviceName() const {return _devname;}

    virtual mResult init(){return M_RESULT_EOK;}
    virtual mResult deInit(){return M_RESULT_EOK;}
protected:
    const char* _devname = nullptr;
private:
};
}