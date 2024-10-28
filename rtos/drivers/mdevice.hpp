#pragma once
#include "mplatform.hpp"
#include <functional>
#include <string>
namespace mDev
{
    using initCallbackExt = std::function<void(bool binit)>;
class mDevice
{
    using interruptCallback = std::function<void(mDevice*)>;
public:
    explicit mDevice(const char* name):_devname(name)
    {
        mPlatform::getInstance()->registerDevice(_devname.c_str(),this);
    }
    virtual ~mDevice()
    {
        mPlatform::getInstance()->unregisterDevice(_devname.c_str());
    }
    mDevice(const mDevice&) = delete;
    mDevice(mDevice&&) = delete;
    mDevice& operator=(const mDevice&) = delete;
    mDevice& operator=(mDevice&&) = delete;
    const char* getDeviceName() const {return _devname.c_str();}

    void registerInterruptCb(const interruptCallback& cb){_cb = cb;}
    void unregisterInterrupt(){_cb = interruptCallback();}
    void runInterruptCb(){if(_cb)_cb(this);}
    void runInitCallback(bool binit){if(_initcb)_initcb(binit);}
protected:
    std::string _devname;
    initCallbackExt _initcb;
private:
    interruptCallback _cb;
};
}