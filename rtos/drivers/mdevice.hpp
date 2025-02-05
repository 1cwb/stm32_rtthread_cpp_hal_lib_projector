#pragma once
#include "mplatform.hpp"
#include <functional>
#include <string>
namespace mDev
{
template<typename T, typename TYPE = int32_t>
struct devCbData
{
    TYPE type;
    T data;
    uint32_t len;
};
    using initCallbackExt = std::function<void(bool binit)>;
class mDevice
{
    using interruptCallback = std::function<void(mDevice*, void*)>;
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
    void unregisterInterrupt(){_cb = nullptr;}
    void runInterruptCb(void* p){if(_cb)_cb(this,p);}
    void runInitCallback(bool binit){if(_initcb)_initcb(binit);}
protected:
    std::basic_string<char, std::char_traits<char>, mMemAllocator<char>>  _devname;
    initCallbackExt _initcb;
private:
    interruptCallback _cb;
};
}