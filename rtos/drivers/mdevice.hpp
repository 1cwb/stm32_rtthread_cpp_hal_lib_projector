#pragma once
#include "mplatform.hpp"
#include <functional>
namespace mDev
{
    using initCallbackExt = std::function<void(bool benable)>;
class mDevice
{
    using interruptCallback = std::function<void(mDevice*)>;
public:
    explicit mDevice(const char* name,const initCallbackExt& cb):_devname(name),_initcb(cb)
    {
        mPlatform::getInstance()->registerDevice(_devname,this);
    }
    virtual ~mDevice() = default;
    mDevice(const mDevice&) = delete;
    mDevice(mDevice&&) = delete;
    mDevice& operator=(const mDevice&) = delete;
    mDevice& operator=(mDevice&&) = delete;
    const char* getDeviceName() const {return _devname;}

    virtual mResult init(){if(_initcb){_initcb(true);}return M_RESULT_EOK;}
    virtual mResult deInit(){if(_initcb){_initcb(false);}return M_RESULT_EOK;}

    void registerInterruptCb(const interruptCallback& cb){_cb = cb;}
    void unregisterInterrupt(){_cb = interruptCallback();}
    void runInterruptCb(){if(_cb)_cb(this);}
protected:
    const char* _devname = nullptr;
private:
    initCallbackExt _initcb;
    interruptCallback _cb;
};
}