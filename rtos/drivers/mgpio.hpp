#pragma once
#include "mdevice.hpp"
#include <functional>
namespace mDev
{
class mGpio : public mDevice
{
public:
    enum GPIOLEVEL
    {
        LEVEL_LOW = 0,
        LEVEL_HIGH = 1
    };

    explicit mGpio(const char* name) : mDevice(name) {}
    virtual ~mGpio() = default;
    virtual void setLevel(GPIOLEVEL level){}
    virtual GPIOLEVEL getLevel(){return LEVEL_LOW;}
    virtual void toggle(){}
    virtual mResult interruptEnable(bool benable){return M_RESULT_EOK;}
private:
    
};
}