#pragma once
#include "mdevice.hpp"
namespace mDev
{
class mSpi : public mDevice
{
public:
    mSpi() = delete;
    explicit mSpi(const char* name) : mDevice(name){}
    virtual ~mSpi() = default;

    virtual mResult init(){return M_RESULT_EOK;}
    virtual mResult deInit(){return M_RESULT_EOK;}
    inline virtual void csEnable(){}
    inline virtual void csDisable(){}
    virtual mResult write(const uint8_t* buff, size_t len){return M_RESULT_EOK;}
    virtual mResult read(uint8_t* buff, size_t len){return M_RESULT_EOK;}
    mResult writeReg(uint8_t reg, const uint8_t* buff, size_t len);
    mResult readReg(uint8_t reg, uint8_t* buff, size_t len);
private:

};
}