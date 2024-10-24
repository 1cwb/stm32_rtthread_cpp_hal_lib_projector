#pragma once
#include "mdevice.hpp"
#include "mgpiodrv.hpp"
namespace mDev
{
class mSpi : public mDevice
{
public:
    mSpi() = delete;
    explicit mSpi(const char* name) : mDevice(name){}
    virtual ~mSpi() = default;
    inline virtual void csEnable(mDev::mGpio* cspin){}
    inline virtual void csDisable(mDev::mGpio* cspin){}
    virtual mResult write(const uint8_t* buff, size_t len){return M_RESULT_EOK;}
    virtual mResult read(uint8_t* buff, size_t len){return M_RESULT_EOK;}
    mResult writeReg(mDev::mGpio* cspin, uint8_t reg, const uint8_t* buff, size_t len)
    {
        mResult ret = M_RESULT_ERROR;
        reg &= 0x7F;
        if(cspin)
        {
            csEnable(cspin);
        }
        do{
            ret = write(&reg, 1);
            if(ret != M_RESULT_EOK)
            {
                break;
            }
            ret = write(buff, len);
            if(ret != M_RESULT_EOK)
            {
                break;
            }
        }while(0);
        if(cspin)
        {
            csDisable(cspin);
        }
        return ret;
    }
    mResult readReg(mDev::mGpio* cspin, uint8_t reg, uint8_t* buff, size_t len)
    {
        mResult ret = M_RESULT_ERROR;
        reg |= 0x80;
        if(cspin)
        {
            csEnable(cspin);
        }
        do{
            ret = write(&reg, 1);
            if(ret != M_RESULT_EOK)
            {
                break;
            }
            ret = read(buff, len);
            if(ret != M_RESULT_EOK)
            {
                break;
            }
        }while(0);
        if(cspin)
        {
            csDisable(cspin);
        }
        return ret; 
    }
private:

};
}