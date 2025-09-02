#pragma once
#include "mdevice.hpp"
#include "mgpiodrv.hpp"
namespace mDev
{
class mSpi : public mDevice
{
public:
    mSpi() = delete;
    explicit mSpi(const char* name) : mDevice(name),_transferMode(transferMode::TRANSFER_MODE_NOMAL),_recvMode(recvMode::RECV_MODE_NOMAL){}
    virtual ~mSpi() = default;

    mResult write(const uint8_t* buff, size_t len, mGpio* cspin = nullptr)
    {
        mResult ret = M_RESULT_ERROR;
        _csEnable(cspin);
        ret = _write(buff, len);
        _csDisable(cspin);
        return ret;
    }
    mResult read(uint8_t* buff, size_t len, mGpio* cspin = nullptr)
    {
        mResult ret = M_RESULT_ERROR;
        _csEnable(cspin);
        ret = _read(buff, len);
        _csDisable(cspin);
        return ret;
    }
    mResult transfer(uint8_t* tx, uint8_t* rx, size_t len, mGpio* cspin = nullptr)
    {
        mResult ret = M_RESULT_ERROR;
        _csEnable(cspin);
        ret = _transfer(tx, rx, len);
        _csDisable(cspin);
        return ret;
    }
    mResult writeReg(uint8_t reg, const uint8_t* buff, size_t len, mDev::mGpio* cspin)
    {
        mResult ret = M_RESULT_ERROR;
        _csEnable(cspin);
        reg &= 0x7F;
        do{
            ret = _write(&reg, 1);
            if(ret != M_RESULT_EOK)
            {
                break;
            }
            ret = _write(buff, len);
            if(ret != M_RESULT_EOK)
            {
                break;
            }
        }while(0);
        _csDisable(cspin);
        return ret;
    }
    mResult readReg(uint8_t reg, uint8_t* buff, size_t len, mDev::mGpio* cspin)
    {
        mResult ret = M_RESULT_ERROR;
        reg |= 0x80;
        _csEnable(cspin);
        do{
            ret = _write(&reg, 1);
            if(ret != M_RESULT_EOK)
            {
                break;
            }
            ret = _read(buff, len);
            if(ret != M_RESULT_EOK)
            {
                break;
            }
        }while(0);
        _csDisable(cspin);
        return ret; 
    }
    void setTransferMode(transferMode mode) {_transferMode = mode;}
    void setRecvMode(recvMode mode) {_recvMode = mode;}
    recvMode getRecvMode() const {return _recvMode;}
    virtual bool btransferComplete() = 0;
protected:
    void _csEnable(mGpio* cspin) {if(cspin){cspin->setLevel(mGpio::GPIOLEVEL::LEVEL_LOW);}};
    void _csDisable(mGpio* cspin) {if(cspin){cspin->setLevel(mGpio::GPIOLEVEL::LEVEL_HIGH);}};
    virtual mResult _write(const uint8_t* buff, size_t len) = 0;
    virtual mResult _read(uint8_t* buff, size_t len) = 0;
    virtual mResult _transfer(uint8_t* tx, uint8_t* rx, size_t len) = 0;
protected:
    transferMode _transferMode;
    recvMode _recvMode;
    //volatile bool _transferComplete;
};
}