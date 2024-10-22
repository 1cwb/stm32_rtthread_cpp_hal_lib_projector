#pragma once
#include "mdevice.hpp"
#include "mipc.hpp"

namespace mDev
{
enum I2C_TYPE
{
    I2C_TYPE_MASTER,
    I2C_TYPE_SLAVE
};
class mI2c : public mDevice
{
public:
    mI2c() = delete;
    explicit mI2c(const char* name, I2C_TYPE type = I2C_TYPE_MASTER) : mDevice(name), _type(type){_sem.init(getDeviceName(),1,IPC_FLAG_FIFO);}
    virtual ~mI2c() {_sem.detach();};
    mResult write(uint16_t slaveAddr, const uint8_t* buff, size_t len)
    {
        mResult ret = M_RESULT_ERROR;
        _sem.semTake(WAITING_FOREVER);
        _write(slaveAddr, buff, len);
        _sem.semRelease();
        return ret;
    }
    mResult read(uint16_t slaveAddr, uint8_t* buff, size_t len)
    {
        mResult ret = M_RESULT_ERROR;
        _sem.semTake(WAITING_FOREVER);
        _read(slaveAddr, buff, len);
        _sem.semRelease();
        return ret;
    }
    mResult writeReg(uint16_t slaveAddr, uint8_t reg, const uint8_t* buff, size_t len)
    {
        mResult ret = M_RESULT_ERROR;
        _sem.semTake(WAITING_FOREVER);
        _writeReg(slaveAddr, reg, buff, len);
        _sem.semRelease();
        return ret;
    }
    mResult readReg(uint16_t slaveAddr, uint8_t reg, uint8_t* buff, size_t len)
    {
        mResult ret = M_RESULT_ERROR;
        _sem.semTake(WAITING_FOREVER);
        _readReg(slaveAddr, reg, buff, len);
        _sem.semRelease();
        return ret;
    }
    bool isMasterMode() const
    {
        return _type == I2C_TYPE_MASTER;
    }
    void enableDma(bool benable) {_benableDMA = benable;}
    void enableISR(bool benable) {_benableISR = benable;}
    bool isEnableDMA() const {return _benableDMA;}
    bool isEnableISR() const {return _benableISR;}
protected:
    virtual mResult _write(uint16_t slaveAddr, const uint8_t* buff, size_t len){return M_RESULT_EOK;}
    virtual mResult _read(uint16_t slaveAddr, uint8_t* buff, size_t len){return M_RESULT_EOK;}
    virtual mResult _writeReg(uint16_t slaveAddr, uint8_t reg, const uint8_t* buff, size_t len){return M_RESULT_EOK;}
    virtual mResult _readReg(uint16_t slaveAddr, uint8_t reg, uint8_t* buff, size_t len) {return M_RESULT_EOK;}

    I2C_TYPE _type = I2C_TYPE_MASTER;
    bool _benableISR;
    bool _benableDMA;
private:
    mSemaphore _sem;
};
}