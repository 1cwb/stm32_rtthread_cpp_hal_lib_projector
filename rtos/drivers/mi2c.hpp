#pragma once
#include "mdevice.hpp"
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
    explicit mI2c(const char* name, I2C_TYPE type = I2C_TYPE_MASTER) : mDevice(name), _type(type){}
    virtual ~mI2c() = default;
    virtual mResult write(uint16_t slaveAddr, const uint8_t* buff, size_t len){return M_RESULT_EOK;}
    virtual mResult read(uint16_t slaveAddr, uint8_t* buff, size_t len){return M_RESULT_EOK;}
    virtual mResult writeReg(uint16_t slaveAddr, uint8_t reg, const uint8_t* buff, size_t len){return M_RESULT_EOK;}
    virtual mResult readReg(uint16_t slaveAddr, uint8_t reg, uint8_t* buff, size_t len) {return M_RESULT_EOK;}
    bool isMasterMode() const
    {
        return _type == I2C_TYPE_MASTER;
    }
protected:
    I2C_TYPE _type = I2C_TYPE_MASTER;
};
}