#pragma once
#include "mi2cdrv.hpp"
#include "stm32h7xx_hal_conf.h"
class i2cx : public mDev::mI2c
{
public:
    i2cx() = delete;
    explicit i2cx(const char* name, mDev::I2C_TYPE type = mDev::I2C_TYPE::I2C_TYPE_MASTER);
    virtual ~i2cx();
    mResult init(const mDev::initCallbackExt& cb ,I2C_HandleTypeDef* i2chandle, bool enableIsr = false, bool enableDma = false);
    mResult deInit();
    virtual mResult _write(uint16_t slaveAddr, const uint8_t* buff, size_t len)override;
    virtual mResult _read(uint16_t slaveAddr, uint8_t* buff, size_t len)override;
    virtual mResult _writeReg(uint16_t slaveAddr, uint8_t reg, const uint8_t* buff, size_t len)override;
    virtual mResult _readReg(uint16_t slaveAddr, uint8_t reg, uint8_t* buff, size_t len)override;
    I2C_HandleTypeDef* i2cxHandle()
    {
        return &_i2cxHandle;
    }
    I2C_HandleTypeDef _i2cxHandle;
private:
};
