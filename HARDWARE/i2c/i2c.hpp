#pragma once
#include "mi2cdrv.hpp"
#include "stm32h7xx_hal_conf.h"
class i2cx : public mDev::mI2c
{
public:
    i2cx() = delete;
    explicit i2cx(const char* name, mDev::I2C_TYPE type = mDev::I2C_TYPE::I2C_TYPE_MASTER);
    virtual ~i2cx();
    mResult init(const mDev::initCallbackExt& cb ,I2C_HandleTypeDef* i2chandle);
    mResult deInit();
    virtual mResult write(uint16_t slaveAddr, const uint8_t* buff, size_t len)override;
    virtual mResult read(uint16_t slaveAddr, uint8_t* buff, size_t len)override;
    virtual mResult writeReg(uint16_t slaveAddr, uint8_t reg, const uint8_t* buff, size_t len)override;
    virtual mResult readReg(uint16_t slaveAddr, uint8_t reg, uint8_t* buff, size_t len)override;
    I2C_HandleTypeDef* i2cxHandle()
    {
        return &_i2cxHandle;
    }
    I2C_HandleTypeDef _i2cxHandle;
private:
};
