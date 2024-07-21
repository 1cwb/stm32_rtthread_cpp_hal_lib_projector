#pragma once
#include "mspi.hpp"
#include "stm32h7xx_hal_conf.h"
class mSpi4 : public mDev::mSpi
{
public:
    mSpi4() = delete;
    explicit mSpi4(const char* name);
    virtual ~mSpi4();
    virtual mResult init();
    virtual mResult deInit();
    inline virtual void csEnable();
    inline virtual void csDisable();
    virtual mResult write(const uint8_t* buff, size_t len);
    virtual mResult read(uint8_t* buff, size_t len);
private:
    SPI_HandleTypeDef _spixHandle;
    uint16_t _spiGpioCs;
    GPIO_TypeDef* _spiGpioGroup;
};