#pragma once
#include "mspidrv.hpp"
#include "stm32h7xx_hal_conf.h"
#include "gpio.hpp"
class spix : public mDev::mSpi
{
public:
    spix() = delete;
    explicit spix(const char* name);
    virtual ~spix();
    mResult init(const mDev::initCallbackExt& cb ,SPI_HandleTypeDef* spihandle);
    mResult deInit();
    inline virtual void csEnable(mDev::mGpio* cspin)override;
    inline virtual void csDisable(mDev::mGpio* cspin)override;
    virtual mResult write(const uint8_t* buff, size_t len)override;
    virtual mResult read(uint8_t* buff, size_t len)override;
    SPI_HandleTypeDef* spixHandle()
    {
        return &_spixHandle;
    }
    SPI_HandleTypeDef _spixHandle;
private:

    //mDev::mGpio* _spiCs = nullptr;
};
