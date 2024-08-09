#pragma once
#include "mspi.hpp"
#include "stm32h7xx_hal_conf.h"
#include "gpio.hpp"
class spix : public mDev::mSpi
{
public:
    spix() = delete;
    explicit spix(const char* name);
    virtual ~spix();
    mResult init(const mDev::initCallbackExt& cb ,SPI_HandleTypeDef* spihandle ,GPIO_TypeDef* csgpiox, uint16_t cspin);
    mResult deInit();
    inline virtual void csEnable()override;
    inline virtual void csDisable()override;
    virtual mResult write(const uint8_t* buff, size_t len)override;
    virtual mResult read(uint8_t* buff, size_t len)override;
    SPI_HandleTypeDef* spixHandle()
    {
        return &_spixHandle;
    }
    SPI_HandleTypeDef _spixHandle;
private:
    
    gpiox _spiCs;
};
